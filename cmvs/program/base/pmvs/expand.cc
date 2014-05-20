#include <algorithm>
#include <numeric>
#include <iterator>
#include "expand.h"
#include "findMatch.h"

using namespace PMVS3;
using namespace std;
using namespace Patch;

Cexpand::Cexpand(CfindMatch& findMatch) : m_fm(findMatch) {
}

void Cexpand::init(void) {
}

void Cexpand::run(void) {
  m_fm.m_count = 0;
  m_fm.m_jobs.clear();
  m_ecounts.resize(m_fm.m_CPU);
  m_fcounts0.resize(m_fm.m_CPU);
  m_fcounts1.resize(m_fm.m_CPU);
  m_pcounts.resize(m_fm.m_CPU);
  fill(m_ecounts.begin(), m_ecounts.end(), 0);
  fill(m_fcounts0.begin(), m_fcounts0.end(), 0);
  fill(m_fcounts1.begin(), m_fcounts1.end(), 0);
  fill(m_pcounts.begin(), m_pcounts.end(), 0);

  time_t starttime = time(NULL);

  m_fm.m_pos.clearCounts();
  m_fm.m_pos.clearFlags();
  
  if (!m_queue.empty()) {
    cerr << "Queue is not empty in expand" << endl;
    exit (1);
  }
  // set queue
  m_fm.m_pos.collectPatches(m_queue);

  cerr << "Expanding patches..." << flush;
  pthread_t threads[m_fm.m_CPU];
  for (int c = 0; c < m_fm.m_CPU; ++c)
    pthread_create(&threads[c], NULL, expandThreadTmp, (void*)this);
  for (int c = 0; c < m_fm.m_CPU; ++c)
    pthread_join(threads[c], NULL); 
  
  cerr << endl
       << "---- EXPANSION: " << (time(NULL) - starttime) << " secs ----" << endl;

  const int trial = accumulate(m_ecounts.begin(), m_ecounts.end(), 0);
  const int fail0 = accumulate(m_fcounts0.begin(), m_fcounts0.end(), 0);
  const int fail1 = accumulate(m_fcounts1.begin(), m_fcounts1.end(), 0);
  const int pass = accumulate(m_pcounts.begin(), m_pcounts.end(), 0);
  cerr << "Total pass fail0 fail1 refinepatch: "
       << trial << ' ' << pass << ' '
       << fail0 << ' ' << fail1 << ' ' << pass + fail1 << endl;
  cerr << "Total pass fail0 fail1 refinepatch: "
       << 100 * trial / (float)trial << ' '
       << 100 * pass / (float)trial << ' '
       << 100 * fail0 / (float)trial << ' '
       << 100 * fail1 / (float)trial << ' '
       << 100 * (pass + fail1) / (float)trial << endl;
  
}
void* Cexpand::expandThreadTmp(void* arg) {
  ((Cexpand*)arg)->expandThread();
  return NULL;
}

void Cexpand::expandThread(void) {
  pthread_rwlock_wrlock(&m_fm.m_lock);
  const int id = m_fm.m_count++;
  pthread_rwlock_unlock(&m_fm.m_lock);

  while (1) {
    Ppatch ppatch;
    int empty = 0;
    pthread_rwlock_wrlock(&m_fm.m_lock);
    if (m_queue.empty())
      empty = 1;
    else {
      ppatch = m_queue.top();
      m_queue.pop();
    }
    pthread_rwlock_unlock(&m_fm.m_lock);

    if (empty)
      break;
    
    // For each direction;
    vector<vector<Vec4f> > canCoords;
    findEmptyBlocks(ppatch, canCoords);

    for (int i = 0; i < (int)canCoords.size(); ++i) {
      for (int j = 0; j < (int)canCoords[i].size(); ++j) {
        const int flag = expandSub(ppatch, id, canCoords[i][j]);
        // fail
        if (flag)
          ppatch->m_dflag |= (0x0001) << i;
      }
    }
  }
}

void Cexpand::findEmptyBlocks(const Ppatch& ppatch,
			      std::vector<std::vector<Vec4f> >& canCoords) {
  // dnum must be at most 8, because m_dflag is char
  const int dnum = 6;
  const Cpatch& patch = *ppatch;

  // Empty six directions
  Vec4f xdir, ydir;
  ortho(ppatch->m_normal, xdir, ydir);
  
  // -1: not empty
  // pos: number of free m_pgrids
  //
  // Check if each direction satisfies both of the following two constraints.
  // a. No neighbor
  // b. At least minImageNumThreshold m_pgrids without any patches and few m_counts
  vector<float> fill;
  fill.resize(dnum);
  std::fill(fill.begin(), fill.end(), 0.0f);
  
  //----------------------------------------------------------------------
  // We look at the effective resolution of each image at the patch.
  // We only use images with good effective resolution to determine
  // empty blocks, because lwo-resolution images can easily satisfy
  // the first condition (neighbors), and no expansion will occur.
  // ----------------------------------------------------------------------
  // Minimum number of images required to obtain high res results, and
  // explor empty blocks.
  const float radius = computeRadius(patch);
  const float radiuslow = radius / 6.0f;//2.0f;
  const float radiushigh = radius * 2.5f;//2.0f;//1.5f;
  
  vector<Ppatch> neighbors;
  m_fm.m_pos.findNeighbors(patch, neighbors, 1, 4.0f);//3.0f);

  vector<Ppatch>::iterator bpatch = neighbors.begin();
  vector<Ppatch>::iterator epatch = neighbors.end();
  while (bpatch != epatch) {
    const Vec4f diff = (*bpatch)->m_coord - ppatch->m_coord;
    Vec2f f2(diff * xdir, diff * ydir);
    const float len = norm(f2);
    if (len < radiuslow || radiushigh < len) {
      ++bpatch;
      continue;
    }
    
    f2 /= len;
    //unitize(f2);
    
    float angle = atan2(f2[1], f2[0]);
    if (angle < 0.0)
      angle += 2 * M_PI;
    
    const float findex = angle / (2 * M_PI / dnum);
    const int lindex = (int)floor(findex);
    const int hindex = lindex + 1;
    
    fill[lindex % dnum] += hindex - findex;
    fill[hindex % dnum] += findex - lindex;
    ++bpatch;
  }

  canCoords.resize(dnum);
  for (int i = 0; i < dnum; ++i) {
    if (0.0f < fill[i])
    //if (0.5f < fill[i])
      continue;
    
    // If already failed, don't try, because we fail again.
    if (ppatch->m_dflag & (0x0001 << i))
      continue;    

    const float angle = 2 * M_PI * i / dnum;
    Vec4f canCoord = ppatch->m_coord +
      cos(angle) * radius * xdir + sin(angle) * radius * ydir;
    canCoords[i].push_back(canCoord);
  }
}

float Cexpand::computeRadius(const Patch::Cpatch& patch) {
  const int minnum = 2;
  vector<float> units;
  m_fm.m_optim.computeUnits(patch, units);
  vector<float> vftmp = units;
#ifdef DEBUG
  if ((int)units.size() < minnum) {
    cerr << "units size less than minnum: " << (int)units.size() << ' ' << minnum << endl;
    cout << (int)patch.m_images.size() << endl;
    exit (1);
  }
#endif
  nth_element(vftmp.begin(), vftmp.begin() + minnum - 1, vftmp.end());
  // Threshold is the second smallest value with some margin
  // ??? critical
  return (*(vftmp.begin() + minnum - 1)) * m_fm.m_csize;
}

int Cexpand::expandSub(const Ppatch& orgppatch, const int id,
                       const Vec4f& canCoord) {
  // Choose the closest one
  Cpatch patch;
  patch.m_coord = canCoord;
  patch.m_normal = orgppatch->m_normal;
  patch.m_flag = 1;

  m_fm.m_pos.setGridsImages(patch, orgppatch->m_images);
  if (patch.m_images.empty())
    return 1;

  //-----------------------------------------------------------------
  // Check bimages and mask. Then, initialize possible visible images
  if (m_fm.m_pss.getMask(patch.m_coord, m_fm.m_level) == 0 ||
        m_fm.insideBimages(patch.m_coord) == 0)
    return 1;

  // Check m_counts and maybe m_pgrids
  const int flag = checkCounts(patch);
  if (flag)
    return 1;

  // Check edge
  m_fm.m_optim.removeImagesEdge(patch);
  if (patch.m_images.empty())
    return 1;

  ++m_ecounts[id];
  //-----------------------------------------------------------------
  // Preprocess
  if (m_fm.m_optim.preProcess(patch, id, 0)) {
    ++m_fcounts0[id];
    return 1;
  }

  //-----------------------------------------------------------------
  m_fm.m_optim.refinePatch(patch, id, 100);

  //-----------------------------------------------------------------
  if (m_fm.m_optim.postProcess(patch, id, 0)) {
    ++m_fcounts1[id];
    return 1;
  }
  ++m_pcounts[id];

  //-----------------------------------------------------------------
  // Finally
  Ppatch ppatch(new Cpatch(patch));

  //patch.m_images = orgppatch->m_images;
  const int add = updateCounts(patch);

  m_fm.m_pos.addPatch(ppatch);

  if (add) {
    pthread_rwlock_wrlock(&m_fm.m_lock);      
    m_queue.push(ppatch);
    pthread_rwlock_unlock(&m_fm.m_lock);  
  }    

  return 0;
}

int Cexpand::checkCounts(Patch::Cpatch& patch) {
  int full = 0;  int empty = 0;

  vector<int>::iterator begin = patch.m_images.begin();
  vector<int>::iterator end = patch.m_images.end();
  vector<Vec2i>::iterator begin2 = patch.m_grids.begin();
  
  while (begin != end) {
    const int index = *begin;
    if (m_fm.m_tnum <= index) {
      ++begin;
      ++begin2;
      continue;
    }
    
    const int ix = (*begin2)[0];    const int iy = (*begin2)[1];
    if (ix < 0 || m_fm.m_pos.m_gwidths[index] <= ix ||
        iy < 0 || m_fm.m_pos.m_gheights[index] <= iy) {
      ++begin;      ++begin2;
      continue;
    }
    
    const int index2 = iy * m_fm.m_pos.m_gwidths[index] + ix;

    int flag = 0;
    pthread_rwlock_rdlock(&m_fm.m_imageLocks[index]);
    if (!m_fm.m_pos.m_pgrids[index][index2].empty())
      flag = 1;
    pthread_rwlock_unlock(&m_fm.m_imageLocks[index]);
    if (flag) {
      ++full;      ++begin;
      ++begin2;    continue;
    }
    
    //pthread_rwlock_wrlock(&m_fm.m_countLocks[index]);
    pthread_rwlock_rdlock(&m_fm.m_countLocks[index]);
    if (m_fm.m_countThreshold1 <= m_fm.m_pos.m_counts[index][index2])
      ++full;
    else
      ++empty;
    //++m_fm.m_pos.m_counts[index][index2];
    pthread_rwlock_unlock(&m_fm.m_countLocks[index]);
    ++begin;    ++begin2;
  }

  //First expansion is expensive and make the condition strict
  if (m_fm.m_depth <= 1) {
    if (empty < m_fm.m_minImageNumThreshold && full != 0)
      return 1;
    else
      return 0;
  }
  else {
    if (empty < m_fm.m_minImageNumThreshold - 1 && full != 0)
      return 1;
    else
      return 0;
  }
}

int Cexpand::updateCounts(const Cpatch& patch) {
  // Use m_images and m_vimages. Loosen when to set add = 1
  int full = 0;  int empty = 0;

  {
    vector<int>::const_iterator begin = patch.m_images.begin();
    vector<int>::const_iterator end = patch.m_images.end();
    vector<Vec2i>::const_iterator begin2 = patch.m_grids.begin();
    
    while (begin != end) {
      const int index = *begin;
      if (m_fm.m_tnum <= index) {
        ++begin;
        ++begin2;
        continue;
      }
      
      const int ix = (*begin2)[0];
      const int iy = (*begin2)[1];
      if (ix < 0 || m_fm.m_pos.m_gwidths[index] <= ix ||
          iy < 0 || m_fm.m_pos.m_gheights[index] <= iy) {
        ++begin;
        ++begin2;
        continue;
      }
      
      const int index2 = iy * m_fm.m_pos.m_gwidths[index] + ix;
      
      pthread_rwlock_wrlock(&m_fm.m_countLocks[index]);
      if (m_fm.m_countThreshold1 <= m_fm.m_pos.m_counts[index][index2])
        ++full;
      else
        ++empty;
      ++m_fm.m_pos.m_counts[index][index2];
      
      pthread_rwlock_unlock(&m_fm.m_countLocks[index]);
      ++begin;    ++begin2;
    }
  }

  {
    vector<int>::const_iterator begin = patch.m_vimages.begin();
    vector<int>::const_iterator end = patch.m_vimages.end();
    vector<Vec2i>::const_iterator begin2 = patch.m_vgrids.begin();
    
    while (begin != end) {
      const int index = *begin;
#ifdef DEBUG
      if (m_fm.m_tnum <= index) {
        cerr << "Impossible in updateCounts" << endl;
        exit (1);
      }
#endif
        
      const int ix = (*begin2)[0];
      const int iy = (*begin2)[1];
      if (ix < 0 || m_fm.m_pos.m_gwidths[index] <= ix ||
          iy < 0 || m_fm.m_pos.m_gheights[index] <= iy) {
        ++begin;
        ++begin2;
        continue;
      }
      
      const int index2 = iy * m_fm.m_pos.m_gwidths[index] + ix;
      
      pthread_rwlock_wrlock(&m_fm.m_countLocks[index]);
      if (m_fm.m_countThreshold1 <= m_fm.m_pos.m_counts[index][index2])
        ++full;
      else
        ++empty;
      ++m_fm.m_pos.m_counts[index][index2];        
      pthread_rwlock_unlock(&m_fm.m_countLocks[index]);
      ++begin;    ++begin2;
    }
  }
  
  if (empty != 0)
    return 1;
  else
    return 0;
}
