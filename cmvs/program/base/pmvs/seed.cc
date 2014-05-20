#include <time.h>
#include <numeric>
#include <ctime>
#include <sys/time.h>
#include "seed.h"
#include "findMatch.h"

using namespace Image;
using namespace PMVS3;
using namespace Patch;
using namespace std;

Cseed::Cseed(CfindMatch& findMatch) : m_fm(findMatch) {
}

void Cseed::init(const std::vector<std::vector<Cpoint> >& points) {
  m_ppoints.clear();
  m_ppoints.resize(m_fm.m_num);

  for (int index = 0; index < m_fm.m_num; ++index) {
    const int gheight = m_fm.m_pos.m_gheights[index];
    const int gwidth = m_fm.m_pos.m_gwidths[index];
    m_ppoints[index].resize(gwidth * gheight);
  }
  
  readPoints(points);
}

void Cseed::readPoints(const std::vector<std::vector<Cpoint> >& points) {
  for (int index = 0; index < m_fm.m_num; ++index) {
    for (int i = 0; i < (int)points[index].size(); ++i) {
      Ppoint ppoint(new Cpoint(points[index][i]));
      ppoint->m_itmp = index;
      const int ix = ((int)floor(ppoint->m_icoord[0] + 0.5f)) / m_fm.m_csize;
      const int iy = ((int)floor(ppoint->m_icoord[1] + 0.5f)) / m_fm.m_csize;
      const int index2 = iy * m_fm.m_pos.m_gwidths[index] + ix;
      m_ppoints[index][index2].push_back(ppoint);
    }
  }
}

void Cseed::run(void) {
  m_fm.m_count = 0;
  m_fm.m_jobs.clear();
  m_scounts.resize(m_fm.m_CPU);
  m_fcounts0.resize(m_fm.m_CPU);
  m_fcounts1.resize(m_fm.m_CPU);
  m_pcounts.resize(m_fm.m_CPU);
  fill(m_scounts.begin(), m_scounts.end(), 0);
  fill(m_fcounts0.begin(), m_fcounts0.end(), 0);
  fill(m_fcounts1.begin(), m_fcounts1.end(), 0);
  fill(m_pcounts.begin(), m_pcounts.end(), 0);
  
  vector<int> vitmp;
  for (int i = 0; i < m_fm.m_tnum; ++i)
    vitmp.push_back(i);

  random_shuffle(vitmp.begin(), vitmp.end());
  m_fm.m_jobs.insert(m_fm.m_jobs.end(), vitmp.begin(), vitmp.end());

  cerr << "adding seeds " << endl;
  
  m_fm.m_pos.clearCounts();

  // If there already exists a patch, don't use
  for (int index = 0; index < (int)m_fm.m_tnum; ++index) {
    for (int j = 0; j < (int)m_fm.m_pos.m_pgrids[index].size(); ++j) {
      if (!m_fm.m_pos.m_pgrids[index][j].empty())
        m_fm.m_pos.m_counts[index][j] = m_fm.m_countThreshold2;
    }
  }

  struct timeval tv;
  gettimeofday(&tv, NULL); 
  time_t curtime = tv.tv_sec;
  pthread_t threads[m_fm.m_CPU];
  for (int i = 0; i < m_fm.m_CPU; ++i)
    pthread_create(&threads[i], NULL, initialMatchThreadTmp, (void*)this);
  for (int i = 0; i < m_fm.m_CPU; ++i)
    pthread_join(threads[i], NULL);
  //----------------------------------------------------------------------
  cerr << "done" << endl;
  
  cerr << "---- Initial: " << tv.tv_sec - curtime << " secs ----" << endl;

  const int trial = accumulate(m_scounts.begin(), m_scounts.end(), 0);
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

void Cseed::initialMatchThread(void) {
  pthread_rwlock_wrlock(&m_fm.m_lock);
  const int id = m_fm.m_count++;
  pthread_rwlock_unlock(&m_fm.m_lock);

  while (1) {
    int index = -1;
    pthread_rwlock_wrlock(&m_fm.m_lock);
    if (!m_fm.m_jobs.empty()) {
      index = m_fm.m_jobs.front();
      m_fm.m_jobs.pop_front();
    }
    pthread_rwlock_unlock(&m_fm.m_lock);
    if (index == -1)
      break;

    initialMatch(index, id);
  }
}

void* Cseed::initialMatchThreadTmp(void* arg) {
  ((Cseed*)arg)->initialMatchThread();
  return NULL;
}

void Cseed::clear(void) {
  vector<vector<vector<Ppoint> > >().swap(m_ppoints);
}

void Cseed::initialMatch(const int index, const int id) {
  vector<int> indexes;
  m_fm.m_optim.collectImages(index, indexes);

  if (m_fm.m_tau < (int)indexes.size())
    indexes.resize(m_fm.m_tau);
  
  if (indexes.empty())
    return;  

  int totalcount = 0;
  //======================================================================
  // for each feature point, starting from the optical center, keep on
  // matching until we find candidateThreshold patches
  const int gheight = m_fm.m_pos.m_gheights[index];
  const int gwidth = m_fm.m_pos.m_gwidths[index];

  int index2 = -1;
  for (int y = 0; y < gheight; ++y) {
    for (int x = 0; x < gwidth; ++x) {
      ++index2;
      if (!canAdd(index, x, y))
	continue;

      for (int p = 0; p < (int)m_ppoints[index][index2].size(); ++p) {
	// collect features that satisfies epipolar geometry
	// constraints and sort them according to the differences of
	// distances between two cameras.
	vector<Ppoint> vcp;
	collectCandidates(index, indexes,
                          *m_ppoints[index][index2][p], vcp);
        
	int count = 0;
	Cpatch bestpatch;
	//======================================================================
	for (int i = 0; i < (int)vcp.size(); ++i) {
	  Cpatch patch;
	  patch.m_coord = vcp[i]->m_coord;
	  patch.m_normal =
            m_fm.m_pss.m_photos[index].m_center - patch.m_coord;

	  unitize(patch.m_normal);
	  patch.m_normal[3] = 0.0;
	  patch.m_flag = 0;

          ++m_fm.m_pos.m_counts[index][index2];
          const int ix = ((int)floor(vcp[i]->m_icoord[0] + 0.5f)) / m_fm.m_csize;
          const int iy = ((int)floor(vcp[i]->m_icoord[1] + 0.5f)) / m_fm.m_csize;
          const int index3 = iy * m_fm.m_pos.m_gwidths[vcp[i]->m_itmp] + ix;
          if (vcp[i]->m_itmp < m_fm.m_tnum)
            ++m_fm.m_pos.m_counts[vcp[i]->m_itmp][index3];
          
	  const int flag = initialMatchSub(index, vcp[i]->m_itmp, id, patch);
	  if (flag == 0) {
	    ++count;
	    if (bestpatch.score(m_fm.m_nccThreshold) <
                patch.score(m_fm.m_nccThreshold))
	      bestpatch = patch;
	    if (m_fm.m_countThreshold0 <= count)
	      break;
	  }
      	}
	if (count != 0) {
	  Ppatch ppatch(new Cpatch(bestpatch));
	  m_fm.m_pos.addPatch(ppatch);
	  ++totalcount;
          break;
	}
      }
    }
  }
  cerr << '(' << index << ',' << totalcount << ')' << flush;
}

void Cseed::collectCells(const int index0, const int index1,
                         const Cpoint& p0, std::vector<Vec2i>& cells) {
  Vec3 point(p0.m_icoord[0], p0.m_icoord[1], p0.m_icoord[2]);
#ifdef DEBUG
  if (p0.m_icoord[2] != 1.0f) {
    cerr << "Impossible in collectCells" << endl;    exit (1);
  }
#endif
  
  Mat3 F;
  Image::setF(m_fm.m_pss.m_photos[index0], m_fm.m_pss.m_photos[index1],
              F, m_fm.m_level);
  const int gwidth = m_fm.m_pos.m_gwidths[index1];
  const int gheight = m_fm.m_pos.m_gheights[index1];
  
  Vec3 line = transpose(F) * point;
  if (line[0] == 0.0 && line[1] == 0.0) {
    cerr << "Point right on top of the epipole?"
         << index0 << ' ' << index1 << endl;
    return;
  }
  // vertical
  if (fabs(line[0]) > fabs(line[1])) {
    for (int y = 0; y < gheight; ++y) {
      const float fy = (y + 0.5) * m_fm.m_csize - 0.5f;
      float fx = (- line[1] * fy - line[2]) / line[0];
      fx = max((float)(INT_MIN + 3.0f), std::min((float)(INT_MAX - 3.0f), fx));
      
      const int ix = ((int)floor(fx + 0.5f)) / m_fm.m_csize;
      if (0 <= ix && ix < gwidth)
        cells.push_back(TVec2<int>(ix, y));
      if (0 <= ix - 1 && ix - 1 < gwidth)
        cells.push_back(TVec2<int>(ix - 1, y));
      if (0 <= ix + 1 && ix + 1 < gwidth)
        cells.push_back(TVec2<int>(ix + 1, y));
    }
  }
  else {
    for (int x = 0; x < gwidth; ++x) {
      const float fx = (x + 0.5) * m_fm.m_csize - 0.5f;
      float fy = (- line[0] * fx - line[2]) / line[1];
      fy = max((float)(INT_MIN + 3.0f), std::min((float)(INT_MAX - 3.0f), fy));
      
      const int iy = ((int)floor(fy + 0.5f)) / m_fm.m_csize;
      if (0 <= iy && iy < gheight)
        cells.push_back(TVec2<int>(x, iy));
      if (0 <= iy - 1 && iy - 1 < gheight)
        cells.push_back(TVec2<int>(x, iy - 1));
      if (0 <= iy + 1 && iy + 1 < gheight)
        cells.push_back(TVec2<int>(x, iy + 1));
    }
  }
}

// make sorted array of feature points in images, that satisfy the
// epipolar geometry coming from point in image
void Cseed::collectCandidates(const int index, const std::vector<int>& indexes,
                              const Cpoint& point, std::vector<Ppoint>& vcp) {
  const Vec3 p0(point.m_icoord[0], point.m_icoord[1], 1.0);
  for (int i = 0; i < (int)indexes.size(); ++i) {        
    const int indexid = indexes[i];
    
    vector<TVec2<int> > cells;
    collectCells(index, indexid, point, cells);
    Mat3 F;
    Image::setF(m_fm.m_pss.m_photos[index], m_fm.m_pss.m_photos[indexid],
                F, m_fm.m_level);
    
    for (int i = 0; i < (int)cells.size(); ++i) {
      const int x = cells[i][0];      const int y = cells[i][1];
      if (!canAdd(indexid, x, y))
	continue;
      const int index2 = y * m_fm.m_pos.m_gwidths[indexid] + x;

      vector<Ppoint>::iterator begin = m_ppoints[indexid][index2].begin();
      vector<Ppoint>::iterator end = m_ppoints[indexid][index2].end();
      while (begin != end) {
        Cpoint& rhs = **begin;
        // ? use type to reject candidates?
        if (point.m_type != rhs.m_type) {
          ++begin;
          continue;
        }
          
        const Vec3 p1(rhs.m_icoord[0], rhs.m_icoord[1], 1.0);
        if (m_fm.m_epThreshold <= Image::computeEPD(F, p0, p1)) {
          ++begin;          
          continue;
        }
        vcp.push_back(*begin);
        ++begin;
      }
    }
  }
  
  // set distances to m_response
  vector<Ppoint> vcptmp;
  for (int i = 0; i < (int)vcp.size(); ++i) {
    unproject(index, vcp[i]->m_itmp, point, *vcp[i], vcp[i]->m_coord);
    
    if (m_fm.m_pss.m_photos[index].m_projection[m_fm.m_level][2] *
        vcp[i]->m_coord <= 0.0)
      continue;

    if (m_fm.m_pss.getMask(vcp[i]->m_coord, m_fm.m_level) == 0 ||
        m_fm.insideBimages(vcp[i]->m_coord) == 0)
      continue;

    //??? from the closest
    vcp[i]->m_response =
      fabs(norm(vcp[i]->m_coord - m_fm.m_pss.m_photos[index].m_center) -
           norm(vcp[i]->m_coord - m_fm.m_pss.m_photos[vcp[i]->m_itmp].m_center));
    
    vcptmp.push_back(vcp[i]);
  }
  vcptmp.swap(vcp);
  sort(vcp.begin(), vcp.end());
}

int Cseed::canAdd(const int index, const int x, const int y) {
  if (!m_fm.m_pss.getMask(index, m_fm.m_csize * x, m_fm.m_csize * y, m_fm.m_level))
    return 0;

  const int index2 = y * m_fm.m_pos.m_gwidths[index] + x;

  if (m_fm.m_tnum <= index)
    return 1;
  
  // Check if m_pgrids already contains something
  if (!m_fm.m_pos.m_pgrids[index][index2].empty())
    return 0;

  //??? critical
  if (m_fm.m_countThreshold2 <= m_fm.m_pos.m_counts[index][index2])
    return 0;
  
  return 1;
}

void Cseed::unproject(const int index0, const int index1,
                      const Cpoint& p0, const Cpoint& p1,
                      Vec4f& coord) const{
  Mat4 A;
  A[0][0] =
    m_fm.m_pss.m_photos[index0].m_projection[m_fm.m_level][0][0] -
    p0.m_icoord[0] * m_fm.m_pss.m_photos[index0].m_projection[m_fm.m_level][2][0];
  A[0][1] =
    m_fm.m_pss.m_photos[index0].m_projection[m_fm.m_level][0][1] -
    p0.m_icoord[0] * m_fm.m_pss.m_photos[index0].m_projection[m_fm.m_level][2][1];
  A[0][2] =
    m_fm.m_pss.m_photos[index0].m_projection[m_fm.m_level][0][2] -
    p0.m_icoord[0] * m_fm.m_pss.m_photos[index0].m_projection[m_fm.m_level][2][2];
  A[1][0] =
    m_fm.m_pss.m_photos[index0].m_projection[m_fm.m_level][1][0] -
    p0.m_icoord[1] * m_fm.m_pss.m_photos[index0].m_projection[m_fm.m_level][2][0];
  A[1][1] =
    m_fm.m_pss.m_photos[index0].m_projection[m_fm.m_level][1][1] -
    p0.m_icoord[1] * m_fm.m_pss.m_photos[index0].m_projection[m_fm.m_level][2][1];
  A[1][2] =
    m_fm.m_pss.m_photos[index0].m_projection[m_fm.m_level][1][2] -
    p0.m_icoord[1] * m_fm.m_pss.m_photos[index0].m_projection[m_fm.m_level][2][2];
  A[2][0] =
    m_fm.m_pss.m_photos[index1].m_projection[m_fm.m_level][0][0] -
    p1.m_icoord[0] * m_fm.m_pss.m_photos[index1].m_projection[m_fm.m_level][2][0];
  A[2][1] =
    m_fm.m_pss.m_photos[index1].m_projection[m_fm.m_level][0][1] -
    p1.m_icoord[0] * m_fm.m_pss.m_photos[index1].m_projection[m_fm.m_level][2][1];
  A[2][2] =
    m_fm.m_pss.m_photos[index1].m_projection[m_fm.m_level][0][2] -
    p1.m_icoord[0] * m_fm.m_pss.m_photos[index1].m_projection[m_fm.m_level][2][2];
  A[3][0] =
    m_fm.m_pss.m_photos[index1].m_projection[m_fm.m_level][1][0] -
    p1.m_icoord[1] * m_fm.m_pss.m_photos[index1].m_projection[m_fm.m_level][2][0];
  A[3][1] =
    m_fm.m_pss.m_photos[index1].m_projection[m_fm.m_level][1][1] -
    p1.m_icoord[1] * m_fm.m_pss.m_photos[index1].m_projection[m_fm.m_level][2][1];
  A[3][2] =
    m_fm.m_pss.m_photos[index1].m_projection[m_fm.m_level][1][2] -
    p1.m_icoord[1] * m_fm.m_pss.m_photos[index1].m_projection[m_fm.m_level][2][2];

  Vec4 b;
  b[0] =
    p0.m_icoord[0] * m_fm.m_pss.m_photos[index0].m_projection[m_fm.m_level][2][3] -
    m_fm.m_pss.m_photos[index0].m_projection[m_fm.m_level][0][3];
  b[1] =
    p0.m_icoord[1] * m_fm.m_pss.m_photos[index0].m_projection[m_fm.m_level][2][3] -
    m_fm.m_pss.m_photos[index0].m_projection[m_fm.m_level][1][3];
  b[2] =
    p1.m_icoord[0] * m_fm.m_pss.m_photos[index1].m_projection[m_fm.m_level][2][3] -
    m_fm.m_pss.m_photos[index1].m_projection[m_fm.m_level][0][3];
  b[3] =
    p1.m_icoord[1] * m_fm.m_pss.m_photos[index1].m_projection[m_fm.m_level][2][3] -
    m_fm.m_pss.m_photos[index1].m_projection[m_fm.m_level][1][3];

  Mat4 AT = transpose(A);
  Mat4 ATA = AT * A;
  Vec4 ATb = AT * b;

  Mat3 ATA3;
  for (int y = 0; y < 3; ++y)
    for (int x = 0; x < 3; ++x)
      ATA3[y][x] = ATA[y][x];
  Vec3 ATb3;
  for (int y = 0; y < 3; ++y)
    ATb3[y] = ATb[y];
  
  Mat3 iATA3;
  invert(iATA3, ATA3);
  Vec3 ans = iATA3 * ATb3;
  for (int y = 0; y < 3; ++y)
    coord[y] = ans[y];
  coord[3] = 1.0f;
}		       

// starting with (index, indexs), set visible images by looking at correlation.
int Cseed::initialMatchSub(const int index0, const int index1,
                           const int id, Cpatch& patch) {
  //----------------------------------------------------------------------
  patch.m_images.clear();
  patch.m_images.push_back(index0);
  patch.m_images.push_back(index1);

  ++m_scounts[id];

  //----------------------------------------------------------------------
  // We know that patch.m_coord is inside bimages and inside mask
  if (m_fm.m_optim.preProcess(patch, id, 1)) {
    ++m_fcounts0[id];
    return 1;
  }
  
  //----------------------------------------------------------------------  
  m_fm.m_optim.refinePatch(patch, id, 100);

  //----------------------------------------------------------------------
  if (m_fm.m_optim.postProcess(patch, id, 1)) {
    ++m_fcounts1[id];
    return 1;
  }
  
  ++m_pcounts[id];
  //----------------------------------------------------------------------
  return 0;
}
