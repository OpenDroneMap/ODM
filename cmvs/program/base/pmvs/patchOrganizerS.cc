#include <string>
#include "patchOrganizerS.h"
#include "findMatch.h"

using namespace PMVS3;
using namespace Patch;
using namespace std;

Ppatch CpatchOrganizerS::m_MAXDEPTH(new Cpatch());
Ppatch CpatchOrganizerS::m_BACKGROUND(new Cpatch());

CpatchOrganizerS::CpatchOrganizerS(CfindMatch& findMatch) : m_fm(findMatch) {
}

// change the contents of m_images from images to indexes
void CpatchOrganizerS::image2index(Cpatch& patch) {
  // first image has to be target image
  vector<int> newimages;
  for (int i = 0; i < (int)patch.m_images.size(); ++i) {
    const int index = m_fm.m_pss.image2index(patch.m_images[i]);
    if (index != -1)
      newimages.push_back(index);
  }
  
  patch.m_images.swap(newimages);  

  // make sure that the reference image is the tagetting image
  int exist = -1;
  for (int j = 0; j < (int)patch.m_images.size(); ++j) {
    if (patch.m_images[j] < m_fm.m_tnum) {
      exist = j;
      break;
    }
  }
  if (exist == -1)
    patch.m_images.clear();
  else if (exist != 0)
    swap(patch.m_images[0], patch.m_images[exist]);
}

// change the contents of m_images from indexes to images
void CpatchOrganizerS::index2image(Cpatch& patch) {
  for (int i = 0; i < (int)patch.m_images.size(); ++i)
    patch.m_images[i] = m_fm.m_pss.m_images[patch.m_images[i]];
  for (int i = 0; i < (int)patch.m_vimages.size(); ++i)
    patch.m_vimages[i] = m_fm.m_pss.m_images[patch.m_vimages[i]];
}

void CpatchOrganizerS::init(void) {
  m_pgrids.clear();   m_pgrids.resize(m_fm.m_tnum);
  m_vpgrids.clear();  m_vpgrids.resize(m_fm.m_tnum);
  m_dpgrids.clear();  m_dpgrids.resize(m_fm.m_tnum);
  m_counts.clear();   m_counts.resize(m_fm.m_tnum);
  
  m_gwidths.clear();  m_gwidths.resize(m_fm.m_num);
  m_gheights.clear(); m_gheights.resize(m_fm.m_num);
  for (int index = 0; index < m_fm.m_num; ++index) {
    const int gwidth = (m_fm.m_pss.getWidth(index, m_fm.m_level)
                        + m_fm.m_csize - 1) / m_fm.m_csize;
    const int gheight = (m_fm.m_pss.getHeight(index, m_fm.m_level)
                         + m_fm.m_csize - 1) / m_fm.m_csize;
    m_gwidths[index] = gwidth;
    m_gheights[index] = gheight;

    if (index < m_fm.m_tnum) {
      m_pgrids[index].resize(gwidth * gheight);
      m_vpgrids[index].resize(gwidth * gheight);
      m_dpgrids[index].resize(gwidth * gheight);
      m_counts[index].resize(gwidth * gheight);
      fill(m_dpgrids[index].begin(), m_dpgrids[index].end(), m_MAXDEPTH);
    }
  }
}

//----------------------------------------------------------------------
void CpatchOrganizerS::writePatches(void) {
  for (int index = 0; index < m_fm.m_tnum; ++index) {
    const int image = m_fm.m_timages[index];
    vector<Ppatch> ppatches;
    collectNonFixPatches(index, ppatches);

    if (ppatches.empty())
      continue;

    char buffer[1024];
    sprintf(buffer, "%smodels/%08d.patc%d", m_fm.m_prefix.c_str(), image, m_fm.m_level);
    
    ofstream ofstr;
    ofstr.open(buffer);
    ofstr << "PATCHES" << endl
          << (int)ppatches.size() << endl;

    for (int p = 0; p < (int)ppatches.size(); ++p) {
      Cpatch patch = *ppatches[p];
      index2image(patch);
      ofstr << patch << endl;
    }
    ofstr.close();
    
    sprintf(buffer, "%smodels/%08d-%d.ply", m_fm.m_prefix.c_str(), image, m_fm.m_level);
    writePLY(ppatches, buffer);
  }

  collectPatches(1);
  char buffer[1024];
  sprintf(buffer, "%smodels/m-%08d-%d.ply", m_fm.m_prefix.c_str(),
          m_fm.m_pss.m_images[0], m_fm.m_level);
  writePLY(m_ppatches, buffer);  
}

void CpatchOrganizerS::writePatches2(const std::string prefix) {
  collectPatches(1);
  {
    char buffer[1024];
    sprintf(buffer, "%s.ply", prefix.c_str());
    writePLY(m_ppatches, buffer);
  }

  {
    char buffer[1024];
    sprintf(buffer, "%s.patch", prefix.c_str());
    ofstream ofstr;
    ofstr.open(buffer);
    ofstr << "PATCHES" << endl
          << (int)m_ppatches.size() << endl;
    for (int p = 0; p < (int)m_ppatches.size(); ++p) {
      Cpatch patch = *m_ppatches[p];
      index2image(patch);
      ofstr << patch << endl;
    }
    ofstr.close();
  }

  {
    char buffer[1024];
    sprintf(buffer, "%s.pset", prefix.c_str());
    ofstream ofstr;
    ofstr.open(buffer);
    for (int p = 0; p < (int)m_ppatches.size(); ++p)
      ofstr << m_ppatches[p]->m_coord[0] << ' '
            << m_ppatches[p]->m_coord[1] << ' '
            << m_ppatches[p]->m_coord[2] << ' '
            << m_ppatches[p]->m_normal[0] << ' '
            << m_ppatches[p]->m_normal[1] << ' '
            << m_ppatches[p]->m_normal[2] << endl;
    ofstr.close();
  }
}

void CpatchOrganizerS::readPatches(void) {
  // Read-in existing reconstructed points. set m_fix to one for non-targetting images
  for (int i = 0; i < m_fm.m_tnum; ++i) {
    const int image = m_fm.m_images[i];
    char buffer[1024];
    sprintf(buffer, "%smodels/%08d.patc%d",
            m_fm.m_prefix.c_str(), image, m_fm.m_level);
    ifstream ifstr;
    ifstr.open(buffer);
    if (!ifstr.is_open())
      continue;

    string header;    int pnum;
    ifstr >> header >> pnum;
    cerr << image << ' ' << pnum << " patches" << endl;
    for (int p = 0; p < pnum; ++p) {
      Ppatch ppatch(new Cpatch());
      ifstr >> *ppatch;
      ppatch->m_fix = 0;
      ppatch->m_vimages.clear();

      image2index(*ppatch);
      if (ppatch->m_images.empty())
        continue;
      
      // m_vimages must be targetting images
#ifdef DEBUG
      for (int j = 0; j < (int)ppatch->m_vimages.size(); ++j)
        if (m_fm.m_tnum <= ppatch->m_vimages[j]) {
          cerr << "Impossible in readPatches. m_vimages must be targetting images" << endl
               << "for patches stored in targetting images, if visdata2 have been consistent" << endl;
          exit (1);
        }
#endif
      setGrids(*ppatch);
      addPatch(ppatch);
    }
    
    ifstr.close();
  }

  // For patches in non-targetting images
  for (int i = m_fm.m_tnum; i < m_fm.m_num; ++i) {
    const int image = m_fm.m_images[i];
    char buffer[1024];
    sprintf(buffer, "%smodels/%08d.patc%d", m_fm.m_prefix.c_str(), image, m_fm.m_level);
    ifstream ifstr;
    ifstr.open(buffer);
    if (!ifstr.is_open())
      continue;
    
    string header;    int pnum;
    ifstr >> header >> pnum;
    cerr << image << ' ' << pnum << " patches" << endl;
    
    for (int p = 0; p < pnum; ++p) {
      Ppatch ppatch(new Cpatch());
      ifstr >> *ppatch;      
      ppatch->m_fix = 1;
      ppatch->m_vimages.clear();
      
      image2index(*ppatch);
      if (ppatch->m_images.empty())
        continue;
      
      setGrids(*ppatch);
      addPatch(ppatch);
    }
    
    ifstr.close();
  }
}

void CpatchOrganizerS::collectPatches(const int target) {
  m_ppatches.clear();

  for (int index = 0; index < m_fm.m_tnum; ++index) {
    for (int i = 0; i < (int)m_pgrids[index].size(); ++i) {
      vector<Ppatch>::iterator begin = m_pgrids[index][i].begin();
      while (begin != m_pgrids[index][i].end()) {
        (*begin)->m_id = -1;
        begin++;
      }
    }
  }
  
  int count = 0;
  for (int index = 0; index < m_fm.m_tnum; ++index) {
    for (int i = 0; i < (int)m_pgrids[index].size(); ++i) {
      vector<Ppatch>::iterator begin = m_pgrids[index][i].begin();
      while (begin != m_pgrids[index][i].end()) {
        if ((*begin)->m_id == -1) {
          (*begin)->m_id = count++;

          if (target == 0 || (*begin)->m_fix == 0)
            m_ppatches.push_back(*begin);
        }
        ++begin;
      }
    }
  }
}

void CpatchOrganizerS::collectPatches(std::priority_queue<Patch::Ppatch,
                                      std::vector<Patch::Ppatch>,
                                      P_compare>& pqpatches) {
  for (int index = 0; index < m_fm.m_tnum; ++index) {
    for (int i = 0; i < (int)m_pgrids[index].size(); ++i) {
      vector<Ppatch>::iterator begin = m_pgrids[index][i].begin();
      while (begin != m_pgrids[index][i].end()) {
        if ((*begin)->m_flag == 0) {
          (*begin)->m_flag = 1;
          pqpatches.push(*begin);
        }
        ++begin;
      }
    }
  }
}

void CpatchOrganizerS::collectPatches(const int index,
                                     std::priority_queue<Patch::Ppatch, std::vector<Patch::Ppatch>,
                                      P_compare>& pqpatches) {
  pthread_rwlock_wrlock(&m_fm.m_imageLocks[index]);
  for (int i = 0; i < (int)m_pgrids[index].size(); ++i) {
    vector<Ppatch>::iterator begin = m_pgrids[index][i].begin();
    vector<Ppatch>::iterator end = m_pgrids[index][i].end();
    
    while (begin != end) {
      if ((*begin)->m_images[0] == index && (*begin)->m_flag == 0) {
        (*begin)->m_flag = 1;
        pqpatches.push(*begin);
      }
      ++begin;
    }
  }
  pthread_rwlock_unlock(&m_fm.m_imageLocks[index]);
}

// Should be used only for writing
void CpatchOrganizerS::collectNonFixPatches(const int index,
                                      std::vector<Patch::Ppatch>& ppatches) {
  pthread_rwlock_wrlock(&m_fm.m_imageLocks[index]);
  for (int i = 0; i < (int)m_pgrids[index].size(); ++i) {
    vector<Ppatch>::iterator begin = m_pgrids[index][i].begin();
    vector<Ppatch>::iterator end = m_pgrids[index][i].end();
    
    while (begin != end) {
      if ((*begin)->m_images[0] == index && (*begin)->m_fix == 0) {
        ppatches.push_back(*begin);
      }
      ++begin;
    }
  }
  pthread_rwlock_unlock(&m_fm.m_imageLocks[index]);
}

void CpatchOrganizerS::clearFlags(void) {
  vector<Ppatch>::iterator bppatch = m_ppatches.begin();
  vector<Ppatch>::iterator eppatch = m_ppatches.end();

  while (bppatch != eppatch) {
    (*bppatch)->m_flag = 0;
    ++bppatch;
  }
}

void CpatchOrganizerS::clearCounts(void) {
  for (int index = 0; index < m_fm.m_tnum; ++index) {
    vector<unsigned char>::iterator begin = m_counts[index].begin();
    vector<unsigned char>::iterator end = m_counts[index].end();
    while (begin != end) {
      *begin = (unsigned char)0;
      ++begin;
    }
  }
}

void CpatchOrganizerS::addPatch(Patch::Ppatch& ppatch) {
  // First handle m_vimages
  vector<int>::iterator bimage = ppatch->m_images.begin();
  vector<int>::iterator eimage = ppatch->m_images.end();
  vector<Vec2i>::iterator bgrid = ppatch->m_grids.begin();
  while (bimage != eimage) {
    const int index = *bimage;
    if (m_fm.m_tnum <= index) {
      ++bimage;      ++bgrid;
      continue;
    }
    
    const int index2 = (*bgrid)[1] * m_gwidths[index] + (*bgrid)[0];
    pthread_rwlock_wrlock(&m_fm.m_imageLocks[index]);
    m_pgrids[index][index2].push_back(ppatch);
    pthread_rwlock_unlock(&m_fm.m_imageLocks[index]);
    ++bimage;
    ++bgrid;
  }

  // If depth, set vimages
  if (m_fm.m_depth == 0)
    return;
      
  bimage = ppatch->m_vimages.begin();
  eimage = ppatch->m_vimages.end();
  bgrid = ppatch->m_vgrids.begin();
  
  while (bimage != eimage) {
    const int index = *bimage;
    const int index2 = (*bgrid)[1] * m_gwidths[index] + (*bgrid)[0];
    pthread_rwlock_wrlock(&m_fm.m_imageLocks[index]);
    m_vpgrids[index][index2].push_back(ppatch);
    pthread_rwlock_unlock(&m_fm.m_imageLocks[index]);
    ++bimage;
    ++bgrid;
  }

  updateDepthMaps(ppatch);
}

void CpatchOrganizerS::updateDepthMaps(Ppatch& ppatch) {
  for (int image = 0; image < m_fm.m_tnum; ++image) {
    const Vec3f icoord = m_fm.m_pss.project(image, ppatch->m_coord, m_fm.m_level);

    const float fx = icoord[0] / m_fm.m_csize;
    const int xs[2] = {(int)floor(fx), (int)ceil(fx)};
    const float fy = icoord[1] / m_fm.m_csize;
    const int ys[2] = {(int)floor(fy), (int)ceil(fy)};
    
    const float depth = m_fm.m_pss.m_photos[image].m_oaxis * ppatch->m_coord;

    pthread_rwlock_wrlock(&m_fm.m_imageLocks[image]);
    for (int j = 0; j < 2; ++j) {
      for (int i = 0; i < 2; ++i) {
	if (xs[i] < 0 || m_gwidths[image] <= xs[i] ||
            ys[j] < 0 || m_gheights[image] <= ys[j])
	  continue;

        const int index = ys[j] * m_gwidths[image] + xs[i];
	if (m_dpgrids[image][index] == m_MAXDEPTH)
	  m_dpgrids[image][index] = ppatch;
	else {
	  const float dtmp = m_fm.m_pss.m_photos[image].m_oaxis *
	    m_dpgrids[image][index]->m_coord;
	  
	  if (depth < dtmp)
	    m_dpgrids[image][index] = ppatch;
	}
      }
    }
    pthread_rwlock_unlock(&m_fm.m_imageLocks[image]);
  }
}

void CpatchOrganizerS::setGridsImages(Patch::Cpatch& patch,
                                     const std::vector<int>& images) const {
  patch.m_images.clear();
  patch.m_grids.clear();
  vector<int>::const_iterator bimage = images.begin();
  vector<int>::const_iterator eimage = images.end();
  while (bimage != eimage) {
    const Vec3f icoord = m_fm.m_pss.project(*bimage, patch.m_coord, m_fm.m_level);
    const int ix = ((int)floor(icoord[0] + 0.5f)) / m_fm.m_csize;
    const int iy = ((int)floor(icoord[1] + 0.5f)) / m_fm.m_csize;
    if (0 <= ix && ix < m_gwidths[*bimage] &&
        0 <= iy && iy < m_gheights[*bimage]) {
      patch.m_images.push_back(*bimage);
      patch.m_grids.push_back(Vec2i(ix, iy));
    }
    ++bimage;
  }
}

void CpatchOrganizerS::setGrids(Ppatch& ppatch) const{
  setGrids(*ppatch);
}

void CpatchOrganizerS::setGrids(Cpatch& patch) const{
  patch.m_grids.clear();
  for (int i = 0; i < (int)patch.m_images.size(); ++i) {
    const int image = patch.m_images[i];
    Vec3f icoord = m_fm.m_pss.project(image, patch.m_coord, m_fm.m_level);
    const int ix = ((int)floor(icoord[0] + 0.5f)) / m_fm.m_csize;
    const int iy = ((int)floor(icoord[1] + 0.5f)) / m_fm.m_csize;
    patch.m_grids.push_back(TVec2<int>(ix, iy));
  }
}

void CpatchOrganizerS::setVImagesVGrids(Ppatch& ppatch) {
  setVImagesVGrids(*ppatch);
}

void CpatchOrganizerS::setVImagesVGrids(Cpatch& patch) {
  vector<int> used;
  used.resize(m_fm.m_tnum);
  fill(used.begin(), used.end(), 0);
  
  vector<int>::iterator bimage = patch.m_images.begin();
  vector<int>::iterator eimage = patch.m_images.end();
  while (bimage != eimage) {
    if ((*bimage) < m_fm.m_tnum)
      used[*(bimage)] = 1;
    ++bimage;
  }
  
  bimage = patch.m_vimages.begin();
  eimage = patch.m_vimages.end();
  while (bimage != eimage)
    used[*(bimage++)] = 1;

  for (int image = 0; image < m_fm.m_tnum; ++image) {
    if (used[image])
      continue;

    int ix, iy;
    if (isVisible0(patch, image, ix, iy,
                   m_fm.m_neighborThreshold, 1) == 0) {
      continue;
    }
    
    if (m_fm.m_pss.getEdge(patch.m_coord, image, m_fm.m_level) == 0)
      continue;
    
    patch.m_vimages.push_back(image);
    patch.m_vgrids.push_back(TVec2<int>(ix, iy));
  }
}

void CpatchOrganizerS::removePatch(const Ppatch& ppatch) {
  for (int i = 0; i < (int)ppatch->m_images.size(); ++i) {
    const int image = ppatch->m_images[i];
    if (m_fm.m_tnum <= image)
      continue;
    
    const int& ix = ppatch->m_grids[i][0];
    const int& iy = ppatch->m_grids[i][1];
    const int index = iy * m_gwidths[image] + ix;
    m_pgrids[image][index].erase(remove(m_pgrids[image][index].begin(),
                                        m_pgrids[image][index].end(),
                                        ppatch),
                                 m_pgrids[image][index].end());
  }

  for (int i = 0; i < (int)ppatch->m_vimages.size(); ++i) {
    const int image = ppatch->m_vimages[i];
#ifdef DEBUG
    if (m_fm.m_tnum <= image) {
      cerr << "Impossible in removePatch. m_vimages must be targetting images" << endl;
      exit (1);
    }
#endif

    const int& ix = ppatch->m_vgrids[i][0];
    const int& iy = ppatch->m_vgrids[i][1];
    const int index = iy * m_gwidths[image] + ix;
    m_vpgrids[image][index].erase(remove(m_vpgrids[image][index].begin(),
                                         m_vpgrids[image][index].end(),
                                         ppatch),
                                  m_vpgrids[image][index].end());
  }
}

int CpatchOrganizerS::isVisible0(const Cpatch& patch, const int image,
                                int& ix, int& iy,
                                const float strict, const int lock) {
  const Vec3f icoord =
    m_fm.m_pss.project(image, patch.m_coord, m_fm.m_level);
  ix = ((int)floor(icoord[0] + 0.5f)) / m_fm.m_csize;
  iy = ((int)floor(icoord[1] + 0.5f)) / m_fm.m_csize;

  return isVisible(patch, image, ix, iy, strict, lock);
}  

int CpatchOrganizerS::isVisible(const Cpatch& patch, const int image,
                               const int& ix, const int& iy,
                               const float strict, const int lock) {
  const int& gwidth = m_gwidths[image];
  const int& gheight = m_gheights[image];
  
  if (ix < 0 || gwidth <= ix || iy < 0 || gheight <= iy)
    return 0;

  if (m_fm.m_depth == 0)
    return 1;

  int ans = 0;
  Ppatch dppatch = m_MAXDEPTH;
  const int index = iy * gwidth + ix;
  
  if (lock)
    pthread_rwlock_rdlock(&m_fm.m_imageLocks[image]);

  if (m_dpgrids[image][index] == m_MAXDEPTH)
    ans = 1;
  else
    dppatch = m_dpgrids[image][index];
  
  if (lock)
    pthread_rwlock_unlock(&m_fm.m_imageLocks[image]);

  if (ans == 1)
    return 1;
  

  Vec4f ray = patch.m_coord - m_fm.m_pss.m_photos[image].m_center;
  unitize(ray);
  const float diff = ray * (patch.m_coord - dppatch->m_coord);
  const float factor = min(2.0, 2.0 + ray * patch.m_normal);
  
  if (diff < m_fm.m_optim.getUnit(image, patch.m_coord) * m_fm.m_csize * strict * factor)
    return 1;
  else
    return 0;
}  

void CpatchOrganizerS::findNeighbors(const Patch::Cpatch& patch,
                                     std::vector<Patch::Ppatch>& neighbors,
                                     const int lock, const float scale,
                                     const int margin,
                                     const int skipvis) {
  const float radius = 1.5 * margin * m_fm.m_expand.computeRadius(patch);
  
  vector<int>::const_iterator bimage = patch.m_images.begin();
  vector<int>::const_iterator eimage = patch.m_images.end();
  vector<Vec2i>::const_iterator bgrid = patch.m_grids.begin();

#ifdef DEBUG
  if (patch.m_images.empty()) {
    cerr << "Empty patches in findCloses" << endl;
    exit (1);
  }
#endif
  float unit = 0.0f;
  for (int i = 0; i < (int)patch.m_images.size(); ++i)
    unit += m_fm.m_optim.getUnit(patch.m_images[i], patch.m_coord);
  unit /= (int)patch.m_images.size();
  unit *= m_fm.m_csize;
  
  while (bimage != eimage) {
    if (m_fm.m_tnum <= *bimage) {
      ++bimage;
      ++bgrid;
      continue;
    }
    const int image = *bimage;
    const int& ix = (*bgrid)[0];
    const int& iy = (*bgrid)[1];
    if (lock)
      pthread_rwlock_rdlock(&m_fm.m_imageLocks[image]);
    for (int j = -margin; j <= margin; ++j) {
      const int ytmp = iy + j;
      if (ytmp < 0 || m_fm.m_pos.m_gheights[image] <= ytmp)
        continue;
      for (int i = -margin; i <= margin; ++i) {
        const int xtmp = ix + i;
        if (xtmp < 0 || m_fm.m_pos.m_gwidths[image] <= xtmp)
          continue;
        const int index = ytmp * m_fm.m_pos.m_gwidths[image] + xtmp;
        vector<Ppatch>::const_iterator bpatch =
          m_fm.m_pos.m_pgrids[image][index].begin();
        vector<Ppatch>::const_iterator epatch =
          m_fm.m_pos.m_pgrids[image][index].end();
        while (bpatch != epatch) {
          if (m_fm.isNeighborRadius(patch, **bpatch, unit,
                                          m_fm.m_neighborThreshold * scale,
                                          radius))
            neighbors.push_back(*bpatch);
          ++bpatch;
        }
        bpatch = m_fm.m_pos.m_vpgrids[image][index].begin();
        epatch = m_fm.m_pos.m_vpgrids[image][index].end();
        while (bpatch != epatch) {
          if (m_fm.isNeighborRadius(patch, **bpatch, unit,
                                    m_fm.m_neighborThreshold * scale,
                                    radius))
            neighbors.push_back(*bpatch);
          ++bpatch;
        }
      }
    }
    if (lock)
      pthread_rwlock_unlock(&m_fm.m_imageLocks[image]);

    ++bimage;
    ++bgrid;
  }

  if (skipvis == 0) {
    bimage = patch.m_vimages.begin();
    eimage = patch.m_vimages.end();
    bgrid = patch.m_vgrids.begin();
    
    while (bimage != eimage) {
      const int image = *bimage;
      const int& ix = (*bgrid)[0];
      const int& iy = (*bgrid)[1];
      if (lock)
        pthread_rwlock_rdlock(&m_fm.m_imageLocks[image]);
      for (int j = -margin; j <= margin; ++j) {
        const int ytmp = iy + j;
        if (ytmp < 0 || m_fm.m_pos.m_gheights[image] <= ytmp)
          continue;
        for (int i = -margin; i <= margin; ++i) {
          const int xtmp = ix + i;
          if (xtmp < 0 || m_fm.m_pos.m_gwidths[image] <= xtmp)
            continue;
          const int index = ytmp * m_fm.m_pos.m_gwidths[image] + xtmp;
          vector<Ppatch>::const_iterator bpatch =
            m_fm.m_pos.m_pgrids[image][index].begin();
          vector<Ppatch>::const_iterator epatch =
            m_fm.m_pos.m_pgrids[image][index].end();
          while (bpatch != epatch) {
            if (m_fm.isNeighborRadius(patch, **bpatch, unit,
                                      m_fm.m_neighborThreshold * scale,
                                      radius))
              neighbors.push_back(*bpatch);
            ++bpatch;
          }
          bpatch = m_fm.m_pos.m_vpgrids[image][index].begin();
          epatch = m_fm.m_pos.m_vpgrids[image][index].end();
          while (bpatch != epatch) {
            if (m_fm.isNeighborRadius(patch, **bpatch, unit,
                                      m_fm.m_neighborThreshold * scale,
                                      radius))
              neighbors.push_back(*bpatch);
            ++bpatch;
          }
        }
      }
      if (lock)
        pthread_rwlock_unlock(&m_fm.m_imageLocks[image]);
      
      ++bimage;
      ++bgrid;
    }
  }
  
  sort(neighbors.begin(), neighbors.end());
  neighbors.erase(unique(neighbors.begin(), neighbors.end()), neighbors.end());
}

float CpatchOrganizerS::computeUnit(const Patch::Cpatch& patch) const{
  float unit = 0.0f;
  for (int i = 0; i < (int)patch.m_images.size(); ++i)
    unit += m_fm.m_optim.getUnit(patch.m_images[i], patch.m_coord);
  unit /= (int)patch.m_images.size();
  unit *= m_fm.m_csize;
  return unit;
}  

void CpatchOrganizerS::setScales(Patch::Cpatch& patch) const {
  const float unit = m_fm.m_optim.getUnit(patch.m_images[0], patch.m_coord);
  const float unit2 = 2.0f * unit;
  Vec4f ray = patch.m_coord - m_fm.m_pss.m_photos[patch.m_images[0]].m_center;
  unitize(ray);

  const int inum = min(m_fm.m_tau, (int)patch.m_images.size());
  
  // First compute, how many pixel difference per unit along vertical
  //for (int i = 1; i < (int)patch.m_images.size(); ++i) {
  for (int i = 1; i < inum; ++i) {
    Vec3f diff = m_fm.m_pss.project(patch.m_images[i], patch.m_coord, m_fm.m_level) -
      m_fm.m_pss.project(patch.m_images[i], patch.m_coord - ray * unit2, m_fm.m_level);
    patch.m_dscale += norm(diff);
  }

  // set m_dscale to the vertical distance where average pixel move is half pixel
  //patch.m_dscale /= (int)patch.m_images.size() - 1;
  patch.m_dscale /= inum - 1;
  patch.m_dscale = unit2 / patch.m_dscale;
  
  patch.m_ascale = atan(patch.m_dscale / (unit * m_fm.m_wsize / 2.0f));
}

// write out results
void CpatchOrganizerS::writePLY(const std::vector<Ppatch>& patches,
                                const std::string filename) {
  ofstream ofstr;
  ofstr.open(filename.c_str());
  ofstr << "ply" << endl
       << "format ascii 1.0" << endl
       << "element vertex " << (int)patches.size() << endl
       << "property float x" << endl
       << "property float y" << endl
       << "property float z" << endl
       << "property float nx" << endl
       << "property float ny" << endl
       << "property float nz" << endl
       << "property uchar diffuse_red" << endl
       << "property uchar diffuse_green" << endl
       << "property uchar diffuse_blue" << endl
       << "end_header" << endl;

  vector<Ppatch>::const_iterator bpatch = patches.begin();
  vector<Ppatch>::const_iterator bend = patches.end();
  
  while (bpatch != bend) {
    // Get color
    Vec3i color;
    
    const int mode = 0;
    // 0: color from images
    // 1: fix
    // 2: angle
    if (mode == 0) {
      int denom = 0;
      Vec3f colorf;
      for (int i = 0; i < (int)(*bpatch)->m_images.size(); ++i) {
        const int image = (*bpatch)->m_images[i];
        colorf += m_fm.m_pss.getColor((*bpatch)->m_coord, image, m_fm.m_level);
        denom++;
      }
      colorf /= denom;
      color[0] = min(255,(int)floor(colorf[0] + 0.5f));
      color[1] = min(255,(int)floor(colorf[1] + 0.5f));
      color[2] = min(255,(int)floor(colorf[2] + 0.5f));
    }
    else if (mode == 1) {
      if ((*bpatch)->m_tmp == 1.0f) {
        color[0] = 255;
        color[1] = 0;
        color[2] = 0;
      }
      else {
        color[0] = 255;
        color[1] = 255;
        color[2] = 255;
      }
    }
    else if (mode == 2) {
      float angle = 0.0f;
      vector<int>::iterator bimage = (*bpatch)->m_images.begin();
      vector<int>::iterator eimage = (*bpatch)->m_images.end();

      while (bimage != eimage) {
        const int index = *bimage;
        Vec4f ray = m_fm.m_pss.m_photos[index].m_center - (*bpatch)->m_coord;
        ray[3] = 0.0f;
        unitize(ray);

        angle += acos(ray * (*bpatch)->m_normal);
        ++bimage;
      }
      
      angle = angle / (M_PI / 2.0f);
      float r, g, b;
      Image::Cimage::gray2rgb(angle, r, g, b);
      color[0] = (int)(r * 255.0f);
      color[1] = (int)(g * 255.0f);
      color[2] = (int)(b * 255.0f);
    }
    
    ofstr << (*bpatch)->m_coord[0] << ' '
          << (*bpatch)->m_coord[1] << ' '
          << (*bpatch)->m_coord[2] << ' '
          << (*bpatch)->m_normal[0] << ' '
          << (*bpatch)->m_normal[1] << ' '
          << (*bpatch)->m_normal[2] << ' '
          << color[0] << ' ' << color[1] << ' ' << color[2] << endl;
      ++bpatch;
  }
  ofstr.close();  
}

void CpatchOrganizerS::writePLY(const std::vector<Ppatch>& patches,
                                const std::string filename,
                                const std::vector<Vec3i>& colors) {
  ofstream ofstr;
  ofstr.open(filename.c_str());
  ofstr << "ply" << endl
       << "format ascii 1.0" << endl
       << "element vertex " << (int)patches.size() << endl
       << "property float x" << endl
       << "property float y" << endl
       << "property float z" << endl
       << "property float nx" << endl
       << "property float ny" << endl
       << "property float nz" << endl
       << "property uchar diffuse_red" << endl
       << "property uchar diffuse_green" << endl
       << "property uchar diffuse_blue" << endl
       << "end_header" << endl;

  vector<Ppatch>::const_iterator bpatch = patches.begin();
  vector<Ppatch>::const_iterator bend = patches.end();
  vector<Vec3i>::const_iterator colorb = colors.begin();
  
  while (bpatch != bend) {
    ofstr << (*bpatch)->m_coord[0] << ' '
          << (*bpatch)->m_coord[1] << ' '
          << (*bpatch)->m_coord[2] << ' '
          << (*bpatch)->m_normal[0] << ' '
          << (*bpatch)->m_normal[1] << ' '
          << (*bpatch)->m_normal[2] << ' '
          << *colorb << endl;
    ++bpatch;
    ++colorb;
  }
  ofstr.close();  
}
