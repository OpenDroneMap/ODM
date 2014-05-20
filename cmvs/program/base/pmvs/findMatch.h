#ifndef PMVS3_FINDMATCH_H
#define PMVS3_FINDMATCH_H

#include <vector>
#include <list>
#include <string>
#include <iostream>
#include <fstream>
#include <queue>
#include "patch.h"
#include <boost/shared_ptr.hpp>
#include <pthread.h>

#include "../image/photoSetS.h"
#include "patchOrganizerS.h"
#include "seed.h"
#include "expand.h"
#include "filter.h"
#include "optim.h"
#include "option.h"

namespace PMVS3 {
  
class CfindMatch {
 public:
  CfindMatch(void);
  virtual ~CfindMatch();

  void init(const PMVS3::Soption& option);
  void run(void);
  void write(const std::string prefix);
  
  int insideBimages(const Vec4f& coord) const;

  int isNeighborRadius(const Patch::Cpatch& lhs, const Patch::Cpatch& rhs,
                       const float hunit, const float neighborThreshold,
                       const float radius) const;
  
  int isNeighbor(const Patch::Cpatch& lhs, const Patch::Cpatch& rhs,
                 const float hunit, const float neighborThreshold) const;
  int isNeighbor(const Patch::Cpatch& lhs, const Patch::Cpatch& rhs,
                 const float neighborThreshold) const;
  
  //----------------------------------------------------------------------
  // num of target images
  int m_tnum;
  // num of total images
  int m_num;
  // target images
  std::vector<int> m_timages;
  // other images where patches are not computed
  std::vector<int> m_oimages;
  // total images
  std::vector<int> m_images;
  
  // prefix
  std::string m_prefix;
  // level
  int m_level;
  // cellsize
  int m_csize;
  // nccThreshold
  float m_nccThreshold;  
  // windows size
  int m_wsize;
  // mininum image num threshold
  int m_minImageNumThreshold;
  // use edge detection or not
  float m_setEdge;
  // bounding images
  std::vector<int> m_bindexes;
  // visdata from SfM. m_num x m_num matrix
  std::vector<std::vector<int> > m_visdata;
  // an array of relavant images
  std::vector<std::vector<int> > m_visdata2;  
  // sequence Threshold
  int m_sequenceThreshold;
  // CPU
  int m_CPU;
  // Threshold on filterQuad
  float m_quadThreshold;
  
  // Maximum number of images used in the optimization
  int m_tau;

  // If patches are dense or not, that is, if we use check(patch) after patch optimization
  int m_depth;
  
  //----------------------------------------------------------------------
  // Thresholds
  //----------------------------------------------------------------------
  // For first feature matching. Images within this angle are used in
  // matching.
  float m_angleThreshold0;
  // tigher angle
  float m_angleThreshold1;

  // Number of success generation from each seed point
  int m_countThreshold0;
  // Number of counts, expansion can be tried
  int m_countThreshold1;

  // Number of trials for each cell in seed
  int m_countThreshold2;
  
  // Parameter for isNeighbor in findemptyblocks
  float m_neighborThreshold;
  // Parameter for isNeighbor in filterOutside
  float m_neighborThreshold1;
  // Parameter for filterNeighbor
  float m_neighborThreshold2;

  // ncc threshold before optim
  float m_nccThresholdBefore;
  // Maximum angle of images must be at least as large as this
  float m_maxAngleThreshold;

  // visibility consistency threshold
  float m_visibleThreshold;
  float m_visibleThresholdLoose;
    
  // Epipolar distance in seed generation
  float m_epThreshold;
  
  //----------------------------------------------------------------------
  // For threads related
  //----------------------------------------------------------------------
  // General lock
  pthread_rwlock_t m_lock;
  // For each image
  std::vector<pthread_rwlock_t> m_imageLocks;
  std::vector<pthread_rwlock_t> m_countLocks;
  // count
  int m_count;
  // jobs
  std::list<int> m_jobs;
  // job unit
  int m_junit;

  //----------------------------------------------------------------------
  // Core members
  //----------------------------------------------------------------------
  // Images
  Image::CphotoSetS m_pss;
  // Patch organizer
  CpatchOrganizerS m_pos;
  
  // Seed generator
  Cseed m_seed;
  // Patch expansion
  Cexpand m_expand;
 public:
  // Patch filter
  Cfilter m_filter;
  // Patch optimizer
  Coptim m_optim;

  int m_debug;
 protected:
  void init(void);
  void initTargets(void);
  void updateThreshold(void);
  void initImages(void);
};
};

#endif // PMVS3_FINDMATCH_H
