#ifndef CMVS_BUNDLE_H
#define CMVS_BUNDLE_H

#include <vector>
#include <string>
#include <list>
#include <map>
#include <set>
#include <queue>
#include <ctime>
#include <sys/time.h>
#include <boost/graph/properties.hpp>
#include <boost/pending/disjoint_sets.hpp>

#include "../stann/sfcnn.hpp"
#include "../numeric/mat3.h"
#include "../image/photoSetS.h"
#include <pthread.h>

namespace CMVS {

struct Sadd {
  Sadd(const int image, const float gain) : m_image(image), m_gain(gain) {
  };
  
  int m_image;
  float m_gain;
};

struct Ssfm2 {
  Ssfm2(void) : m_cluster(-1), m_score(-2.0f),
                m_scoreThreshold(-1.0f), m_satisfied(1) {
  }
  // which cluster it belongs to currently
  int m_cluster;

  // current score
  float m_score;
  
  // score threshold
  float m_scoreThreshold;

  // If SFM is satisfied or not.
  // 1: satisfied, 0: not satisfied
  //
  // In adding images,
  // 2: currently not satisfied, 1: satisfied,
  // 0: not satisfied from the begining and no hope
  char m_satisfied;
  
  // For SfM point that has not bee satisfied, compute several number
  // of images that can be added to gain more info
  std::vector<Sadd> m_adds;

  // best images
  std::vector<int> m_uimages;
  
};

class Cbundle {
 public:
  Cbundle(void);
  virtual ~Cbundle();
  
  void run(const std::string prefix, const int imageThreshold,
           const int tau, const float scoreRatioThreshold,
           const float coverageThreshold, const int pnumThreshold,
           const int CPU);
  // root dir
  std::string m_prefix;

  // # of cameras
  int m_cnum;
  // # of points
  int m_pnum;
  
  // Point params
  std::vector<Vec4f> m_coords;
  std::vector<std::vector<int> > m_visibles;

  std::vector<Vec3f> m_colors;
  
  // A set of point ids visible in each camera
  std::vector<std::vector<int> > m_vpoints;

  std::vector<int> m_pweights;

  //----------------------------------------------------------------------
  // Generated data
  //----------------------------------------------------------------------
  // A list of connected images, for each camera.
  std::vector<std::vector<int> > m_neighbors;

  // Width and height of depth map
  std::vector<int> m_widths;
  std::vector<int> m_heights;
  // scale
  std::vector<int> m_levels;

  //----------------------------------------------------------------------
  // Output
  //----------------------------------------------------------------------
  // clusters
  // m_timages need to be sorted for addImages.
  std::vector<std::vector<int> > m_timages;
  std::vector<std::vector<int> > m_oimages;
  
 protected:
  void prep(const std::string prefix, const int imageThreshold,
            const int tau, const float scoreRatioThreshold,
            const float coverageThreshold, const int pnumThreshold,
            const int CPU);

  void prep2(void);
  
  void readBundle(const std::string file);
  void setWidthsHeightsLevels(void);
  void setNeighbors(void);
  
  int totalNum(void) const;

  // set m_scoreThresholds
  void setScoreThresholds(void);
  
  void resetVisibles(void);
  
  // set new images without image while taking into account m_removed
  void setNewImages(const int pid, const int rimage,
                    std::vector<int>& newimages);

  void sRemoveImages(void);
  void checkImage(const int image);
  
  void setCluster(const int p);
  
  void setScoresClusters(void);
  
  // For unsatisfied sfm points, update cluster
  void setClusters(void);

  void slimNeighborsSetLinks(void);
  
  float computeLink(const int image0, const int image1);
  
  void addImagesP(void);
  int addImages(void);
  int addImagesSub(const std::vector<std::map<int, float> >& cands);
  
  // angle score
  static float angleScore(const Vec4f& ray0, const Vec4f& ray1);
  
  void mergeSfM(void);
  void mergeSfMP(void);
  void mergeSfMPThread(void);
  static void* mergeSfMPThreadTmp(void* arg); 
    
  std::vector<char> m_merged;
  
  void findPNeighbors(sfcnn<const float*, 3, float>& tree,
                      const int pid, std::vector<int>& pneighbors);
  
  void resetPoints(void);
  
  static void mymerge(const std::vector<int>& lhs,
                      const std::vector<int>& rhs,
                      std::vector<int>& output);

  static int my_isIntersect(const std::vector<int>& lhs,
                            const std::vector<int>& rhs);
  
  // Cluster images
  void setTimages(void);
  void divideImages(const std::vector<int>& lhs,
                    std::vector<std::vector<int> >& rhs);

  float computeScore2(const Vec4f& coord,
                      const std::vector<int>& images) const;
  float computeScore2(const Vec4f& coord,
                      const std::vector<int>& images,
                      std::vector<int>& uimages) const;
  // Enforce the specified image to be inside
  float computeScore2(const Vec4f& coord,
                      const std::vector<int>& images,
                      const int index) const;

  void writeCameraCenters(void);
  void writeVis(void);
  void writeGroups(void);
  //-----------------------------------------------------------------
  //-----------------------------------------------------------------
  // Link info
  std::vector<std::vector<float> > m_links;
  
  // Removed or not for images
  std::vector<int> m_removed;
  // Number of SFM points above threshold for each image. We can
  // remove images until m_allows is non-negative for all the images.
  // Used in removing images
  std::vector<int> m_allows;
  // Used in adding images
  std::vector<int> m_lacks;

  // For an image, how sfm point (m_vpoints) changes if the image is
  // removed.
  //  0: unsatisfy
  //  1: satisfy->satisfy
  //  2: satisfy->unsatisfy
  std::vector<char> m_statsT;
  // image under consideration
  int m_imageT;
  // The value of lacks
  int m_lacksT;
  
  // sfm information used in addimages
  std::vector<Ssfm2> m_sfms2;

  // Number of images used in computeScore2
  int m_tau;
  // union find operations to be executed
  std::vector<std::vector<std::vector<int> > > m_ufsT;
  // Smallest scale
  std::vector<float> m_minScales;  

  // add nums
  std::vector<int> m_addnums;
  
  // scaling factor for depth
  float m_dscale;
  // scaling for kdtree version
  float m_dscale2;

  //----------------------------------------------------------------------
  Image::CphotoSetS m_pss;

  // depth level
  int m_dlevel;
  // maxLevel in m_pss.
  int m_maxLevel;
  
  int m_imageThreshold;
  // Num of points for images to be connected
  int m_pnumThreshold;

  // link threshold for neighbor
  float m_linkThreshold;

  // Score ratio threshold. Optimal score using all the visible images
  // times this threshold is the mimimum possible score to be
  // satisfied.
  float m_scoreRatioThreshold;
  // How much SFM must be satisfied in each image.
  float m_coverageThreshold;

  // union find for sfm points
  boost::disjoint_sets_with_storage<>* m_puf;

  sfcnn<const float*, 3, float>* m_ptree;
  
  //----------------------------------------------------------------------
  // Threads
  int m_CPU;
  pthread_rwlock_t m_lock;
  std::list<int> m_jobs;
  int m_junit;
  int m_thread;
  int m_count;
  
  int m_debug;


  void startTimer(void);
  time_t curTimer(void);
  
  struct timeval m_tv;
  time_t m_curtime;
};
};

#endif // CMVS_BUNDLE_H
