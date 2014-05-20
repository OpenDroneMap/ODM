#ifndef PMVS3_OPTIM_H
#define PMVS3_OPTIM_H

#include <vector>
#include "patch.h"
#include <gsl/gsl_multimin.h>

namespace PMVS3 {
  
class CfindMatch;
 
class Coptim {
 public:
  Coptim(CfindMatch& findMatch);

  void init(void);

  //-----------------------------------------------------------------
  // Image manipulation
  //-----------------------------------------------------------------
  void collectImages(const int index, std::vector<int>& indexes) const;
  void addImages(Patch::Cpatch& patch) const;
  void removeImagesEdge(Patch::Cpatch& patch) const;

  float getUnit(const int index, const Vec4f& coord) const;

  void computeUnits(const Patch::Cpatch& patch,
                    std::vector<int>& indexes,
                    std::vector<float>& fineness,
                    std::vector<Vec4f>& rays) const;
  void computeUnits(const Patch::Cpatch& patch,
                    std::vector<float>& fineness) const;
  
  //-----------------------------------------------------------------
  // Optimization
  //-----------------------------------------------------------------
  
  int preProcess(Patch::Cpatch& patch, const int id, const int seed);
  void refinePatch(Patch::Cpatch& patch, const int id, const int time);
  
  void refinePatchBFGS(Patch::Cpatch& patch, const int id, const int time);
  void refinePatchBFGS(Patch::Cpatch& patch, const int id, const int time,
                       const int ncc);
  void refineDepthBFGS(Patch::Cpatch& patch, const int id, const int time,
                       const int ncc);
  
  int postProcess(Patch::Cpatch& patch, const int id, const int seed);

  void setRefImage(Patch::Cpatch& patch, const int id);
  
  int check(Patch::Cpatch& patch);

  std::vector<int> m_status;
  
 protected:
  void filterImagesByAngle(Patch::Cpatch& patch);
  
  void sortImages(Patch::Cpatch& patch) const;
  void constraintImages(Patch::Cpatch& patch, const float nccThreshold,
                        const int id);
  void setRefConstraintImages(Patch::Cpatch& patch, const float nccThreshold,
                              const int id);
  
  void setINCCs(const Patch::Cpatch& patch,
                std::vector<float> & nccs,
                const std::vector<int>& indexes,
                const int id, const int robust);
  
  void setINCCs(const Patch::Cpatch& patch,
                std::vector<std::vector<float> >& nccs,
                const std::vector<int>& indexes,
                const int id, const int robust);
  
  int grabTex(const Vec4f& coord, const Vec4f& pxaxis, const Vec4f& pyaxis,
              const Vec4f& pzaxis, const int index, const int size,
              std::vector<float>& tex) const;
  
  int grabSafe(const int index, const int size, const Vec3f& center,
               const Vec3f& dx, const Vec3f& dy, const int level) const;

  double computeSSD(const Vec4f& coord, const Vec4f& normal,
                    const std::vector<int>& indexes, const int id);

  double computeSSD(const Vec4f& coord, const Vec4f& normal,
                    const std::vector<int>& indexes, const Vec4f& pxaxis,
                    const Vec4f& pyaxis, const int id);
  /*
  double computeINCC(const Vec4f& coord, const Vec4f& normal,
                     const std::vector<int>& indexes, const int id,
                     const int robust);
  */
  double computeINCC(const Vec4f& coord, const Vec4f& normal,
                     const std::vector<int>& indexes, const Vec4f& pxaxis,
                     const Vec4f& pyaxis, const int id,
                     const int robust);

 public:
  static void normalize(std::vector<float>& tex);
  static void normalize(std::vector<std::vector<float> >& texs, const int size);
  
  float dot(const std::vector<float>& tex0, const std::vector<float>& tex1) const;
  float ssd(const std::vector<float>& tex0, const std::vector<float>& tex1) const;
 protected:
  static void lfunc(double* p, double* hx, int m, int n, void* adata);
  void func(int m, int n, double* x, double* fvec, int* iflag, void* arg);

  //BFGS
  static double my_f(const gsl_vector *v, void *params);
  static void my_df(const gsl_vector *v, void *params,
                    gsl_vector *df);
  static void my_fdf(const gsl_vector *x, void *params, 
                     double *f, gsl_vector *df);
  // for derivative computation
  static double my_f0(double x, void* params);
  static double my_f1(double x, void* params);
  static double my_f2(double x, void* params);
  //----------------------------------------------------------------------
  // For ssd
  static double my_f_ssd(const gsl_vector *v, void *params);
  static void my_df_ssd(const gsl_vector *v, void *params,
                        gsl_vector *df);
  static void my_fdf_ssd(const gsl_vector *x, void *params, 
                         double *f, gsl_vector *df);
  // for derivative computation
  static double my_f_ssd0(double x, void* params);
  static double my_f_ssd1(double x, void* params);
  static double my_f_ssd2(double x, void* params);
  //----------------------------------------------------------------------
  // For debugging depth
  static double my_f_depth(const gsl_vector *v, void *params);
  static void my_df_depth(const gsl_vector *v, void *params,
                    gsl_vector *df);
  static void my_fdf_depth(const gsl_vector *x, void *params, 
                     double *f, gsl_vector *df);
  // for derivative computation
  static double my_f0_depth(double x, void* params);
  
  void encode(const Vec4f& coord,
              double* const vect, const int id) const;
  void encode(const Vec4f& coord, const Vec4f& normal,
              double* const vect, const int id) const;
  void decode(Vec4f& coord, Vec4f& normal,
	      const double* const vect, const int id) const;
  void decode(Vec4f& coord, const double* const vect, const int id) const;
  

 public:
  void setWeightsT(const Patch::Cpatch& patch, const int id);
  
  double computeINCC(const Vec4f& coord, const Vec4f& normal,
                     const std::vector<int>& indexes, const int id,
                     const int robust);
  void getPAxes(const int index, const Vec4f& coord, const Vec4f& normal,
                Vec4f& pxaxis, Vec4f& pyaxis) const;
  static inline float robustincc(const float rhs) {
    return rhs / (1 + 3 * rhs);
  }

  static inline float unrobustincc(const float rhs) {
    return rhs / (1 - 3 * rhs);
  }
  
 protected:
  
  void setAxesScales(void);
  
  static Coptim* m_one;  
  CfindMatch& m_fm;
  
  //-----------------------------------------------------------------
  // Axes
  std::vector<Vec3f> m_xaxes;
  std::vector<Vec3f> m_yaxes;
  std::vector<Vec3f> m_zaxes;
  // Scales
  std::vector<float> m_ipscales;
  
  //-----------------------------------------------------------------
  // For threads
  std::vector<float> m_vect0T;  
  std::vector<Vec4f> m_centersT;
  std::vector<Vec4f> m_raysT;
  std::vector<std::vector<int> > m_indexesT;
  std::vector<float> m_dscalesT;
  std::vector<float> m_ascalesT;

  // stores current parameters for derivative computation
  std::vector<Vec3f> m_paramsT;
  
  // Grabbed texture
  std::vector<std::vector<std::vector<float> > > m_texsT;
  // weights for refineDepthOrientationWeighed
  std::vector<std::vector<float> > m_weightsT;
  // Working array for levmar
  std::vector<std::vector<double> > m_worksT;
  
};
};

#endif // PMVS3_OPTIM_H
