#ifndef IMAGE_IMAGE_H
#define IMAGE_IMAGE_H

#include <vector>
#include <string>
#include <cstdlib>
#include <cmath>
#include "../numeric/vec3.h"

namespace Image {

class Cimage {
 public:
  Cimage(void);
  virtual ~Cimage();

  virtual void init(const std::string name, const std::string mname,
		    const int maxLevel = 1);
  virtual void init(const std::string name, const std::string mname,
                    const std::string ename, const int maxLevel = 1);

  void sift(const Vec3f& center, const Vec3f& xaxis, const Vec3f& yaxis,
            std::vector<float>& descriptor) const;
  
  void sift(const Vec3f& center, const Vec3f& xaxis, const Vec3f& yaxis,
            const int level, std::vector<float>& descriptor) const;
            
  void setEdge(const float threshold);
  
  // access to image/masks
  inline Vec3f getColor(const float fx, const float fy, const int level) const;
  inline Vec3f getColor(const int ix, const int iy, const int level) const;

  inline void setColor(const int ix, const int iy, const int level,
                       const Vec3f& rgb);
  
  inline int getMask(const float fx, const float fy, const int level) const;
  inline int getMask(const int ix, const int iy, const int level) const;

  inline int getEdge(const float fx, const float fy, const int level) const;
  inline int getEdge(const int ix, const int iy, const int level) const;  
  
  inline int getWidth(const int level = 0) const;
  inline int getHeight(const int level = 0) const;
  //inline int getCWidth(const int beta, const int level = 0) const;
  //inline int getCHeight(const int beta, const int level = 0) const;

  inline const std::vector<unsigned char>& getImage(const int level) const;
  inline const std::vector<unsigned char>& getMask(const int level) const;
  inline const std::vector<unsigned char>& getEdge(const int level) const;
  inline std::vector<unsigned char>& getImage(const int level);
  inline std::vector<unsigned char>& getMask(const int level);
  inline std::vector<unsigned char>& getEdge(const int level);
  
  inline int isSafe(const Vec3f& icoord, const int level) const;
  // Check if a mask image exists
  inline int isMask(void) const;
  // Check if an edge image exists
  inline int isEdge(void) const;
  
  //allocate and free memories, this function is also called when you call
  //getColor/getMask when the memory is not allocated
  void alloc(const int fast = 0, const int filter = 0);
  // free memory
  void free(void);
  // free memory below the specified level
  void free(const int freeLevel);

  static int readPBMImage(const std::string file,
                          std::vector<unsigned char>& image,
                          int& width, int& height, const int fast);

  static int writePBMImage(const std::string file,
                           std::vector<unsigned char>& image,
                           int& width, int& height, const int fast);
  
  static int readPGMImage(const std::string file,
			  std::vector<unsigned char>& image,
			  int& width, int& height, const int fast);
  
  static int writePGMImage(const std::string file,
                           const std::vector<unsigned char>& image,
                           const int width, const int height);
  
  static int readPPMImage(const std::string file,
			  std::vector<unsigned char>& image,
			  int& width, int& height, const int fast);

  static int writePPMImage(const std::string file,
                           const std::vector<unsigned char>& image,
                           const int width, const int height);
  
  static int readJpegImage(const std::string file,
			   std::vector<unsigned char>& image,
			   int& width, int& height, const int fast);
  
  static void writeJpegImage(const std::string filename,
			     const std::vector<unsigned char>& buffer,
			     const int width, const int height,
			     const int flip = 0);

  static float hsdis(const float h0, const float s0,
		     const float h1, const float s1);
  
  static void rgb2hsv(const float r, const float g, const float b,
		      float& hr, float& sr, float& vr);
  static void rgb2hsv(const Vec3f& rgb, Vec3f& hsv);
  static void rgb2hsv(const Vec3f& rgb, float& hr, float& sr, float& vr);
  static void rgb2hsv(const float r, const float g, const float b, Vec3f& hsv);  
  
  static void rgb2hs(const float r, const float g, const float b,
		     float& h, float& s);
  static void rgb2hs(const Vec3f& rgb, float& h, float& s);
  static void rgb2hs(const Vec3f& rgb, Vec2f& hs);
  static void rgb2hs(const float r, const float g, const float b, Vec2f& hs);
  
  static void gray2rgb(const float gray, float& r, float& g, float& b);

  // Some low-level image processing
  // 2D convolution with twice 1D gaussian convolution.
  
  // Create a 1d gaussian filter based on sigma
  static void createFilter(const float sigma, std::vector<float>& filter);
  
  static void filterG(const std::vector<float>& filter,
                      std::vector<std::vector<float> >& data);
  static void filterG(const std::vector<float>& filter,
                      std::vector<std::vector<float> >& data,
                      std::vector<std::vector<float> >& buffer);

  static void filterG(const std::vector<float>& filter,
                      const int width, const int height,
                      std::vector<float>& data);

  static void filterG(const std::vector<float>& filter,
                      const int width, const int height,
                      std::vector<float>& data,
                      std::vector<float>& buffer);                      
  
  // non maximum surpression
  static void nms(std::vector<std::vector<float> >& data);
  static void nms(std::vector<std::vector<float> >& data,
                  std::vector<std::vector<float> >& buffer);
  
  /*
  // ThreshMode
  // Used to filter out outliers. Very general algorithm.
  static void setInOut(const std::vector<std::vector<float> >& data, std::vector<int>& inout,
		       const float sigma = 1.0f, const int specular = 0);

  // Used to filter out outliers. RGB version for specular highlights.
  static void setInOut(const std::vector<Vec3f>& rgbs, std::vector<int>& inout,
		       const float sigma = 1.0f, const int specular = 0);
  
  // Used to filter out outliers. HSV version for specular highlights.
  static void setInOutHSV(const std::vector<Vec3f>& hsvs, std::vector<int>& inout,
			  const float sigma = 1.0f, const int specular = 0);
  */
 protected:
  //----------------------------------------------------------------------
  // member functions
  //----------------------------------------------------------------------
  // complete the name of an image file
  static void completeName(const std::string& lhs, std::string& rhs,
                           const int color);
  
  // build image pyramids
  void buildImageMaskEdge(const int filter);
  void buildImage(const int filter);
  void buildMask(void);
  void buildEdge(void);
  
#ifdef FURUKAWA_IMAGE_GAMMA
  void decodeGamma(void);
#endif
  
  //----------------------------------------------------------------------
  // Variables updated at every alloc/free
  //----------------------------------------------------------------------  
  // 0: nothing allocated
  // 1: width/height allocated
  // 2: memory allocated
  int m_alloc;
  // a pyramid of images
  std::vector<std::vector<unsigned char> > m_images;
  // a pyramid of masks
  std::vector<std::vector<unsigned char> > m_masks;
  // a pyramid of images specifying regions with edges(texture)
  std::vector<std::vector<unsigned char> > m_edges;
  
  // width of an image in each level
  std::vector<int> m_widths;
  // height of an image in each level
  std::vector<int> m_heights;

#ifdef FURUKAWA_IMAGE_GAMMA
  // For gamma decoded images
  std::vector<std::vector<float> > m_dimages;
#endif
  
  //----------------------------------------------------------------------
  // Variables keep fixed
  //----------------------------------------------------------------------  
  // a name of an image
  std::string m_name;
  // a name of a mask image
  std::string m_mname;
  // a name of an image specifying regions with edges(texture)
  std::string m_ename;
  
  // number of levels
  int m_maxLevel;
};
  
inline int Cimage::isSafe(const Vec3f& icoord, const int level) const {
#ifdef FURUKAWA_IMAGE_BICUBIC
  if (icoord[0] < 1.0 || m_widths[level] - 3 < icoord[0] ||
      icoord[1] < 1.0 || m_heights[level] - 3 < icoord[1])
    return 0;
  else
    return 1;
#else
  if (icoord[0] < 0.0 || m_widths[level] - 2 < icoord[0] ||
      icoord[1] < 0.0 || m_heights[level] - 2 < icoord[1])
    return 0;
  else
    return 1;
#endif
};
 
// Check if a mask image exists
inline int Cimage::isMask(void) const {
  if (m_masks[0].empty())
    return 0;
  else
    return 1;
 };
 
// Check if an edge image exists
inline int Cimage::isEdge(void) const {
  if (m_edges[0].empty())
    return 0;
  else
    return 1;
 };
 
inline const std::vector<unsigned char>& Cimage::getImage(const int level) const{
  if (m_alloc != 2) {
    std::cerr << "First allocate" << std::endl;
    exit (1);
  }

#ifdef FURUKAWA_IMAGE_GAMMA
  std::cerr << "Cannot do getImage in the gamma correction mode." << std::endl;
  exit (1);
#endif
  
  return m_images[level];
};

inline const std::vector<unsigned char>& Cimage::getMask(const int level) const{
  if (m_alloc != 2) {
    std::cerr << "First allocate" << std::endl;
    exit (1);
  }  
  return m_masks[level];
};

inline const std::vector<unsigned char>& Cimage::getEdge(const int level) const{
  if (m_alloc != 2) {
    std::cerr << "First allocate" << std::endl;
    exit (1);
  }  
  return m_edges[level];
};

inline std::vector<unsigned char>& Cimage::getImage(const int level){
  if (m_alloc != 2) {
    std::cerr << "First allocate" << std::endl;
    exit (1);
  }

#ifdef FURUKAWA_IMAGE_GAMMA
  std::cerr << "Cannot do getImage in the gamma correction mode." << std::endl;
  exit (1);
#endif
  
  return m_images[level];
};

inline std::vector<unsigned char>& Cimage::getMask(const int level){
  if (m_alloc != 2) {
    std::cerr << "First allocate" << std::endl;
    exit (1);
  }  
  return m_masks[level];
};

inline std::vector<unsigned char>& Cimage::getEdge(const int level){
  if (m_alloc != 2) {
    std::cerr << "First allocate" << std::endl;
    exit (1);
  }  
  return m_edges[level];
};

int Cimage::getWidth(const int level) const{
  if (m_alloc == 0) {
    std::cerr << "First allocate (getWidth)" << std::endl;
    exit (1);
  }
  return m_widths[level];
};
 
int Cimage::getHeight(const int level) const{
  if (m_alloc == 0) {
    //alloc(1);
    std::cerr << "First allocate (getHeight)" << std::endl;
    exit (1);
  }    
  return m_heights[level];
};

/* 
int Cimage::getCWidth(const int beta, const int level) const{
  if (m_alloc == 0) {
    //alloc(1);
    std::cerr << "First allocate (getCWidth)" << std::endl;
    exit (1);
  }    
  return (m_widths[level] - 1) / beta + 1;
};
*/
/* 
int Cimage::getCHeight(const int beta, const int level) const{
  if (m_alloc == 0) {
    //alloc(1);
    std::cerr << "First allocate (getCHeight)" << std::endl;
    exit (1);
  }    
  return (m_heights[level] - 1) / beta + 1;
};
*/
 
Vec3f Cimage::getColor(const float x, const float y, const int level) const{
#ifdef FURUKAWA_DEBUG
  if (m_alloc != 2) {
    std::cerr << "First allocate" << std::endl;
    exit (1);
  }
#endif

#ifdef FURUKAWA_IMAGE_BICUBIC
  const int x1 = (int)floor(x);      const int y1 = (int)floor(y);
  const float p = x - x1;      const float q = y - y1;
  
  float f = 1+p;
  const float wx0 = (((-1) * f + 5) * f - 8) * f + 4;
  f = 2-p;
  const float wx3 = (((-1) * f + 5) * f - 8) * f + 4;
  f = p;
  const float wx1 = (((1) * f - 2) * f) * f + 1;
  f = 1 - p;
  const float wx2 = (((1) * f - 2) * f) * f + 1;
  
  f = 1+q;
  const float wy0 = (((-1) * f + 5) * f - 8) * f + 4;
  f = 2-q;
  const float wy3 = (((-1) * f + 5) * f - 8) * f + 4;
  f = q;
  const float wy1 = (((1) * f - 2) * f) * f + 1;
  f = 1 - q;
  const float wy2 = (((1) * f - 2) * f) * f + 1;
  
  const int offset = m_widths[level] * 3;
  const int index0 = ((y1 - 1) * m_widths[level] + x1 - 1) * 3;
  const int index1 = index0 + offset;
  const int index2 = index1 + offset;
  const int index3 = index2 + offset;
  
#ifdef FURUKAWA_IMAGE_GAMMA
  const float& r00 = m_dimages[level][index0];
  const float& g00 = m_dimages[level][index0+1];
  const float& b00 = m_dimages[level][index0+2];
  const float& r01 = m_dimages[level][index0+3];
  const float& g01 = m_dimages[level][index0+4];
  const float& b01 = m_dimages[level][index0+5];
  const float& r02 = m_dimages[level][index0+6];
  const float& g02 = m_dimages[level][index0+7];
  const float& b02 = m_dimages[level][index0+8];
  const float& r03 = m_dimages[level][index0+9];
  const float& g03 = m_dimages[level][index0+10];
  const float& b03 = m_dimages[level][index0+11];
  
  const float& r10 = m_dimages[level][index1];
  const float& g10 = m_dimages[level][index1+1];
  const float& b10 = m_dimages[level][index1+2];
  const float& r11 = m_dimages[level][index1+3];
  const float& g11 = m_dimages[level][index1+4];
  const float& b11 = m_dimages[level][index1+5];
  const float& r12 = m_dimages[level][index1+6];
  const float& g12 = m_dimages[level][index1+7];
  const float& b12 = m_dimages[level][index1+8];
  const float& r13 = m_dimages[level][index1+9];
  const float& g13 = m_dimages[level][index1+10];
  const float& b13 = m_dimages[level][index1+11];
  
  const float& r20 = m_dimages[level][index2];
  const float& g20 = m_dimages[level][index2+1];
  const float& b20 = m_dimages[level][index2+2];
  const float& r21 = m_dimages[level][index2+3];
  const float& g21 = m_dimages[level][index2+4];
  const float& b21 = m_dimages[level][index2+5];
  const float& r22 = m_dimages[level][index2+6];
  const float& g22 = m_dimages[level][index2+7];
  const float& b22 = m_dimages[level][index2+8];
  const float& r23 = m_dimages[level][index2+9];
  const float& g23 = m_dimages[level][index2+10];
  const float& b23 = m_dimages[level][index2+11];
  
  const float& r30 = m_dimages[level][index3];
  const float& g30 = m_dimages[level][index3+1];
  const float& b30 = m_dimages[level][index3+2];
  const float& r31 = m_dimages[level][index3+3];
  const float& g31 = m_dimages[level][index3+4];
  const float& b31 = m_dimages[level][index3+5];
  const float& r32 = m_dimages[level][index3+6];
  const float& g32 = m_dimages[level][index3+7];
  const float& b32 = m_dimages[level][index3+8];
  const float& r33 = m_dimages[level][index3+9];
  const float& g33 = m_dimages[level][index3+10];
  const float& b33 = m_dimages[level][index3+11];
#else
  const unsigned char& r00 = m_images[level][index0];
  const unsigned char& g00 = m_images[level][index0+1];
  const unsigned char& b00 = m_images[level][index0+2];
  const unsigned char& r01 = m_images[level][index0+3];
  const unsigned char& g01 = m_images[level][index0+4];
  const unsigned char& b01 = m_images[level][index0+5];
  const unsigned char& r02 = m_images[level][index0+6];
  const unsigned char& g02 = m_images[level][index0+7];
  const unsigned char& b02 = m_images[level][index0+8];
  const unsigned char& r03 = m_images[level][index0+9];
  const unsigned char& g03 = m_images[level][index0+10];
  const unsigned char& b03 = m_images[level][index0+11];
  
  const unsigned char& r10 = m_images[level][index1];
  const unsigned char& g10 = m_images[level][index1+1];
  const unsigned char& b10 = m_images[level][index1+2];
  const unsigned char& r11 = m_images[level][index1+3];
  const unsigned char& g11 = m_images[level][index1+4];
  const unsigned char& b11 = m_images[level][index1+5];
  const unsigned char& r12 = m_images[level][index1+6];
  const unsigned char& g12 = m_images[level][index1+7];
  const unsigned char& b12 = m_images[level][index1+8];
  const unsigned char& r13 = m_images[level][index1+9];
  const unsigned char& g13 = m_images[level][index1+10];
  const unsigned char& b13 = m_images[level][index1+11];
  
  const unsigned char& r20 = m_images[level][index2];
  const unsigned char& g20 = m_images[level][index2+1];
  const unsigned char& b20 = m_images[level][index2+2];
  const unsigned char& r21 = m_images[level][index2+3];
  const unsigned char& g21 = m_images[level][index2+4];
  const unsigned char& b21 = m_images[level][index2+5];
  const unsigned char& r22 = m_images[level][index2+6];
  const unsigned char& g22 = m_images[level][index2+7];
  const unsigned char& b22 = m_images[level][index2+8];
  const unsigned char& r23 = m_images[level][index2+9];
  const unsigned char& g23 = m_images[level][index2+10];
  const unsigned char& b23 = m_images[level][index2+11];
  
  const unsigned char& r30 = m_images[level][index3];
  const unsigned char& g30 = m_images[level][index3+1];
  const unsigned char& b30 = m_images[level][index3+2];
  const unsigned char& r31 = m_images[level][index3+3];
  const unsigned char& g31 = m_images[level][index3+4];
  const unsigned char& b31 = m_images[level][index3+5];
  const unsigned char& r32 = m_images[level][index3+6];
  const unsigned char& g32 = m_images[level][index3+7];
  const unsigned char& b32 = m_images[level][index3+8];
  const unsigned char& r33 = m_images[level][index3+9];
  const unsigned char& g33 = m_images[level][index3+10];
  const unsigned char& b33 = m_images[level][index3+11];
#endif
  // separate x and y
  const float row0[3] = {wx0 * r00 + wx1 * r01 + wx2 * r02 + wx3 * r03,
			 wx0 * g00 + wx1 * g01 + wx2 * g02 + wx3 * g03,
			 wx0 * b00 + wx1 * b01 + wx2 * b02 + wx3 * b03};
  const float row1[3] = {wx0 * r10 + wx1 * r11 + wx2 * r12 + wx3 * r13,
			 wx0 * g10 + wx1 * g11 + wx2 * g12 + wx3 * g13,
			 wx0 * b10 + wx1 * b11 + wx2 * b12 + wx3 * b13};
  const float row2[3] = {wx0 * r20 + wx1 * r21 + wx2 * r22 + wx3 * r23,
			 wx0 * g20 + wx1 * g21 + wx2 * g22 + wx3 * g23,
			 wx0 * b20 + wx1 * b21 + wx2 * b22 + wx3 * b23};
  const float row3[3] = {wx0 * r30 + wx1 * r31 + wx2 * r32 + wx3 * r33,
			 wx0 * g30 + wx1 * g31 + wx2 * g32 + wx3 * g33,
			 wx0 * b30 + wx1 * b31 + wx2 * b32 + wx3 * b33};

  float r = wy0 * row0[0] + wy1 * row1[0] + wy2 * row2[0] + wy3 * row3[0];
  float g = wy0 * row0[1] + wy1 * row1[1] + wy2 * row2[1] + wy3 * row3[1];
  float b = wy0 * row0[2] + wy1 * row1[2] + wy2 * row2[2] + wy3 * row3[2];
  
  return Vec3f(r, g, b);
#else
  // Bilinear case
  const int lx = (int)floor(x);
  const int ly = (int)floor(y);
  const int index = 3 * (ly * m_widths[level] + lx);

  const float dx1 = x - lx;  const float dx0 = 1.0f - dx1;
  const float dy1 = y - ly;  const float dy0 = 1.0f - dy1;
  
  const float f00 = dx0 * dy0;  const float f01 = dx0 * dy1;
  const float f10 = dx1 * dy0;  const float f11 = dx1 * dy1;
  const int index2 = index + 3 * m_widths[level];
    
#ifdef FURUKAWA_IMAGE_GAMMA
  const float* fp0 = &m_dimages[level][index] - 1;
  const float* fp1 = &m_dimages[level][index2] - 1;
  float r = 0.0f;  float g = 0.0f;  float b = 0.0f;
  r += *(++fp0) * f00 + *(++fp1) * f01;
  g += *(++fp0) * f00 + *(++fp1) * f01;
  b += *(++fp0) * f00 + *(++fp1) * f01;
  r += *(++fp0) * f10 + *(++fp1) * f11;
  g += *(++fp0) * f10 + *(++fp1) * f11;
  b += *(++fp0) * f10 + *(++fp1) * f11;
  return Vec3f(r, g, b);
  /*
  return Vec3f(m_dimages[level][index] * f00 + m_dimages[level][index + 3] * f10 +
               m_dimages[level][index2] * f01 + m_dimages[level][index2 + 3] * f11,
               m_dimages[level][index + 1] * f00 + m_dimages[level][index + 4] * f10 +
               m_dimages[level][index2 + 1] * f01 + m_dimages[level][index2 + 4] * f11,
               m_dimages[level][index + 2] * f00 + m_dimages[level][index + 5] * f10 +
               m_dimages[level][index2 + 2] * f01 + m_dimages[level][index2 + 5] * f11);
  */
#else
  const unsigned char* ucp0 = &m_images[level][index] - 1;
  const unsigned char* ucp1 = &m_images[level][index2] - 1;
  float r = 0.0f;  float g = 0.0f;  float b = 0.0f;
  r += *(++ucp0) * f00 + *(++ucp1) * f01;
  g += *(++ucp0) * f00 + *(++ucp1) * f01;
  b += *(++ucp0) * f00 + *(++ucp1) * f01;
  r += *(++ucp0) * f10 + *(++ucp1) * f11;
  g += *(++ucp0) * f10 + *(++ucp1) * f11;
  b += *(++ucp0) * f10 + *(++ucp1) * f11;
  return Vec3f(r, g, b);
  /*
  return Vec3f(m_images[level][index] * f00 + m_images[level][index + 3] * f10 +
               m_images[level][index2] * f01 + m_images[level][index2 + 3] * f11,
               
               m_images[level][index + 1] * f00 + m_images[level][index + 4] * f10 +
               m_images[level][index2 + 1] * f01 + m_images[level][index2 + 4] * f11,
               
               m_images[level][index + 2] * f00 + m_images[level][index + 5] * f10 +
               m_images[level][index2 + 2] * f01 + m_images[level][index2 + 5] * f11);
  */
#endif
  /*
  const int lx = (int)floor(x);    const int ux = lx + 1;
  const int ly = (int)floor(y);    const int uy = ly + 1;

  const Vec3f vc[2][2] = {{getColor(lx, ly, level), getColor(lx, uy, level)},
			  {getColor(ux, ly, level), getColor(ux, uy, level)}};
  
  const Vec3f color = vc[0][0] * (ux - x) * (uy - y) + vc[0][1] * (ux - x) * (y - ly) +
    vc[1][0] * (x - lx) * (uy - y) + vc[1][1] * (x - lx) * (y - ly);
  return color;
  */
#endif
};

void Cimage::setColor(const int ix, const int iy, const int level,
                      const Vec3f& rgb) {
#ifdef FURUKAWA_DEBUG
  if (m_alloc != 2) {
    std::cerr << "First allocate" << std::endl;
    exit (1);
  }  
#endif  
  const int index = (iy * m_widths[level] + ix) * 3;

#ifdef FURUKAWA_IMAGE_GAMMA
  m_dimages[level][index] = rgb[0];
  m_dimages[level][index+1] = rgb[1];
  m_dimages[level][index+2] = rgb[2];
#else
  m_images[level][index] = (unsigned char)floor(rgb[0] + 0.5f);
  m_images[level][index+1] = (unsigned char)floor(rgb[1] + 0.5f);
  m_images[level][index+2] = (unsigned char)floor(rgb[2] + 0.5f);
#endif
};

Vec3f Cimage::getColor(const int ix, const int iy, const int level) const{
#ifdef FURUKAWA_DEBUG
  if (m_alloc != 2) {
    std::cerr << "First allocate" << std::endl;
    exit (1);
  }  
#endif  
  const int index = (iy * m_widths[level] + ix) * 3;

#ifdef FURUKAWA_IMAGE_GAMMA
  return Vec3f(m_dimages[level][index],
	       m_dimages[level][index+1],
	       m_dimages[level][index+2]);
#else
  return Vec3f(m_images[level][index],
	       m_images[level][index+1],
	       m_images[level][index+2]);
#endif
};

int Cimage::getMask(const float fx, const float fy, const int level) const{
  if (m_alloc != 2) {
    std::cerr << "First allocate" << std::endl;
    exit (1);
  }    
  
  if (m_masks[level].empty())
    return 1;
  
  const int ix = (int)floor(fx + 0.5f);
  const int iy = (int)floor(fy + 0.5f);
  return getMask(ix, iy, level);
};

int Cimage::getMask(const int ix, const int iy, const int level) const{
  if (m_alloc != 2) {
    std::cerr << "First allocate" << std::endl;
    exit (1);
  }    

  if (m_masks[level].empty())
    return 1;

  if (ix < 0 || m_widths[level] <= ix || iy < 0 || m_heights[level] <= iy)
    return 1;
  
  const int index = iy * m_widths[level] + ix;
  return m_masks[level][index];
};

int Cimage::getEdge(const float fx, const float fy, const int level) const{
  if (m_alloc != 2) {
    std::cerr << "First allocate" << std::endl;
    exit (1);
  }    

  if (m_edges[level].empty())
    return 1;    
  const int ix = (int)floor(fx + 0.5f);
  const int iy = (int)floor(fy + 0.5f);
  return getEdge(ix, iy, level);
};

int Cimage::getEdge(const int ix, const int iy, const int level) const{
  if (m_alloc != 2) {
    std::cerr << "First allocate" << std::endl;
    exit (1);
  }    
  
  if (m_edges[level].empty())
    return 1;

  if (ix < 0 || m_widths[level] <= ix || iy < 0 || m_heights[level] <= iy)
    return 1;
  
  const int index = iy * m_widths[level] + ix;
  return m_edges[level][index];
};
  
};

#endif // IMAGE_H
