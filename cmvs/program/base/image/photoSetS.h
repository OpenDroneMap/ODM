#ifndef IMAGE_PHOTOSETS_H
#define IMAGE_PHOTOSETS_H

#include <map>
#include "photo.h"

namespace Image {

class CphotoSetS {
 public:
  CphotoSetS(void);
  virtual ~CphotoSetS();

  void init(const std::vector<int>& images, const std::string prefix,
            const int maxLevel, const int size, const int alloc);
  
  // grabTex given 2D sampling information
  void grabTex(const int index, const int level, const Vec2f& icoord,
	       const Vec2f& xaxis, const Vec2f& yaxis,
	       std::vector<Vec3f>& tex, const int normalizef = 1) const;

  // grabTex given 3D sampling information
  void grabTex(const int index, const int level, const Vec4f& coord,
	       const Vec4f& pxaxis, const Vec4f& pyaxis, const Vec4f& pzaxis,
	       std::vector<Vec3f>& tex, float& weight,
               const int normalizef = 1) const;
  
  void write(const std::string outdir);
  void free(void);
  void free(const int level);

  void setEdge(const float threshold);

  inline Vec3f project(const int index, const Vec4f& coord, const int level) const;
  inline Vec3f mult(const int index, const Vec4f& coord, const int level) const;
  
  inline int getWidth(const int index, const int level) const;
  inline int getHeight(const int index, const int level) const;

  inline Vec3f getColor(const Vec4f& coord, const int index,
                        const int level) const;
  inline Vec3f getColor(const int index, const float fx, const float fy,
                        const int level) const;
  inline Vec3f getColor(const int index, const int ix, const int iy,
                        const int level) const;
  
  inline int getMask(const Vec4f& coord, const int level) const;
  inline int getMask(const Vec4f& coord, const int index, const int level) const;
  inline int getMask(const int index, const float fx, const float fy,
                     const int level) const;
  inline int getMask(const int index, const int ix, const int iy,
                     const int level) const;

  inline int getEdge(const Vec4f& coord, const int index, const int level) const;
  inline int getEdge(const int index, const float fx, const float fy,
                     const int level) const;
  inline int getEdge(const int index, const int ix, const int iy,
                     const int level) const;
  
  static float incc(const std::vector<std::vector<Vec3f> >& texs,
		    const std::vector<float>& weights);

  int checkAngles(const Vec4f& coord, const std::vector<int>& indexes,
                  const float minAngle, const float maxAngle,
                  const int tau) const;
  
  void getMinMaxAngles(const Vec4f& coord, const std::vector<int>& indexes,
                       float& minAngle, float& maxAngle) const;

  float computeDepth(const int index, const Vec4f& coord) const;
  
  // Take care of indexes
  std::vector<int> m_images;
  std::vector<Cphoto> m_photos;

  int image2index(const int image) const;
  std::map<int, int> m_dict;
  
  // Number of cameras.
  int m_num;
  // Root directory
  std::string m_prefix;
  // maximum level
  int m_maxLevel;
  // Window size used to refine location
  int m_size;

  // getPAxes
  void getPAxes(const int index, const Vec4f& coord, const Vec4f& normal,
		Vec4f& pxaxis, Vec4f& pyaxis) const;

  // pairwise distance based on optical center and viewing direction
  void setDistances(void);
  std::vector<std::vector<float> > m_distances;
 protected:  
}; 
 
Vec3f CphotoSetS::project(const int index, const Vec4f& coord,
                                    const int level) const{
  return m_photos[index].project(coord, level);
};

Vec3f CphotoSetS::mult(const int index, const Vec4f& coord,
                                    const int level) const{
  return m_photos[index].mult(coord, level);
};
 
int CphotoSetS::getWidth(const int index, const int level) const {
  return m_photos[index].getWidth(level);
};
 
int CphotoSetS::getHeight(const int index, const int level) const {
  return m_photos[index].getHeight(level);
};

Vec3f CphotoSetS::getColor(const Vec4f& coord, const int index,
                          const int level) const {
  return m_photos[index].getColor(coord, level);
};
 
Vec3f CphotoSetS::getColor(const int index, const float fx, const float fy,
                          const int level) const {
  return m_photos[index].Image::Cimage::getColor(fx, fy, level);
};
 
Vec3f CphotoSetS::getColor(const int index, const int ix, const int iy,
                          const int level) const {
  return m_photos[index].Image::Cimage::getColor(ix, iy, level);
};
 
int CphotoSetS::getMask(const Vec4f& coord, const int level) const {
  for (int index = 0; index < m_num; ++index)
    if (getMask(coord, index, level) == 0)
      return 0;
  return 1;
};
 
int CphotoSetS::getMask(const Vec4f& coord, const int index,
                       const int level) const {
  return m_photos[index].getMask(coord, level);
};
 
int CphotoSetS::getMask(const int index, const float fx, const float fy,
                       const int level) const {
  return m_photos[index].Image::Cimage::getMask(fx, fy, level);
};

int CphotoSetS::getMask(const int index, const int ix, const int iy,
                       const int level) const {
  return m_photos[index].Image::Cimage::getMask(ix, iy, level); 
};

int CphotoSetS::getEdge(const Vec4f& coord, const int index,
                       const int level) const {
  return m_photos[index].getEdge(coord, level);
};
 
int CphotoSetS::getEdge(const int index, const float fx, const float fy,
                       const int level) const {
  return m_photos[index].Image::Cimage::getEdge(fx, fy, level);
};

int CphotoSetS::getEdge(const int index, const int ix, const int iy,
                       const int level) const {
  return m_photos[index].Image::Cimage::getEdge(ix, iy, level); 
};
 
};

#endif // IMAGE_PHOTOSETS_H
  
