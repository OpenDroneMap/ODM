#include <fstream>
#include "photo.h"

using namespace std;
using namespace Image;

Cphoto::Cphoto(void) {
}

Cphoto::~Cphoto() {
}

void Cphoto::init(const std::string name, const std::string mname,
		  const std::string cname, const int maxLevel) {
  Cimage::init(name, mname, maxLevel);
  Ccamera::init(cname, maxLevel);
}

void Cphoto::init(const std::string name, const std::string mname,
                  const std::string ename,
		  const std::string cname, const int maxLevel) {
  Cimage::init(name, mname, ename, maxLevel);
  Ccamera::init(cname, maxLevel);
}

float Cphoto::ssd(const std::vector<Vec3f>& tex0,
		  const std::vector<Vec3f>& tex1) {
  float ans = 0.0f;
  for (int i = 0; i < (int)tex0.size(); ++i)
    ans += norm2(tex0[i] - tex1[i]);

  // Make sure that the score is below 2.0f
  ans /= (int)tex0.size() * (255.0 * 255.0 * 3.0);
  
  return ans;
}

float Cphoto::idot(const std::vector<Vec3f>& tex0,
		   const std::vector<Vec3f>& tex1) {
  if (tex0.empty() || tex1.empty()) {
    cerr << "Error in idot. Empty textures" << endl;
    exit (1);
  }
  float ans = 0.0;
  for (int i = 0; i < (int)tex0.size(); ++i) {
    ans += tex0[i] * tex1[i];
  }

  return 1.0f - ans / (3 * (int)tex0.size());
}

void Cphoto::idotC(const std::vector<Vec3f>& tex0,
		   const std::vector<Vec3f>& tex1, double* idc) {
  if (tex0.empty() || tex1.empty()) {
    cerr << "Error in idotC. Empty textures" << endl;
    exit (1);
  }
  idc[0] = 0.0;  idc[1] = 0.0;  idc[2] = 0.0;
  for (int i = 0; i < (int)tex0.size(); ++i) {
    for (int j = 0; j < 3; ++j)
      idc[j] += tex0[i][j] * tex1[i][j];
  }
  for (int j = 0; j < 3; ++j)
    idc[j] = 1.0 - idc[j] / (int)tex0.size();
}

void Cphoto::normalize(std::vector<Vec3f>& tex) {
  //----------------------------------------------------------------------
  // normalize average
  Vec3f ave;
  for (int i = 0; i < (int)tex.size(); ++i)
    ave += tex[i];
  ave /= (int)tex.size();

  for (int i = 0; i < (int)tex.size(); ++i)
    tex[i] -= ave;
  //----------------------------------------------------------------------  
  // compute variance
  float ave2 = 0.0f;
  for (int i = 0; i < (int)tex.size(); ++i)
    ave2 += tex[i] * tex[i];
  ave2 /= (int)tex.size() * 3;
  ave2 = sqrt(ave2);
  if (ave2 == 0.0f)
    ave2 = 1.0f;
  
  for (int i = 0; i < (int)tex.size(); ++i)
    tex[i] /= ave2;
}

void Cphoto::grabTex(const int level, const Vec2f& icoord,
		     const Vec2f& xaxis, const Vec2f& yaxis,
		     const int size, std::vector<Vec3f>& tex,
                     const int normalizef) const{
  const int margin = size / 2;
  
  // Check boundary condition
  const float maxx = icoord[0] + size * fabs(xaxis[0]) + size * fabs(yaxis[0]);
  const float minx = icoord[0] - size * fabs(xaxis[0]) - size * fabs(yaxis[0]);
  const float maxy = icoord[1] + size * fabs(xaxis[1]) + size * fabs(yaxis[1]);
  const float miny = icoord[1] - size * fabs(xaxis[1]) - size * fabs(yaxis[1]);
  
  tex.clear();
  if (minx < 0 || getWidth(level) - 1 <= maxx ||
      miny < 0 || getHeight(level) - 1 <= maxy)
    return;
     
  //tex.reserve(size * size);
  for (int y = -margin; y <= margin; ++y) {
    Vec2f v2ftmp = icoord - margin * xaxis + y * yaxis;
    for (int x = -margin; x <= margin; ++x) {
      tex.push_back(Cimage::getColor(v2ftmp[0], v2ftmp[1], level));
      v2ftmp += xaxis;
    }
  }

  if (normalizef)
    normalize(tex);
}

void Cphoto::grabTex(const int level, const Vec4f& coord,
		     const Vec4f& pxaxis, const Vec4f& pyaxis, const Vec4f& pzaxis,
		     const int size,
		     std::vector<Vec3f>& tex, float& weight,
                     const int normalizef) const {
  const int scale = 0x0001 << level;
  
  const Vec3f icoord3 = project(coord, level);
  const Vec2f icoord(icoord3[0], icoord3[1]);
  
  const Vec3f xaxis3 = project(coord + pxaxis * scale, level) - icoord3;
  const Vec2f xaxis(xaxis3[0], xaxis3[1]);
  
  const Vec3f yaxis3 = project(coord + pyaxis * scale, level) - icoord3;
  const Vec2f yaxis(yaxis3[0], yaxis3[1]);

  grabTex(level, icoord, xaxis, yaxis, size, tex, normalizef);

  Vec4f ray = m_center - coord;
  unitize(ray);
  weight = max(0.0f, pzaxis * ray);
}
