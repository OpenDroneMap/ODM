#include <set>
#include "point.h"
#include "detector.h"

using namespace PMVS3;
using namespace std;

void Cdetector::setGaussD(const float sigmaD, std::vector<float>& gaussD) {
  //----------------------------------------------------------------------  
  const int marginD = (int)ceil(2 * sigmaD);
  const int sizeD = 2 * marginD + 1;
    
  gaussD.resize(sizeD);

  //----------------------------------------------------------------------
  // set m_gaussD
  float denom = 0.0;
  for (int x = 0; x < sizeD; ++x) {
    int xtmp = x - marginD;
    const float dtmp = xtmp * exp(- (xtmp * xtmp) / (2 * sigmaD * sigmaD));
    gaussD[x] = dtmp;
    if (0.0 < dtmp)
      denom += dtmp;
  }
  
  for (int x = 0; x < sizeD; ++x)
    gaussD[x] /= denom;
}

void Cdetector::setGaussI(const float sigmaI, std::vector<float>& gaussI) {
  const int marginI = (int)ceil(2 * sigmaI);
  const int sizeI = 2 * marginI + 1;
  
  gaussI.resize(sizeI);
  
  //----------------------------------------------------------------------
  // set m_gaussI
  float denom = 0.0;
  for (int x = 0; x < sizeI; ++x) {
    int xtmp = x - marginI;
    const float dtmp = exp(- (xtmp * xtmp) / (2 * sigmaI * sigmaI));
    gaussI[x] = dtmp;
    denom += dtmp;
  }
  for (int x = 0; x < sizeI; ++x)
    gaussI[x] /= denom;
}


float Cdetector::setThreshold(std::multiset<Cpoint>& grid) {
  float ave = 0.0;
  float ave2 = 0.0;
  multiset<Cpoint>::iterator begin = grid.begin();
  multiset<Cpoint>::iterator end = grid.end();
  int count = 0;
  while (begin != end) {
    count++;
    ave += begin->m_response;
    ave2 += begin->m_response * begin->m_response;
    begin++;
  }
  if (count == 0)
    count = 1;
  ave /= count;      ave2 /= count;
  ave2 = sqrt(max(0.0f, ave2 - ave * ave));
  const float threshold = ave + ave2;

  //cout << ave << ' ' << ave2 << endl;

  return threshold;
}

int Cdetector::isCloseBoundary(const int x, const int y, const int margin) const {
  if (m_mask.empty())
    return 0;

  if (x - margin < 0 || m_width <= x + margin ||
      y - margin < 0 || m_height <= y + margin)
    return 1;
  
  for (int j = -margin; j <= margin; ++j) {
    const int ytmp = y + j;
    for (int i = -margin; i <= margin; ++i) {
      const int xtmp = x + i;

      if (m_mask[ytmp][xtmp] == 0)
	return 1;
    }
  }
  return 0;
}
