#include "harris.h"

using namespace PMVS3;
using namespace std;

void Charris::init(const std::vector<unsigned char>& image,
		   const std::vector<unsigned char>& mask,
                   const std::vector<unsigned char>& edge) {
  m_image.clear();
  m_image.resize(m_height);
  int count = 0;
  for (int y = 0; y < m_height; ++y) {
    m_image[y].resize(m_width);
    for (int x = 0; x < m_width; ++x) {
      m_image[y][x][0] = ((int)image[count++]) / 255.0f;
      m_image[y][x][1] = ((int)image[count++]) / 255.0f;
      m_image[y][x][2] = ((int)image[count++]) / 255.0f;
    }
  }

  m_mask.clear();
  if (!mask.empty() || !edge.empty()) {
    m_mask.resize(m_height);
    count = 0;
    for (int y = 0; y < m_height; ++y) {
      m_mask[y].resize(m_width);
      for (int x = 0; x < m_width; ++x) {
        if (mask.empty())
          m_mask[y][x] = edge[count++];
        else if (edge.empty())
          m_mask[y][x] = mask[count++];
        else {
          if (mask[count] && edge[count])
            m_mask[y][x] = (unsigned char)255;
          else
            m_mask[y][x] = 0;
          count++;
        }
      }
    }
  }
  
  setGaussD(m_sigmaD, m_gaussD);
  setGaussI(m_sigmaI, m_gaussI); 
}

void Charris::setDerivatives(void) {
  // set m_dIdx, m_dIdy
  preprocess();
  
  // now set m_dIdxdIdx, m_dIdydIdy, m_dIdxDIdy
  preprocess2();
}

void Charris::preprocess2(void) {
  m_dIdxdIdx.clear();  m_dIdydIdy.clear();  m_dIdxdIdy.clear();
  m_dIdxdIdx.resize(m_height);
  m_dIdydIdy.resize(m_height);
  m_dIdxdIdy.resize(m_height);
  for (int y = 0; y < m_height; ++y) {
    m_dIdxdIdx[y].resize(m_width);
    m_dIdydIdy[y].resize(m_width);
    m_dIdxdIdy[y].resize(m_width);
    for (int x = 0; x < m_width; ++x) {
      m_dIdxdIdx[y][x] = m_dIdydIdy[y][x] = m_dIdxdIdy[y][x] = 0.0;
      if (!m_mask.empty() && m_mask[y][x] == 0)	continue;
      
      m_dIdxdIdx[y][x] += m_dIdx[y][x] * m_dIdx[y][x];
      m_dIdydIdy[y][x] += m_dIdy[y][x] * m_dIdy[y][x];
      m_dIdxdIdy[y][x] += m_dIdx[y][x] * m_dIdy[y][x];
    }
  }

  {
    vector<vector<Vec3f> >().swap(m_dIdx);
    vector<vector<Vec3f> >().swap(m_dIdy);
  }
  
  //----------------------------------------------------------------------
  // blur
  vector<vector<float> > vvftmp;
  vvftmp.resize(m_height);
  for (int y = 0; y < m_height; ++y) {
    vvftmp[y].resize(m_width);
    for (int x = 0; x < m_width; ++x)
      vvftmp[y][x] = 0.0;
  }
  
  //----------------------------------------------------------------------
  // m_dIdxdIdx
  convolveX(m_dIdxdIdx, m_mask, m_gaussI, vvftmp);
  convolveY(m_dIdxdIdx, m_mask, m_gaussI, vvftmp);
  
  //----------------------------------------------------------------------
  // m_dIdydIdy
  convolveX(m_dIdydIdy, m_mask, m_gaussI, vvftmp);
  convolveY(m_dIdydIdy, m_mask, m_gaussI, vvftmp);
  
  //----------------------------------------------------------------------
  // m_dIdxdIdy
  convolveX(m_dIdxdIdy, m_mask, m_gaussI, vvftmp);
  convolveY(m_dIdxdIdy, m_mask, m_gaussI, vvftmp);
}

void Charris::preprocess(void) {
  vector<vector<Vec3f> > vvvftmp;
  vvvftmp.resize(m_height);
  for (int y = 0; y < m_height; ++y) {
    vvvftmp[y].resize(m_width);
    for (int x = 0; x < m_width; ++x)
      vvvftmp[y][x] = Vec3f();
  }
  
  m_dIdx = m_image;

  vector<float> dfilter, ifilter;
  dfilter.resize(3);
  dfilter[0] = -0.5;  dfilter[1] = 0;  dfilter[2] = 0.5;
  ifilter.resize(3);
  ifilter[0] = 1.0 / 3.0;  ifilter[1] = 1.0 / 3.0;  ifilter[2] = 1.0 / 3.0;

  convolveX(m_dIdx, m_mask, dfilter, vvvftmp);
  convolveY(m_dIdx, m_mask, ifilter, vvvftmp);
  
  m_dIdy = m_image;
  convolveX(m_dIdy, m_mask, ifilter, vvvftmp);
  convolveY(m_dIdy, m_mask, dfilter, vvvftmp);
}

void Charris::setResponse(void) {
  m_response.clear();
  m_response.resize(m_height);
  for (int y = 0; y < m_height; ++y) {
    m_response[y].resize(m_width);
    for (int x = 0; x < m_width; ++x) {
      m_response[y][x] = 0.0;
      if (!m_mask.empty() && m_mask[y][x] == 0)	continue;
      
      const float D = m_dIdxdIdx[y][x] * m_dIdydIdy[y][x] - m_dIdxdIdy[y][x] * m_dIdxdIdy[y][x];
      const float tr = m_dIdxdIdx[y][x] + m_dIdydIdy[y][x];
      m_response[y][x] = D - 0.06 * tr * tr;
    }
  }
  
  //----------------------------------------------------------------------
  // suppress non local max
  vector<vector<float> > vvftmp = m_response;
  for (int y = 1; y < m_height - 1; ++y) {
    for (int x = 1; x < m_width - 1; ++x) {
      if (m_response[y][x] < m_response[y][x+1] ||
	  m_response[y][x] < m_response[y][x-1] ||
	  m_response[y][x] < m_response[y+1][x] ||
	  m_response[y][x] < m_response[y-1][x])
	vvftmp[y][x] = 0.0;
    }
  }
  
  vvftmp.swap(m_response);
}

void Charris::run(const std::vector<unsigned char>& image,
                  const std::vector<unsigned char>& mask,
                  const std::vector<unsigned char>& edge,
		  const int width, const int height,
		  const int gspeedup, const float sigma,
		  std::multiset<Cpoint> & result) {

  cerr << "Harris running ..." << flush;
  m_width = width;       m_height = height;
  m_sigmaD = sigma;      m_sigmaI = sigma;
  init(image, mask, edge);
  setDerivatives();  setResponse();

  const int factor = 2;
  const int maxPointsGrid = factor * factor;
  const int gridsize = gspeedup * factor;

  const int w = (m_width + gridsize - 1) / gridsize;
  const int h = (m_height + gridsize - 1) / gridsize;
  
  vector<vector<multiset<Cpoint> > > resultgrids;
  resultgrids.resize(h);
  for (int y = 0; y < h; ++y)
    resultgrids[y].resize(w);

  const int margin = (int)m_gaussD.size() / 2;
  for (int y = margin; y < m_height - margin; ++y) {
    for (int x = margin; x < m_width - margin; ++x) {
      if (m_response[y][x] == 0.0)
	continue;

      const int x0 = min(x / gridsize, w - 1);
      const int y0 = min(y / gridsize, h - 1);
      
      if ((int)resultgrids[y0][x0].size() < maxPointsGrid ||
	  resultgrids[y0][x0].begin()->m_response < m_response[y][x]) {
	Cpoint p;
	p.m_icoord = Vec3f(x, y, 1.0f);
	p.m_response = m_response[y][x];
	p.m_type = 0;
	
	resultgrids[y0][x0].insert(p);
	if (maxPointsGrid < (int)resultgrids[y0][x0].size())
	  resultgrids[y0][x0].erase(resultgrids[y0][x0].begin ());
      }
    }
  }  
  
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      //const float threshold = setThreshold(resultgrids[y][x]);      
      multiset<Cpoint>::iterator begin = resultgrids[y][x].begin();
      multiset<Cpoint>::iterator end = resultgrids[y][x].end();
      while (begin != end) {
	//if (threshold <= begin->m_response)
	  result.insert(*begin);
	begin++;
      }
    }

  cerr << (int)result.size() << " harris done" << endl;
}
