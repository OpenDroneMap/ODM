#include "dog.h"
#include "point.h"

using namespace PMVS3;
using namespace std;

int Cdog::notOnEdge(const std::vector<std::vector<float> >& dog, int x, int y) {
  return 1;
  //const float thresholdEdge = 0.06f;
  const float thresholdEdge = 0.06f;
  
  const float H00 = dog[y][x-1] - 2.0f * dog[y][x] + dog[y][x+1];
  const float H11 = dog[y-1][x] - 2.0f * dog[y][x] + dog[y+1][x];
  const float H01 = ((dog[y+1][x+1] - dog[y-1][x+1]) - (dog[y+1][x-1] - dog[y-1][x-1])) / 4.0f;
  const float det = H00 * H11 - H01 * H01;
  const float trace = H00 + H11;
  return det > thresholdEdge * trace * trace;
}

float Cdog::getResponse(const std::vector<std::vector<float> >& pdog,
		  const std::vector<std::vector<float> >& cdog,
		  const std::vector<std::vector<float> >& ndog,
		  const int x, const int y) {
  return fabs(getResponse(pdog, x, y) + getResponse(cdog, x, y) + getResponse(ndog, x, y) +
	      (cdog[y][x] - pdog[y][x]) + (cdog[y][x] - ndog[y][x]));
}

float Cdog::getResponse(const std::vector<std::vector<float> >& dog,
		  const int x, const int y) {
  const float sum =
    dog[y-1][x-1] + dog[y  ][x-1] +
    dog[y+1][x-1] + dog[y-1][x  ] +
    dog[y+1][x  ] + dog[y-1][x+1] +
    dog[y  ][x+1] + dog[y+1][x+1];

  return 8 * dog[y][x] - sum;
}  

int Cdog::isLocalMax(const std::vector<std::vector<float> >& dog,
		     const int x, const int y) {
  const float value = dog[y][x];

  if (0.0 < value) {
    if (dog[y-1][x-1] < value && dog[y  ][x-1] < value &&
	dog[y+1][x-1] < value && dog[y-1][x  ] < value &&
	dog[y+1][x  ] < value && dog[y-1][x+1] < value &&
	dog[y  ][x+1] < value && dog[y+1][x+1] < value)
      return 1;
  }
  else {
    if (dog[y-1][x-1] > value && dog[y  ][x-1] > value &&
	dog[y+1][x-1] > value && dog[y-1][x  ] > value &&
	dog[y+1][x  ] > value && dog[y-1][x+1] > value &&
	dog[y  ][x+1] > value && dog[y+1][x+1] > value)
      return -1;
  }
  return 0;
}

int Cdog::isLocalMax(const std::vector<std::vector<float> >& pdog,
		     const std::vector<std::vector<float> >& cdog,
		     const std::vector<std::vector<float> >& ndog,
		     const int x, const int y) {
  /*
  const int flag0 = isLocalMax(pdog, x, y);
  const int flag1 = isLocalMax(cdog, x, y);
  const int flag2 = isLocalMax(ndog, x, y);
  
  if (flag0 == 1 && flag1 == 1 && flag2 == 1) {
    if (pdog[y][x] < cdog[y][x] && ndog[y][x] < cdog[y][x])
      return 1;
    else
      return 0;
  }
  else if (flag0 == -1 && flag1 == -1 && flag2 == -1) {
    if (cdog[y][x] < pdog[y][x] && cdog[y][x] < ndog[y][x])
      return -1;
    else
      return 0;
  }
  return 0;
  */

  const int flag = isLocalMax(cdog, x, y);
  
  if (flag == 1) {
    if (pdog[y][x] < cdog[y][x] && ndog[y][x] < cdog[y][x])
      return 1;
    else
      return 0;
  }
  else if (flag == -1) {
    if (cdog[y][x] < pdog[y][x] && cdog[y][x] < ndog[y][x])
      return -1;
    else
      return 0;
  }
  return 0;
}

void Cdog::setDOG(const std::vector<std::vector<float> >& cres,
		  const std::vector<std::vector<float> >& nres,
		  std::vector<std::vector<float> >& dog) {
  const int height = (int)cres.size();
  const int width = (int)cres[0].size();

  dog = nres;
  for (int y = 0; y < height; ++y)
    for (int x = 0; x < width; ++x)
      dog[y][x] -= cres[y][x];
}

void Cdog::run(const std::vector<unsigned char>& image,
               const std::vector<unsigned char>& mask,
               const std::vector<unsigned char>& edge,
	       const int width, const int height,
	       const int gspeedup,
	       const float firstScale,   // 1.4f
	       const float lastScale,    // 4.0f
	       std::multiset<Cpoint> & result) {
  cerr << "DoG running..." << flush;
  m_width = width;
  m_height = height;
  
  m_firstScale = firstScale;
  m_lastScale = lastScale;
  
  init(image, mask, edge);

  const int factor = 2;
  const int maxPointsGrid = factor * factor;
  const int gridsize = gspeedup * factor;
  
  const int w = (m_width + gridsize - 1) / gridsize;
  const int h = (m_height + gridsize - 1) / gridsize;
  
  /*
  const int gridsize = 50;
  const int w = (int)ceil(m_width / (float)gridsize);
  const int h = (int)ceil(m_height / (float)gridsize);
  const int maxPointsGrid = (int)(m_width * m_height * 0.0025 / (w * h));
  */
  
  vector<vector<multiset<Cpoint> > > resultgrids;
  resultgrids.resize(h);
  for (int y = 0; y < h; ++y)
    resultgrids[y].resize(w);

  const float scalestep = pow(2.0f, 1 / 2.0f);
  //const float scalestep = pow(2.0f, 1.0f);
  const int steps = max(4, (int)ceil(log(m_lastScale / m_firstScale) / log(scalestep)));
  
  vector<vector<float> > pdog, cdog, ndog, cres, nres;

  setRes(m_firstScale, cres);
  setRes(m_firstScale * scalestep, nres);
  setDOG(cres, nres, cdog);
  cres.swap(nres);
  setRes(m_firstScale * scalestep * scalestep, nres);
  setDOG(cres, nres, ndog);

  vector<vector<unsigned char> > alreadydetected;
  alreadydetected.resize(m_height);
  for (int y = 0; y < m_height; ++y) {
    alreadydetected[y].resize(m_width);
    for (int x = 0; x < m_width; ++x) {
      alreadydetected[y][x] = (unsigned char)0;
    }
  }

  for (int i = 2; i <= steps - 1; ++i) {
    const float cscale = m_firstScale * pow(scalestep, i + 1);
    cres.swap(nres);
    setRes(cscale, nres);
    
    pdog.swap(cdog);
    cdog.swap(ndog);
    setDOG(cres, nres, ndog);
    
    const int margin = (int)ceil(2 * cscale);
    // now 3 response maps are ready
    for (int y = margin; y < m_height - margin; ++y) {
      for (int x = margin; x < m_width - margin; ++x) {
	if (alreadydetected[y][x])
	  continue;
	if (cdog[y][x] == 0.0)
	  continue;
	//if (isCloseBoundary(x, y, margin))
        //continue;
	
	// check local maximum
	if (isLocalMax(pdog, cdog, ndog, x, y) && notOnEdge(cdog, x, y)) {
	  const int x0 = min(x / gridsize, w - 1);
	  const int y0 = min(y / gridsize, h - 1);

	  alreadydetected[y][x] = 1;
	  Cpoint p;
	  p.m_icoord = Vec3f(x, y, 1.0f);
	  p.m_response = fabs(cdog[y][x]);
	  p.m_type = 1;
	  
	  resultgrids[y0][x0].insert(p);

	  if (maxPointsGrid < (int)resultgrids[y0][x0].size())
	    resultgrids[y0][x0].erase(resultgrids[y0][x0].begin());
	}
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
  
  cerr << (int)result.size() << " dog done" << endl;  
}

void Cdog::setRes(const float sigma,
		  std::vector<std::vector<float> >& res) {
  vector<float> gauss;
  setGaussI(sigma, gauss);
  
  vector<vector<Vec3f> > vvftmp;
  vvftmp.resize((int)m_image.size());
  for (int y = 0; y < (int)m_image.size(); ++y)
    vvftmp[y].resize((int)m_image[y].size());

  vector<vector<Vec3f> > restmp = m_image;
  convolveX(restmp, m_mask, gauss, vvftmp);
  convolveY(restmp, m_mask, gauss, vvftmp);

  res.resize((int)m_image.size());
  for (int y = 0; y < (int)m_image.size(); ++y) {
    res[y].resize((int)m_image[y].size());
    for (int x = 0; x < (int)m_image[y].size(); ++x)
      res[y][x] = norm(restmp[y][x]);
  }
}		  

void Cdog::init(const std::vector<unsigned char>& image,
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
}
