#include <list>
#include <fstream>
#include "../numeric/mat4.h"
#include "image.h"
#include <setjmp.h>

extern "C" {
#include <jpeglib.h>
};

using namespace std;
using namespace Image;

/* min3 -- return minimum of 3 values */
#define min3(a, b, c) ((a)<(b) ? ((a)<(c) ? (a) : (c)) : ((b)<(c) ? (b) : (c)))
/* max3 -- return maximum of 3 values */
#define max3(a, b, c) ((a)>(b) ? ((a)>(c) ? (a) : (c)) : ((b)>(c) ? (b) : (c)))

Cimage::Cimage(void) {
  m_alloc = 0;
}

Cimage::~Cimage() {
}

void Cimage::completeName(const std::string& lhs, std::string& rhs,
                          const int color) {
  if (5 <= lhs.length() && lhs[lhs.length() - 4] == '.') {
    rhs = lhs;
    return;
  }

  // ppm jpg
  if (color) {
    string stmp0 = lhs + ".ppm";    string stmp1 = lhs + ".jpg";

    if (ifstream(stmp0.c_str()))
      rhs = stmp0;
    else if (ifstream(stmp1.c_str()))
      rhs = stmp1;
    else
      rhs = lhs;
  }
  // pgm pbm
  else {
    string stmp0 = lhs + ".pgm";    string stmp1 = lhs + ".pbm";

    if (ifstream(stmp0.c_str()))
      rhs = stmp0;
    else if (ifstream(stmp1.c_str()))
      rhs = stmp1;
    else
      rhs = lhs;
  }  
}

void Cimage::init(const std::string name, const std::string mname,
		  const int maxLevel) {
  m_alloc = 0;

  if (!name.empty())
    completeName(name, m_name, 1);
  if (!mname.empty())
    completeName(mname, m_mname, 0);
  m_maxLevel = maxLevel;
  //m_color = 1;

  if (m_maxLevel == 0) {
    cerr << "Number of level 0, set it to 1." << endl;
    m_maxLevel = 1;
  }
}

void Cimage::init(const std::string name, const std::string mname,
		  const std::string ename, const int maxLevel) {
  init(name, mname, maxLevel);
  if (!ename.empty())
    completeName(ename, m_ename, 0);
}

void Cimage::alloc(const int fast, const int filter) {
  if (m_alloc == 1 && fast == 1)
    return;
  if (m_alloc == 2)
    return;

  if (m_name.length() < 3) {
    cerr << "Image file name has less than 3 characters." << endl
	 << "Cannot allocate: " << m_name << endl;
    exit (1);
  }
  
  m_images.resize(m_maxLevel);  m_masks.resize(m_maxLevel);
  m_edges.resize(m_maxLevel);   m_widths.resize(m_maxLevel);
  m_heights.resize(m_maxLevel);

  if (readJpegImage(m_name, m_images[0], m_widths[0], m_heights[0], fast) == 0 &&
      readPPMImage(m_name, m_images[0], m_widths[0], m_heights[0], fast) == 0) {
    cerr << "Only jpeg and ppm formats are allowed. Stop allocation: "
	 << m_name << endl;
    return;
  }
  
#ifdef FURUKAWA_IMAGE_GAMMA
  m_dimages.resize(m_maxLevel);
  decodeGamma();
#endif
  
  // set widths, heights
  for (int level = 1; level < m_maxLevel; ++level) {
    m_widths[level] = m_widths[level - 1] / 2;
    m_heights[level] = m_heights[level - 1] / 2;
  }

  //setColor();
  m_alloc = 1;
  
  if (fast)
    return;
  //----------------------------------------------------------------------
  if (!m_mname.empty()) {
    if (readPGMImage(m_mname, m_masks[0], m_widths[0], m_heights[0], 0) ||
        readPBMImage(m_mname, m_masks[0], m_widths[0], m_heights[0], 0)) {
      // 255: in, 0 : out
      cerr << "Read mask: " << m_mname << endl;
      for (int i = 0; i < (int)m_masks[0].size(); ++i) {
        if (127 < (int)m_masks[0][i])
          m_masks[0][i] = (unsigned char)255;
        else
          m_masks[0][i] = (unsigned char)0;
      }
    }
    else {
      m_mname = "";
    }
  }
  //----------------------------------------------------------------------
  if (!m_ename.empty()) {
    if (readPGMImage(m_ename, m_edges[0], m_widths[0], m_heights[0], 0) ||
        readPBMImage(m_ename, m_edges[0], m_widths[0], m_heights[0], 0)) {
      cerr << "Read edge: " << m_ename << endl;
      // 255: in, 0 : out
      for (int i = 0; i < (int)m_edges[0].size(); ++i) {
        if (1 < (unsigned char)m_edges[0][i])
          m_edges[0][i] = (unsigned char)255;
        else
          m_edges[0][i] = (unsigned char)0;
      }
    }
    else {
      m_ename = "";
    }
  }
  
  //----------------------------------------------------------------------
  // build image/mask/edge pyramids
  buildImageMaskEdge(filter);
  
  m_alloc = 2;
}

#ifdef FURUKAWA_IMAGE_GAMMA
void Cimage::decodeGamma(void) {
  m_dimages[0].resize((int)m_images[0].size());
  for (int i = 0; i < (int)m_images[0].size(); ++i) {
    float ftmp = (float)m_images[0][i] / 255.0;
    ftmp = pow(ftmp, 2.2f);
    m_dimages[0][i] = ftmp;
  }
  
  vector<vector<unsigned char> >().swap(m_images);
}
#endif

/*
void Cimage::setColor(void) {
  m_color = 0;
  int index = 0;
  for (int y = 0; y < m_heights[0]; ++y) {
    if (m_color == 1)
      break;
    for (int x = 0; x < m_widths[0]; ++x) {
      const unsigned char r = m_images[0][index++];
      const unsigned char g = m_images[0][index++];
      const unsigned char b = m_images[0][index++];

      if (r != g || g != b || b != r) {
	m_color = 1;
	break;
      }
    }
  }
}
*/

void Cimage::free(const int freeLevel) {
  for (int l = 0; l < freeLevel; ++l) {
#ifdef FURUKAWA_IMAGE_GAMMA
    vector<float>().swap(m_dimages[l]);
#else
    vector<unsigned char>().swap(m_images[l]);
#endif
    if (!m_masks.empty())
      vector<unsigned char>().swap(m_masks[l]);
    if (!m_edges.empty())
      vector<unsigned char>().swap(m_edges[l]);
  }
}

void Cimage::free(void) {
  if (m_alloc != 0)
    m_alloc = 1;
  else
    m_alloc = 0;

  vector<vector<unsigned char> >().swap(m_images);
  vector<vector<unsigned char> >().swap(m_masks);
  vector<vector<unsigned char> >().swap(m_edges);
  //vector<int>().swap(m_widths);
  //vector<int>().swap(m_heights);
}

void Cimage::buildImageMaskEdge(const int filter) {
  buildImage(filter);

  if (!m_mname.empty())
    buildMask();

  if (!m_ename.empty())  
    buildEdge();
}

void Cimage::buildImage(const int filter) {
  Mat4 mask;
  mask[0] = Vec4(1.0, 3.0, 3.0, 1.0);  mask[1] = Vec4(3.0, 9.0, 9.0, 3.0);
  mask[2] = Vec4(3.0, 9.0, 9.0, 3.0);  mask[3] = Vec4(1.0, 3.0, 3.0, 1.0);

  float total = 64.0f;
  for (int y = 0; y < 4; ++y)
    for (int x = 0; x < 4; ++x) 
      mask[y][x] /= total;
  
  //----------------------------------------------------------------------
  // image
  for (int level = 1; level < m_maxLevel; ++level) {
    const int size = m_widths[level] * m_heights[level] * 3;
#ifdef FURUKAWA_IMAGE_GAMMA
    m_dimages[level].resize(size);
#else
    m_images[level].resize(size);
#endif
    for (int y = 0; y < m_heights[level]; ++y) {      
      for (int x = 0; x < m_widths[level]; ++x) {

	Vec3 color;
        if (filter == 2)
          color[0] = color[1] = color[2] = 255.0;
        
	float denom = 0.0;

	for (int j = -1; j < 3; ++j) {
	  const int ytmp = 2 * y + j;
	  if (ytmp < 0 || m_heights[level - 1] - 1 < ytmp)
	    continue;
	  
	  for (int i = -1; i < 3; ++i) {
	    const int xtmp = 2 * x + i;
	    if (xtmp < 0 || m_widths[level - 1] - 1 < xtmp)
	      continue;

	    const int index = (ytmp * m_widths[level - 1] + xtmp) * 3;
#ifdef FURUKAWA_IMAGE_GAMMA
            if (filter == 0) {
              color[0] += mask[j+1][i+1] * (double)m_dimages[level - 1][index];
              color[1] += mask[j+1][i+1] * (double)m_dimages[level - 1][index+1];
              color[2] += mask[j+1][i+1] * (double)m_dimages[level - 1][index+2];
              denom += mask[j+1][i+1];
            }
            else if (filter == 1) {
              color[0] = max(color[0], (double)m_dimages[level - 1][index]);
              color[1] = max(color[1], (double)m_dimages[level - 1][index+1]);
              color[2] = max(color[2], (double)m_dimages[level - 1][index+2]);
            }
            else {
              color[0] = min(color[0], (double)m_dimages[level - 1][index]);
              color[1] = min(color[1], (double)m_dimages[level - 1][index+1]);
              color[2] = min(color[2], (double)m_dimages[level - 1][index+2]);
            }
#else
            if (filter == 0) {
              color[0] += mask[j+1][i+1] * (double)m_images[level - 1][index];
              color[1] += mask[j+1][i+1] * (double)m_images[level - 1][index+1];
              color[2] += mask[j+1][i+1] * (double)m_images[level - 1][index+2];
              denom += mask[j+1][i+1];              
            }
            else if (filter == 1) {
              color[0] = max(color[0], (double)m_images[level - 1][index]);
              color[1] = max(color[1], (double)m_images[level - 1][index+1]);
              color[2] = max(color[2], (double)m_images[level - 1][index+2]);
            }
            else {
              color[0] = min(color[0], (double)m_images[level - 1][index]);
              color[1] = min(color[1], (double)m_images[level - 1][index+1]);
              color[2] = min(color[2], (double)m_images[level - 1][index+2]);
            }
#endif
          }
	}
        if (filter == 0)
          color /= denom;
	const int index = (y * m_widths[level] + x) * 3;
#ifdef FURUKAWA_IMAGE_GAMMA
	m_dimages[level][index] = color[0];
	m_dimages[level][index + 1] = color[1];
	m_dimages[level][index + 2] = color[2];
#else        
	m_images[level][index] = (unsigned char)((int)floor(color[0] + 0.5f));
	m_images[level][index + 1] = (unsigned char)((int)floor(color[1] + 0.5f));
	m_images[level][index + 2] = (unsigned char)((int)floor(color[2] + 0.5f));
#endif
      }
    }
  }
}

void Cimage::buildMask(void) {
  //----------------------------------------------------------------------
  // mask
  for (int level = 1; level < m_maxLevel; ++level) {
    const int size = m_widths[level] * m_heights[level];
    
    m_masks[level].resize(size);
    for (int y = 0; y < m_heights[level]; ++y) {
      const int ys[2] = {2 * y, min(m_heights[level - 1] - 1, 2 * y + 1)};
      for (int x = 0; x < m_widths[level]; ++x) {
        const int xs[2] = {2 * x, min(m_widths[level - 1] - 1, 2 * x + 1)};
        int in = 0;	int out = 0;
	
        for (int j = 0; j < 2; ++j) {
          for (int i = 0; i < 2; ++i) {
            const int index = ys[j] * m_widths[level - 1] + xs[i];
            if (m_masks[level - 1][index])
              in++;
            else
              out++;
          }
        }
        
        const int index = y * m_widths[level] + x;
        //if (out <= in)
        if (0 < in)
          m_masks[level][index] = (unsigned char)255;
        else
          m_masks[level][index] = (unsigned char)0;
      }
    }
  }
}

void Cimage::buildEdge(void) {
  //----------------------------------------------------------------------
  // edge
  for (int level = 1; level < m_maxLevel; ++level) {
    const int size = m_widths[level] * m_heights[level];
    
    m_edges[level].resize(size);
    for (int y = 0; y < m_heights[level]; ++y) {
      const int ys[2] = {2 * y, min(m_heights[level - 1] - 1, 2 * y + 1)};
      for (int x = 0; x < m_widths[level]; ++x) {
        const int xs[2] = {2 * x, min(m_widths[level - 1] - 1, 2 * x + 1)};
        int in = 0;	int out = 0;
	
        for (int j = 0; j < 2; ++j) {
          for (int i = 0; i < 2; ++i) {
            const int index = ys[j] * m_widths[level - 1] + xs[i];
            if (m_edges[level - 1][index])
              in++;
            else
              out++;
          }
        }
        
        const int index = y * m_widths[level] + x;
        //if (out <= in)
        if (0 < in)
          m_edges[level][index] = (unsigned char)255;
        else
          m_edges[level][index] = (unsigned char)0;
      }
    }
  }
}

void Cimage::setEdge(const float threshold) {
  const int size = m_widths[0] * m_heights[0];
  m_edges[0].resize(size);
  for (int i = 0; i < size; ++i)
    m_edges[0][i] = (unsigned char)0;

  vector<vector<float> > vvitmp, vvitmp2;
  vvitmp.resize(m_heights[0]);
  vvitmp2.resize(m_heights[0]);
  for (int y = 0; y < m_heights[0]; ++y) {
    vvitmp[y].resize(m_widths[0]);
    vvitmp2[y].resize(m_widths[0]);
    for (int x = 0; x < m_widths[0]; ++x) {
      vvitmp[y][x] = 0;
      vvitmp2[y][x] = 0;
    }
  }
  
  for (int y = 1; y < m_heights[0] - 1; ++y)
    for (int x = 1; x < m_widths[0] - 1; ++x) {
      const int index = 3 * (y * m_widths[0] + x);
      const int rindex = index + 3;
      const int lindex = index - 3;
      const int tindex = index - 3 * m_widths[0];
      const int bindex = index + 3 * m_widths[0];
      for (int i = 0; i < 3; ++i) {
        const int itmp0 = abs(m_images[0][rindex + i] -
                              m_images[0][lindex + i]);
        vvitmp[y][x] += itmp0 * itmp0;
        const int itmp1 = abs(m_images[0][bindex + i] -
                              m_images[0][tindex + i]);
        vvitmp[y][x] += itmp1 * itmp1;
      }
    }

  const float sigma = 3.0f;
  const float sigma2 = 2.0f * sigma * sigma;
  const int margin = (int)floor(2 * sigma);
  vector<float> filter;
  filter.resize(2 * margin + 1);
  for (int i = -margin; i <= margin; ++i)
    filter[i + margin] = exp(- i * i / sigma2);
  
  filterG(filter, vvitmp, vvitmp2);

  const float newThreshold = threshold * threshold * (2 * margin + 1) * (2 * margin + 1) / 3.0f;
  int count = -1;

  /*
  static int scount = -1;
  scount++;
  char buffer[1024];
  sprintf(buffer, "%04d.pgm", scount);
  ofstream ofstr;
  ofstr.open(buffer);
  ofstr << "P2" << endl
        << m_widths[0] << ' ' << m_heights[0] << endl
        << 255 << endl;
  */
  
  for (int y = 0; y < m_heights[0]; ++y) {
    for (int x = 0; x < m_widths[0]; ++x) {
      count++;
      if (newThreshold < vvitmp[y][x])
        m_edges[0][count] = (unsigned char)255;
      else
        m_edges[0][count] = (unsigned char)0;

      //ofstr << (int)m_edges[0][count] << ' ';
    }
  }
  //ofstr.close();
  buildEdge();
}

int Cimage::readPBMImage(const std::string file,
                         std::vector<unsigned char>& image,
                         int& width, int& height, const int fast) {
  if (file.substr(file.length() - 3, file.length()) != "pbm")
    return 0; 
  
  ifstream ifstr;
  ifstr.open(file.c_str());
  if (!ifstr.is_open()) {
    return 0;
    //exit (1);
  }
  string header;  unsigned char uctmp;

  ifstr >> header;
  ifstr.read((char*)&uctmp, sizeof(unsigned char));

  if (header != "P4") {
    cerr << "Only accept binary pbm format: " << file << endl;
    return 0;
  }
  
  while (1) {
    ifstr.read((char*)&uctmp, sizeof(unsigned char));
    ifstr.putback(uctmp);
    if (uctmp == '#') {
      char buffer[1024];
      ifstr.getline(buffer, 1024);
    }
    else
      break;
  }
  ifstr >> width >> height;
  ifstr.read((char*)&uctmp, sizeof(unsigned char));

  image.clear();
  if (fast) {
    ifstr.close();
    return 1;
  }
  int bcount = width * height;
  if (bcount % 8 != 0)
    bcount++;

  int count = 0;
  for (int i = 0; i < bcount; ++i) {
    ifstr.read((char*)&uctmp, sizeof(unsigned char));
    for (int j = 0; j < 8; ++j) {
      if (uctmp >> 7)
        image.push_back((unsigned char)0);
      else
        image.push_back((unsigned char)255);
      count++;
      uctmp <<= 1;
      if (count == width * height)
        break;
    }
  }
  ifstr.close();
  
  return 1;
}

int Cimage::writePBMImage(const std::string file,
                          std::vector<unsigned char>& image,
                          int& width, int& height, const int fast) {
  ofstream ofstr;
  ofstr.open(file.c_str());
  if (!ofstr.is_open()) {
    cerr << "Cannot write to a file: " << file << endl;
    return 0;
  }
  
  ofstr << "P4" << endl
        << width << ' ' << height << endl;

  unsigned char uctmp = 0;
  for (int i = 0; i < width * height; ++i) {
    uctmp <<= 1;
    if (image[i] < 127)
      uctmp |= 0x0001;
    else
      uctmp &= 0x0001;

    if (i % 8 == 7)
      ofstr.write((char*)&uctmp, sizeof(char));
  }
  const int itmp = (width * height) % 8;
  if (itmp != 0) {
    uctmp <<= 8 - itmp;
    ofstr.write((char*)&uctmp, sizeof(char));
  }
  
  ofstr.close();
  return 1;
}

int Cimage::writePGMImage(const std::string file,
                          const std::vector<unsigned char>& image,
                          const int width, const int height) {
  ofstream ofstr;
  ofstr.open(file.c_str());
  if (!ofstr.is_open()) {
    cerr << "Cannot write to a file: " << file << endl;
    return 0;
  }

  ofstr << "P5" << endl
        << width << ' ' << height << endl
        << 255 << endl;

  for (int i = 0; i < width * height; ++i) {
    unsigned char uctmp = image[i];
    ofstr.write((char*)&uctmp, sizeof(unsigned char));
  }

  ofstr.close();
  return 1;
}

int Cimage::readPGMImage(const std::string file,
			 std::vector<unsigned char>& image,
			 int& width, int& height, const int fast) {
  if (file.substr(file.length() - 3, file.length()) != "pgm")
    return 0;
  
  ifstream ifstr;
  ifstr.open(file.c_str());
  if (!ifstr.is_open()) {
    //cerr << "Cannot open a file: " << file << endl;
    return 0;
    //exit (1);
  }
  string header;  unsigned char uctmp;  int itmp;
  
  ifstr >> header;
  ifstr.read((char*)&uctmp, sizeof(unsigned char));  
  if (header != "P5") {
    cerr << "Only accept binary pgm format: " << file << ' ' << header << endl;
    return 0;
  }
  
  while (1) {
    ifstr.read((char*)&uctmp, sizeof(unsigned char));
    ifstr.putback(uctmp);
    if (uctmp == '#') {
      char buffer[1024];
      ifstr.getline(buffer, 1024);
    }
    else
      break;
  }
  ifstr >> width >> height >> itmp;
  ifstr.read((char*)&uctmp, sizeof(unsigned char));
  
  image.clear();
  if (fast) {
    ifstr.close();
    return 1;
  }
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      ifstr.read((char*)&uctmp, sizeof(unsigned char));
      image.push_back(uctmp);
    }
  }
  ifstr.close();
  
  return 1;
}

int Cimage::readPPMImage(const std::string file,
			 std::vector<unsigned char>& image,
			 int& width, int& height, const int fast) {
  if (file.substr(file.length() - 3, file.length()) != "ppm")
    return 0;
  
  ifstream ifstr;
  ifstr.open(file.c_str());
  if (!ifstr.is_open()) {
    //cerr << "Cannot open a file: " << file << endl;
    return 0;
    //exit (1);
  }
  string header;  unsigned char uctmp;  int itmp;
  
  ifstr >> header;
  ifstr.read((char*)&uctmp, sizeof(unsigned char));  
  if (header != "P6") {
    cerr << "Only accept binary ppm format: " << file << endl;
    return 0;
  }
  
  while (1) {
    ifstr.read((char*)&uctmp, sizeof(unsigned char));
    ifstr.putback(uctmp);
    if (uctmp == '#') {
      char buffer[1024];
      ifstr.getline(buffer, 1024);
    }
    else
      break;
  }
  ifstr >> width >> height >> itmp;
  ifstr.read((char*)&uctmp, sizeof(unsigned char));
  
  image.clear();
  if (fast) {
    ifstr.close();
    return 1;
  }
  
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      for (int i = 0; i < 3; ++i) {
	ifstr.read((char*)&uctmp, sizeof(unsigned char));
	image.push_back(uctmp);
      }
    }
  }

  ifstr.close();
  return 1;
}

int Cimage::writePPMImage(const std::string file,
                          const std::vector<unsigned char>& image,
                          const int width, const int height) {
  ofstream ofstr;
  ofstr.open(file.c_str());
  if (!ofstr.is_open()) {
    cerr << "Cannot write to a file: " << file << endl;
    return 0;
  }

  ofstr << "P6" << endl
        << width << ' ' << height << endl
        << 255 << endl;

  for (int i = 0; i < 3 * width * height; ++i) {
    unsigned char uctmp = image[i];
    ofstr.write((char*)&uctmp, sizeof(unsigned char));
  }

  ofstr.close();
  return 1;
}

//----------------------------------------------------------------------
// Jpeg functions
//----------------------------------------------------------------------
struct my_error_mgr {
  struct jpeg_error_mgr pub;	/* "public" fields */
  
  jmp_buf setjmp_buffer;	/* for return to caller */
};
typedef struct my_error_mgr * my_error_ptr;

METHODDEF(void)
my_error_exit (j_common_ptr cinfo)
{
  /* cinfo->err really points to a my_error_mgr struct, so coerce pointer */
  my_error_ptr myerr = (my_error_ptr) cinfo->err;

  /* Always display the message. */
  /* We could postpone this until after returning, if we chose. */
  (*cinfo->err->output_message) (cinfo);

  /* Return control to the setjmp point */
  longjmp(myerr->setjmp_buffer, 1);
}

int Cimage::readJpegImage(const std::string file,
			  std::vector<unsigned char>& image,
			  int& width, int& height, const int fast) {
  if (file.substr(file.length() - 3, file.length()) != "jpg")
    return 0;
  
  //======================================================================
  
  struct jpeg_decompress_struct cinfo;
  struct my_error_mgr jerr;
  FILE * infile;		/* source file */
  JSAMPARRAY buffer;		/* Output row buffer */
  int row_stride;		/* physical row width in output buffer */

  if ((infile = fopen(file.c_str(), "rb")) == NULL) {
    fprintf(stderr, "can't open %s\n", file.c_str());
    return 0;
  }

  cinfo.err = jpeg_std_error(&jerr.pub);
  jerr.pub.error_exit = my_error_exit;

  //Establish the setjmp return context for my_error_exit to use. 
  if (setjmp(jerr.setjmp_buffer)) {
    jpeg_destroy_decompress(&cinfo);
    fclose(infile);
    return 0;
  }

  jpeg_create_decompress(&cinfo);
  jpeg_stdio_src(&cinfo, infile);
  (void) jpeg_read_header(&cinfo, TRUE);
  (void) jpeg_start_decompress(&cinfo);

  /* We may need to do some setup of our own at this point before reading
   * the data.  After jpeg_start_decompress() we have the correct scaled
   * output image dimensions available, as well as the output colormap
   * if we asked for color quantization.
   * In this example, we need to make an output work buffer of the right size.
   */ 
  /* JSAMPLEs per row in output buffer */
  row_stride = cinfo.output_width * cinfo.output_components;
  const int component = cinfo.output_components;

  buffer = (*cinfo.mem->alloc_sarray)
    ((j_common_ptr) &cinfo, JPOOL_IMAGE, row_stride, 1);

  width = cinfo.output_width;
  height = cinfo.output_height;

  if (fast) {
    image.clear();
    //(void) jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    fclose(infile);
    return 1;
  } 
  
  std::vector<unsigned char> imagetmp;
  imagetmp.resize(height * row_stride);
  while (cinfo.output_scanline < cinfo.output_height) {
    /* jpeg_read_scanlines expects an array of pointers to scanlines.
     * Here the array is only one element long, but you could ask for
     * more than one scanline at a time if that's more convenient.
     */
    (void) jpeg_read_scanlines(&cinfo, buffer, 1);

    for (int i = 0; i < row_stride; ++i)
      imagetmp[(cinfo.output_scanline - 1) * row_stride + i] = (unsigned char)buffer[0][i];
  }
  (void) jpeg_finish_decompress(&cinfo);
  jpeg_destroy_decompress(&cinfo);
  fclose(infile);

  int count = 0;
  image.clear();
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (component == 1) {
	for (int i = 0; i < 3; ++i)
	  image.push_back(imagetmp[count]);
	count++;
      }
      else if (component == 3) {
	for (int i = 0; i < 3; ++i)
	  image.push_back(imagetmp[count++]);
      }
      else {
	cerr << "Cannot handle this component. Component num is " << component << endl;
	exit (1);
      }
    }
  }
  
  return 1;
}

void Cimage::writeJpegImage(const std::string filename,
			    const std::vector<unsigned char>& buffer,
			    const int width, const int height, const int flip) {
  const int quality = 100;
											
  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;
  /* More stuff */
  FILE * outfile;		/* target file */
  JSAMPROW row_pointer[1];	/* pointer to JSAMPLE row[s] */
  int row_stride;		/* physical row width in image buffer */

  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);

  if ((outfile = fopen(filename.c_str(), "wb")) == NULL) {
    fprintf(stderr, "can't open %s\n", filename.c_str());
    exit(1);
  }
  jpeg_stdio_dest(&cinfo, outfile);

  cinfo.image_width = width;
  cinfo.image_height = height;
  cinfo.input_components = 3;
  cinfo.in_color_space = JCS_RGB;
  jpeg_set_defaults(&cinfo);  
  jpeg_set_quality(&cinfo, quality, TRUE /* limit to baseline-JPEG values */);

  jpeg_start_compress(&cinfo, TRUE);

  row_stride = width * 3;	/* JSAMPLEs per row in image_buffer */

  while (cinfo.next_scanline < cinfo.image_height) {
    if (flip)
      row_pointer[0] = (JSAMPROW)& buffer[(cinfo.image_height - 1 - cinfo.next_scanline) * row_stride];
    else
      row_pointer[0] = (JSAMPROW)& buffer[cinfo.next_scanline * row_stride];
    (void) jpeg_write_scanlines(&cinfo, row_pointer, 1);

  }
 
  jpeg_finish_compress(&cinfo);
  fclose(outfile);

  jpeg_destroy_compress(&cinfo);
}

void Cimage::rgb2hs(const Vec3f& rgb, float& h, float& s) {
  rgb2hs(rgb[0], rgb[1], rgb[2], h, s);
}

void Cimage::rgb2hs(const Vec3f& rgb, Vec2f& hs) {
  rgb2hs(rgb[0], rgb[1], rgb[2], hs[0], hs[1]);
}

void Cimage::rgb2hs(const float r, const float g, const float b, Vec2f& hs) {
  rgb2hs(r, g, b, hs[0], hs[1]);
}

void Cimage::rgb2hs(const float r, const float g, const float b,
		    float& h, float& s) {
  double max, min, del, rc, gc, bc;

  max = max3(r, g, b);
  min = min3(r, g, b);

  del = max - min;
  s = (max == 0.0) ? 0.0 : del / max;

  h = -1;					/* No hue */
  if (s != 0.0) {
    rc = (max - r) / del;
    gc = (max - g) / del;
    bc = (max - b) / del;

    if (r == max) h = bc - gc;
    else if (g == max) h = 2 + rc - bc;
    else /* if (b == max) */ h = 4 + gc - rc;

    h = h * 60;
    if (h < 0) h += 360;
  }
}

/* rgb2hsv -- convert RGB to HSV */
void Cimage::rgb2hsv(const float r, const float g, const float b,
		     float& h, float& s, float& v) {
  double max, min, del, rc, gc, bc;

  max = max3(r, g, b);
  min = min3(r, g, b);

  del = max - min;
  v = max;
  s = max == 0.0 ? 0.0 : del / max;

  //h = -1;					/* No hue */
  h = 0.0;
  if (s != 0.0) {
    rc = (max - r) / del;
    gc = (max - g) / del;
    bc = (max - b) / del;

    if (r == max) h = bc - gc;
    else if (g == max) h = 2 + rc - bc;
    else /* if (b == max) */ h = 4 + gc - rc;

    h = h * 60;
    if (h < 0) h += 360;
  }
}

void Cimage::rgb2hsv(const Vec3f& rgb, Vec3f& hsv) {
  rgb2hsv(rgb[0], rgb[1], rgb[2], hsv[0], hsv[1], hsv[2]);
}

void Cimage::rgb2hsv(const Vec3f& rgb, float& hr, float& sr, float& vr) {
  rgb2hsv(rgb[0], rgb[1], rgb[2], hr, sr, vr);
}

void Cimage::rgb2hsv(const float r, const float g, const float b, Vec3f& hsv) {
  rgb2hsv(r, g, b, hsv[0], hsv[1], hsv[2]);
}

float Cimage::hsdis(const float h0, const float s0, const float h1, const float s1) {
  // Represent by 2d vector each
  const float angle0 = h0 * M_PI / 180.0;
  Vec2f axis0(cos(angle0), sin(angle0));
  axis0 *= s0;

  const float angle1 = h1 * M_PI / 180.0;
  Vec2f axis1(cos(angle1), sin(angle1));
  axis1 *= s1;

  return norm(axis0 - axis1) / 2.0;
}

void Cimage::gray2rgb(const float gray, float& r, float& g, float& b) {
  if (gray < 0.5) {
    r = 0.0f;
    g = 2.0f * gray;
    b = 1.0f - g;
  }
  else {
    r = (gray - 0.5f) * 2.0f;
    g = 1.0f - r;
    b = 0.0f;
  }
}

// Some low-level image processing
// 2D convolution with twice 1D gaussian convolution.
void Cimage::filterG(const std::vector<float>& filter,
                     std::vector<std::vector<float> >& data) {
  vector<vector<float> > buffer;
  const int height = (int)data.size();
  const int width = (int)data[0].size();
  buffer.resize(height);
  for (int y = 0; y < height; ++y)
    buffer[y].resize(width);
  filterG(filter, data, buffer);
}

void Cimage::filterG(const std::vector<float>& filter,
                     const int width, const int height,
                     std::vector<float>& data) {
  vector<float> buffer;
  buffer.resize(width * height);
  filterG(filter, width, height, data, buffer);
}

void Cimage::filterG(const std::vector<float>& filter,
                     const int width, const int height,
                     std::vector<float>& data,
                     std::vector<float>& buffer) {
  if ((int)filter.size() % 2 == 0) {
    cerr << "Filter must have an odd length" << endl;
    exit (1);
  }
  const int margin = (int)filter.size() / 2;
  
  // vertical smooth
  int index = -1;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      ++index;
      buffer[index] = 0.0f;
      float denom = 0.0f;
      int index1 = index - (margin + 1) * width;
      for (int j = -margin; j <= margin; ++j) {
        index1 += width;
        const int ytmp = y + j;
        if (ytmp < 0 || height <= ytmp)
          continue;
        buffer[index] += filter[j + margin] * data[index1];
        denom += filter[j + margin];
      }
      buffer[index] /= denom;
    }
  }
  // horizontal smooth
  buffer.swap(data);
  index = -1;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      ++index;
      buffer[index] = 0.0f;
      float denom = 0.0f;
      int index1 = index - (margin + 1);
      for (int i = -margin; i <= margin; ++i) {
        ++index1;
        const int xtmp = x + i;
        if (xtmp < 0 || width <= xtmp)
          continue;
        buffer[index] += filter[i + margin] * data[index1];
        denom += filter[i + margin];
      }
      buffer[index] /= denom;
    }
  }
  buffer.swap(data);
}

void Cimage::filterG(const std::vector<float>& filter,
                     std::vector<std::vector<float> >& data,
                     std::vector<std::vector<float> >& buffer) {
  if ((int)filter.size() % 2 == 0) {
    cerr << "Filter must have an odd length" << endl;
    exit (1);
  }
  const int margin = (int)filter.size() / 2;
  const int height = (int)data.size();
  const int width = (int)data[0].size();
  
  // vertical smooth
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      buffer[y][x] = 0.0f;
      float denom = 0.0f;
      for (int j = -margin; j <= margin; ++j) {
        const int ytmp = y + j;
        if (ytmp < 0 || height <= ytmp)
          continue;
        buffer[y][x] += filter[j + margin] * data[ytmp][x];
        denom += filter[j + margin];
      }
      buffer[y][x] /= denom;
    }
  }
  // horizontal smooth
  buffer.swap(data);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      buffer[y][x] = 0.0f;
      float denom = 0.0f;
      for (int i = -margin; i <= margin; ++i) {
        const int xtmp = x + i;
        if (xtmp < 0 || width <= xtmp)
          continue;
        buffer[y][x] += filter[i + margin] * data[y][xtmp];
        denom += filter[i + margin];
      }
      buffer[y][x] /= denom;
    }
  }
  buffer.swap(data);
}

// non maximum surpression
void Cimage::nms(std::vector<std::vector<float> >& data) {
  vector<vector<float> > buffer;
  const int height = (int)data.size();
  const int width = (int)data[0].size();
  buffer.resize(height);
  for (int y = 0; y < height; ++y)
    buffer[y].resize(width);
  nms(data, buffer);
}

void Cimage::nms(std::vector<std::vector<float> >& data,
                 std::vector<std::vector<float> >& buffer) {
  const int height = (int)data.size();
  const int width = (int)data[0].size();
  // non-max surpression
  for (int y = 0; y < height; ++y)
    for (int x = 0; x < width; ++x) {
      if (x != 0 && data[y][x] < data[y][x - 1]) {
        buffer[y][x] = 0.0f;
        continue;
      }
      if (x != width - 1 && data[y][x] < data[y][x + 1]) {
        buffer[y][x] = 0.0f;
        continue;
      }
      if (y != 0 && data[y][x] < data[y - 1][x]) {
        buffer[y][x] = 0.0f;
        continue;
      }
      if (y != height - 1 && data[y][x] < data[y + 1][x]) {
        buffer[y][x] = 0.0f;
        continue;
      }
      buffer[y][x] = data[y][x];
    }
  buffer.swap(data);
}

void Cimage::createFilter(const float sigma, std::vector<float>& filter) {
  const float sigma2 = 2.0f * sigma * sigma;
  const int margin = (int)floor(2 * sigma);

  float sum = 0.0f;
  filter.resize(2 * margin + 1);
  for (int i = -margin; i <= margin; ++i) {
    filter[i + margin] = exp(- i * i / sigma2);
    sum += filter[i + margin];
  }
  for (int i = 0; i < 2 * margin + 1; ++i)
    filter[i] /= sum;
}

void Cimage::sift(const Vec3f& center,
                  const Vec3f& xaxis, const Vec3f& yaxis,
                  std::vector<float>& descriptor) const {
  const float step = (norm(xaxis) + norm(yaxis)) / 2.0f;
  const int level = max(0, min(m_maxLevel - 1,
                               (int)floor(log(step) / log(2.0f) + 0.5f)));

  if (level != 0) {
    const float scale = 0x0001 << level;
    sift(center / scale, xaxis / scale, yaxis / scale,
         level, descriptor);
  }
  else
    sift(center, xaxis, yaxis, 0, descriptor);
}
  
void Cimage::sift(const Vec3f& center,
                  const Vec3f& xaxis, const Vec3f& yaxis,
                  const int level, std::vector<float>& descriptor) const {
  /*
  Mat2f A;
  A[0][0] = xaxis[0];  A[1][0] = xaxis[1];
  A[0][1] = yaxis[0];  A[1][1] = yaxis[1];
  Mat2f IA;
  invert(IA, A);
  */
  // bin descritization (pbin x pbin x abin)
  const int pbin = 4;
  const int abin = 8;
  const float aunit = 2 * M_PI / abin;
  // pixels in each bin
  const int pnum = 4;
  // max value
  const float maxValue = 0.2f;

  descriptor.clear();
  
  const int width = pbin * pnum;
  const int width2 = width / 2;

  // Bounding box check
  const float width205 = width2 - 0.5f;
  const Vec3f topleft = center - width205 * yaxis - width205 * xaxis;
  const Vec3f topright = center - width205 * yaxis + width205 * xaxis;
  const Vec3f bottomleft = center + width205 * yaxis - width205 * xaxis;
  const Vec3f bottomright = center + width205 * yaxis + width205 * xaxis;

  const float minx = min(min(topleft[0], topright[0]),
                         min(bottomleft[0], bottomright[0]));
  const float maxx = max(max(topleft[0], topright[0]),
                         max(bottomleft[0], bottomright[0]));
  const float miny = min(min(topleft[1], topright[1]),
                         min(bottomleft[1], bottomright[1]));
  const float maxy = max(max(topleft[1], topright[1]),
                         max(bottomleft[1], bottomright[1]));
  
  if (minx < 0.0 || getWidth(level) - 1 <= maxx ||
      miny < 0.0 || getHeight(level) - 1 <= maxy)
    return;

  descriptor.resize(pbin * pbin * abin, 0.0f);
  
  const float sigma2 = 2 * width2 * width2;  
  
  for (int y = 0; y < width; ++y) {
    Vec3f start = topleft + y * yaxis;
    const int ybin = y / pnum;
    // deviation from center
    const float fy = y - width2 + 0.5f;
    
    for (int x = 0; x < width; ++x) {
      const int xbin = x / pnum;
      
      const Vec3f px = start - xaxis;   const Vec3f nx = start + xaxis;
      const Vec3f py = start - yaxis;   const Vec3f ny = start + yaxis;

      const float dx = getColor(nx[0], nx[1], level).sum() -
        getColor(px[0], px[1], level).sum();
      const float dy = getColor(ny[0], ny[1], level).sum() -
        getColor(py[0], py[1], level).sum();
      
      float angle = atan2(dx, dy);
      if (angle < 0.0)
        angle += 2 * M_PI;
      const float af = angle / aunit;   const int lf = (int)floor(af);
      const float hfweight = af - lf;   const int hf = (lf + 1) % abin;
      const float lfweight = 1.0f - hfweight;

      // deviation from center
      const float fx = x - width2 + 0.5f;
      const float weight = sqrt(dx * dx + dy * dy) *
        exp(- (fx * fx + fy * fy) / sigma2);
      
      const int offset = (ybin * pbin + xbin) * abin;
      descriptor[offset + lf] += lfweight * weight;
      descriptor[offset + hf] += hfweight * weight;
      
      start += xaxis;
    }
  }

  // Normalize, set max to 0.2
  float ss = 0.0f;
  for (int i = 0; i < (int)descriptor.size(); ++i)
    ss += descriptor[i] * descriptor[i];
  ss = sqrt(ss);

  if (ss == 0.0f)
    return;

  int change = 0;
  for (int i = 0; i < (int)descriptor.size(); ++i) {
    descriptor[i] /= ss;
    if (maxValue < descriptor[i]) {
      descriptor[i] = maxValue;
      change = 1;
    }
  }

  if (change) {
    ss = 0.0f;
    for (int i = 0; i < (int)descriptor.size(); ++i)
      ss += descriptor[i] * descriptor[i];
    ss = sqrt(ss);

    for (int i = 0; i < (int)descriptor.size(); ++i)
      descriptor[i] /= ss;
  }
}


/*
// If you want to use, use all the entries altogether instead of independently

// Used to filter out outliers. Very general algorithm.
// Use standard mean and deviation
void Cimage::setInOut(const std::vector<vector<float> >& data, std::vector<int>& inout,
		      const float sigma, const int specular) {
  const int size = (int)data.size();
  if (size == 0)
    return;
  
  inout.resize(size);
  fill(inout.begin(), inout.end(), 0);
  
  const int csize = (int)data[0].size();
  if (csize == 0)
    return;

  // For each channel.
  for (int c = 0; c < csize; ++c) {
    float ave = 0.0;    float ave2 = 0.0;
    for (int i = 0; i < size; ++i) {
      ave += data[i][c];
      ave2 += data[i][c] * data[i][c];
    }
    ave /= size;    ave2 /= size;
    ave2 = sqrt(max(0.0f, ave2 - ave * ave));

    const float mint = ave - sigma * ave2;
    const float maxt = ave + sigma * ave2;

    for (int i = 0; i < size; ++i) {
      if (specular == 0) {
	if (data[i][c] < mint || maxt < data[i][c])
	  inout[i]++;
      }
      else {
	if (maxt < data[i][c])
	  inout[i]++;
      }
    }
  }
}

// Used to filter out outliers. Very general algorithm.
// Use mean and deviation
void Cimage::setInOut(const std::vector<Vec3f>& data, std::vector<int>& inout,
		      const float sigma, const int specular) {
  const int size = (int)data.size();
  if (size == 0)
    return;
  
  inout.resize(size);
  fill(inout.begin(), inout.end(), 0);
  
  for (int c = 0; c < 3; ++c) {
    float ave = 0.0;    float ave2 = 0.0;
    for (int i = 0; i < size; ++i) {
      ave += data[i][c];
      ave2 += data[i][c] * data[i][c];
    }
    ave /= size;    ave2 /= size;
    ave2 = sqrt(max(0.0f, ave2 - ave * ave));
    
    const float mint = ave - sigma * ave2;
    const float maxt = ave + sigma * ave2;

    for (int i = 0; i < size; ++i) {
      if (specular == 0) {
	if (data[i][c] < mint || maxt < data[i][c])
	  inout[i]++;
      }
      else {
	if (maxt < data[i][c])
	  inout[i]++;
      }
    }
  } 
}

// Used to filter out outliers. HSV version for specular highlights.
void Cimage::setInOutHSV(const std::vector<Vec3f>& hsvs, std::vector<int>& inout,
			 const float sigma, const int specular) {
  const int size = (int)hsvs.size();
  if (size == 0)
    return;
  
  inout.resize(size);
  fill(inout.begin(), inout.end(), 0);

  // Because we cannot just taken an average for hue, use the angle method
  {
    // First find the reference index
    vector<float> sum;    sum.resize(size);
    fill(sum.begin(), sum.end(), 0.0);
    
    for (int i = 0; i < size; ++i)
      for (int j = i+1; j < size; ++j) {
	const float ftmp = hdis(hsvs[i][0], hsvs[j][0]);
	sum[i] += ftmp;
	sum[j] += ftmp;
      }
    const int refindex = (int)(min_element(sum.begin(), sum.end()) - sum.begin());
    float ave = 0.0;    float ave2 = 0.0;
    const float refvalue = hsvs[refindex][0];
    for (int i = 0; i < size; ++i) {
      float ftmp = hsvs[i][0] - refvalue;
      if (180.0 < ftmp)
	ftmp -= 360.0;
      else if (ftmp < -180.0)
	ftmp += 360.0;

      ave += ftmp;
      ave2 += ftmp * ftmp;
    }
    ave /= size;    ave2 /= size;
    ave2 = sqrt(max(0.0f, ave2 - ave * ave));

    const float mint = ave - sigma * ave2;
    const float maxt = ave + sigma * ave2;
    
    for (int i = 0; i < size; ++i)
      if (hsvs[i][0] < mint || maxt < hsvs[i][0])
	inout[i]++;
  }

  // For saturation and intensity
  for (int c = 1; c < 3; ++c) {
    float ave = 0.0;    float ave2 = 0.0;
    for (int i = 0; i < size; ++i) {
      ave += hsvs[i][c];
      ave2 += hsvs[i][c] * hsvs[i][c];
    }
    ave /= size;    ave2 /= size;
    ave2 = sqrt(max(0.0f, ave2 - ave * ave));
    
    const float mint = ave - sigma * ave2;
    const float maxt = ave + sigma * ave2;

    if (c == 2 && specular) {
      for (int i = 0; i < size; ++i)
	if (maxt < hsvs[i][c])
	  inout[i]++;
    }
    else {
      for (int i = 0; i < size; ++i)
	if (hsvs[i][c] < mint || maxt < hsvs[i][c])
	  inout[i]++;
    }
  }
}
*/
