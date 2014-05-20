#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cmath>
#include "option.h"
#include <algorithm>

using namespace std;
using namespace PMVS3;

Soption::Soption(void) {
  m_level = 1;          m_csize = 2;
  m_threshold = 0.7;    m_wsize = 7;
  m_minImageNum = 3;    m_CPU = 4;
  m_setEdge = 0.0f;     m_useBound = 0;
  m_useVisData = 0;     m_sequence = -1;
  m_tflag = -10;
  m_oflag = -10;

  // Max angle must be at least this big
  m_maxAngleThreshold = 10.0f * M_PI / 180.0f;
  // The smaller the tighter
  m_quadThreshold = 2.5f;
}  

void Soption::init(const std::string prefix, const std::string option) {
  m_prefix = prefix;
  m_option = option;
  std::ifstream ifstr;
  string optionfile = prefix + option;
  ifstr.open(optionfile.c_str());
  while (1) {
    string name;
    ifstr >> name;
    if (ifstr.eof())
      break;
    if (name[0] == '#') {
      char buffer[1024];
      ifstr.putback('#');
      ifstr.getline(buffer, 1024);
      continue;
    }
    if (name == "level")             ifstr >> m_level;
    else if (name == "csize")        ifstr >> m_csize;
    else if (name == "threshold")    ifstr >> m_threshold;
    else if (name == "wsize")        ifstr >> m_wsize;
    else if (name == "minImageNum")  ifstr >> m_minImageNum;
    else if (name == "CPU")          ifstr >> m_CPU;
    else if (name == "setEdge")      ifstr >> m_setEdge;
    else if (name == "useBound")     ifstr >> m_useBound;
    else if (name == "useVisData")   ifstr >> m_useVisData;
    else if (name == "sequence")     ifstr >> m_sequence;
    else if (name == "timages") {
      ifstr >> m_tflag;
      if (m_tflag == -1) {
        int firstimage, lastimage;
        ifstr >> firstimage >> lastimage;
        for (int i = firstimage; i < lastimage; ++i)
          m_timages.push_back(i);
      }
      else if (0 < m_tflag) {
        for (int i = 0; i < m_tflag; ++i) {
          int itmp;          ifstr >> itmp;
          m_timages.push_back(itmp);
        }
      }
      else {
        cerr << "tflag is not valid: " << m_tflag << endl;   exit (1);
      }
    }
    else if (name == "oimages") {
      ifstr >> m_oflag;
      if (m_oflag == -1) {
        int firstimage, lastimage;
        ifstr >> firstimage >> lastimage;
        for (int i = firstimage; i < lastimage; ++i)
          m_oimages.push_back(i);
      }
      else if (0 <= m_oflag) {
        for (int i = 0; i < m_oflag; ++i) {
          int itmp;          ifstr >> itmp;
          m_oimages.push_back(itmp);
        }
      }
      else if (m_oflag != -2 && m_oflag != -3) {
        cerr << "oflag is not valid: " << m_oflag << endl;   exit (1);
      }
    }
    else if (name == "quad")      ifstr >> m_quadThreshold;
    else if (name == "maxAngle") {
      ifstr >> m_maxAngleThreshold;
      m_maxAngleThreshold *= M_PI / 180.0f;
    }
    else {
      cerr << "Unrecognizable option: " << name << endl;   exit (1);
    }
  }
  ifstr.close();

  if (m_tflag == -10 || m_oflag == -10) {
    cerr << "m_tflag and m_oflag not specified: "
         << m_tflag << ' ' << m_oflag << endl;
    exit (1);
  }
  
  //----------------------------------------------------------------------
  string sbimages = prefix + string("bimages.dat");

  for (int i = 0; i < (int)m_timages.size(); ++i)
    m_dict[m_timages[i]] = i;

  initOimages();
  initVisdata();
  
  if (m_useBound)
    initBindexes(sbimages);

  cerr << "--------------------------------------------------" << endl  
       << "--- Summary of specified options ---" << endl;
  cerr << "# of timages: " << (int)m_timages.size();
  if (m_tflag == -1)
    cerr << " (range specification)" << endl;
  else
    cerr << " (enumeration)" << endl;
  cerr << "# of oimages: " << (int)m_oimages.size();
  if (m_oflag == -1)
    cerr << " (range specification)" << endl;
  else if (0 <= m_oflag)
    cerr << " (enumeration)" << endl;
  else if (m_oflag == -2)
    cerr << " (vis.dat is used)" << endl;
  else if (m_oflag == -3)
    cerr << " (not used)" << endl;

  cerr << "level: " << m_level << "  csize: " << m_csize << endl
       << "threshold: " << m_threshold << "  wsize: " << m_wsize << endl
       << "minImageNum: " << m_minImageNum << "  CPU: " << m_CPU << endl
       << "useVisData: " << m_useVisData << "  sequence: " << m_sequence << endl;
  cerr << "--------------------------------------------------" << endl;
}

void Soption::initOimages(void) {
  if (m_oflag != -2)
    return;

  string svisdata = m_prefix + string("vis.dat");
  ifstream ifstr;
  ifstr.open(svisdata.c_str());
  if (!ifstr.is_open()) {
    cerr << "No vis.dat although specified to initOimages: " << endl
         << svisdata << endl;
    exit (1);
  }
  
  string header;  int num2;
  ifstr >> header >> num2;

  m_oimages.clear();
  for (int c = 0; c < num2; ++c) {
    int index0;
    map<int, int>::iterator ite0 = m_dict.find(c);
    if (ite0 == m_dict.end())
      index0 = -1;
    else
      index0 = ite0->second;
    int itmp;
    ifstr >> itmp >> itmp;
    for (int i = 0; i < itmp; ++i) {
      int itmp2;
      ifstr >> itmp2;
      if (index0 != -1 && m_dict.find(itmp2) == m_dict.end())
        m_oimages.push_back(itmp2);
    }
  }
  ifstr.close();
  
  sort(m_oimages.begin(), m_oimages.end());
  m_oimages.erase(unique(m_oimages.begin(), m_oimages.end()), m_oimages.end());
}

// When do not use vis.dat
void Soption::initVisdata(void) {
  // Case classifications. Set m_visdata by using vis.dat or not.
  if (m_useVisData == 0) {
    const int tnum = (int)m_timages.size();
    const int onum = (int)m_oimages.size();
    const int num = tnum + onum;
    m_visdata.resize(num);
    m_visdata2.resize(num);
    for (int y = 0; y < num; ++y) {
      m_visdata[y].resize(num);
      for (int x = 0; x < num; ++x)
        if (x == y)
          m_visdata[y][x] = 0;
        else {
          m_visdata[y][x] = 1;
          m_visdata2[y].push_back(x);
        }
    }
  }
  else
    initVisdata2();
}

// Given m_timages and m_oimages, set m_visdata, m_visdata2
void Soption::initVisdata2(void) {
  string svisdata = m_prefix + string("vis.dat");
  
  vector<int> images;
  images.insert(images.end(), m_timages.begin(), m_timages.end());
  images.insert(images.end(), m_oimages.begin(), m_oimages.end());
  map<int, int> dict2;
  for (int i = 0; i < (int)images.size(); ++i)
    dict2[images[i]] = i;

  ifstream ifstr;
  ifstr.open(svisdata.c_str());
  if (!ifstr.is_open()) {
    cerr << "No vis.dat although specified to initVisdata2: " << endl
         << svisdata << endl;
    exit (1);
  }
  
  string header;  int num2;
  ifstr >> header >> num2;

  m_visdata2.resize((int)images.size());
  for (int c = 0; c < num2; ++c) {
    int index0;
    map<int, int>::iterator ite0 = dict2.find(c);
    if (ite0 == dict2.end())
      index0 = -1;
    else
      index0 = ite0->second;
    int itmp;
    ifstr >> itmp >> itmp;
    for (int i = 0; i < itmp; ++i) {
      int itmp2;
      ifstr >> itmp2;
      int index1;
      map<int, int>::iterator ite1 = dict2.find(itmp2);
      if (ite1 == dict2.end())
        index1 = -1;
      else
        index1 = ite1->second;

      if (index0 != -1 && index1 != -1)
        m_visdata2[index0].push_back(index1);
    }
  }
  ifstr.close();

  const int num = (int)images.size();
  m_visdata.clear();  
  m_visdata.resize(num);
  for (int y = 0; y < num; ++y) {
    m_visdata[y].resize(num);
    fill(m_visdata[y].begin(), m_visdata[y].end(), 0);
    for (int x = 0; x < (int)m_visdata2[y].size(); ++x)
      m_visdata[y][m_visdata2[y][x]] = 1;
  }

  // check symmetry
  for (int i = 0; i < (int)m_visdata.size(); ++i) {
    for (int j = i+1; j < (int)m_visdata.size(); ++j) {
      if (m_visdata[i][j] != m_visdata[j][i]) {
        m_visdata[i][j] = m_visdata[j][i] = 1;
      }
    }
  }
}
                                             
void Soption::initBindexes(const std::string sbimages) {
  if (sbimages.empty())
    return;
  
  m_bindexes.clear();
  ifstream ifstr;
  ifstr.open(sbimages.c_str());
  if (!ifstr.is_open()) {
    cerr << "File not found: " << sbimages << endl;
    exit (1);
  }
  
  cerr << "Reading bimages" << endl;
  int itmp;
  ifstr >> itmp;
  for (int i = 0; i < itmp; ++i) {
    int itmp0;
    ifstr >> itmp0;

    if (m_dict.find(itmp0) != m_dict.end())
      m_bindexes.push_back(m_dict[itmp0]);
  }
  ifstr.close();
}
