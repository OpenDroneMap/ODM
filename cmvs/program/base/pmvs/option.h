#ifndef PMVS3_OPTION_H
#define PMVS3_OPTION_H

#include <string>
#include <vector>
#include <map>

namespace PMVS3 {
  
struct Soption{
  public:
  int m_level;
  int m_csize;
  float m_threshold;
  int m_wsize;
  int m_minImageNum;
  int m_CPU;
  float m_setEdge;
  int m_useBound;
  int m_useVisData;
  int m_sequence;
  
  float m_maxAngleThreshold;
  float m_quadThreshold;
  
  std::string m_prefix;
  std::string m_option;
  int m_tflag;
  std::vector<int> m_timages;
  int m_oflag;
  std::vector<int> m_oimages;

  std::map<int, int> m_dict;
  
  std::vector<int> m_bindexes;
  std::vector<std::vector<int> > m_visdata;
  std::vector<std::vector<int> > m_visdata2;
  
  Soption(void);
  
  void init(const std::string prefix, const std::string option);
  
  protected:
  void initOimages(void);
  void initBindexes(const std::string sbimages);
  void initVisdata(void);
  void initVisdata2(void);
};
};

#endif // PMVS3_OPTION_H
