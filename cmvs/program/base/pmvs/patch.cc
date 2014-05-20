#include <string>
#include "../numeric/vec4.h"
#include "patch.h"

using namespace std;

std::istream& Patch::operator >>(std::istream& istr, Cpatch& rhs) {
  string header;
  int itmp;
  istr >> header >> rhs.m_coord >> rhs.m_normal >> rhs.m_ncc
       >> rhs.m_dscale >> rhs.m_ascale;

  if (header == "PATCHA") {
    int type;    Vec4f dir;
    istr >> type >> dir;
  }
  
  istr >> itmp;
  rhs.m_images.resize(itmp);
  for (int i = 0; i < itmp; ++i)
    istr >> rhs.m_images[i];

  istr >> itmp;
  rhs.m_vimages.resize(itmp);
  for (int i = 0; i < itmp; ++i)
    istr >> rhs.m_vimages[i];

  return istr;
}

std::ostream& Patch::operator <<(std::ostream& ostr, const Cpatch& rhs) {
  ostr << "PATCHS" << endl
       << rhs.m_coord << endl
       << rhs.m_normal << endl
       << rhs.m_ncc << ' '
       << rhs.m_dscale << ' '
       << rhs.m_ascale << endl
       << (int)rhs.m_images.size() << endl;
  for (int i = 0; i < (int)rhs.m_images.size(); ++i)
    ostr << rhs.m_images[i] << ' ';
  ostr << endl;
  
  ostr << (int)rhs.m_vimages.size() << endl;
  for (int i = 0; i < (int)rhs.m_vimages.size(); ++i)
    ostr << rhs.m_vimages[i] << ' ';
  ostr << endl;

  return ostr;
}
