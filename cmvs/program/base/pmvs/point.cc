#include "point.h"

using namespace PMVS3;
using namespace std;

Cpoint::Cpoint(void) {
  m_response = -1.0;
  m_type = -1;
}

Cpoint::~Cpoint() {
}

std::istream& PMVS3::operator >>(std::istream& istr, Cpoint& rhs) {
  string header;
  istr >> header;
  istr >> rhs.m_icoord[0] >> rhs.m_icoord[1] >> rhs.m_response >> rhs.m_type;
  rhs.m_icoord[2] = 1.0f;
  return istr;
}

std::ostream& PMVS3::operator <<(std::ostream& ostr, const Cpoint& rhs) {
  ostr << "POINT0" << endl
       << rhs.m_icoord[0] << ' ' << rhs.m_icoord[1] << ' ' << rhs.m_response << ' '
       << rhs.m_type;
  return ostr;
}
