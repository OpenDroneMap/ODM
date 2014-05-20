#ifndef NUMERIC_VEC4_H
#define NUMERIC_VEC4_H

#include <cmath>
#include "vec3.h"

template<class T>
class TVec4 {
 private:
  T m_elt[4];
  
 public:
  // Standard constructors
  //
  TVec4(T s=0) { *this = s; }
  TVec4(T x, T y, T z, T w) { m_elt[0]=x; m_elt[1]=y; m_elt[2]=z; m_elt[3]=w; }
  
  // Copy constructors & assignment operators
  template<class U> TVec4(const TVec4<U>& v) { *this = v; }
  template<class U> TVec4(const TVec3<U>& v,T w)
    { m_elt[0]=v[0];  m_elt[1]=v[1];  m_elt[2]=v[2];  m_elt[3]=w; }
  template<class U> TVec4(const U v[4])
    { m_elt[0]=v[0]; m_elt[1]=v[1]; m_elt[2]=v[2]; m_elt[3]=v[3]; }
  template<class U> TVec4& operator=(const TVec4<U>& v)
  { m_elt[0]=v[0];  m_elt[1]=v[1];  m_elt[2]=v[2]; m_elt[3]=v[3]; return *this; }
  TVec4& operator=(T s) { m_elt[0]=m_elt[1]=m_elt[2]=m_elt[3]=s; return *this; }
  
  
  // Descriptive interface
  //
  typedef T value_type;
  static int size() { return 4; }

  // Access methods
  //
  operator       T*()       { return m_elt; }
  operator const T*() const { return m_elt; }
  
  T& operator[](int i)       { return m_elt[i]; }
  T  operator[](int i) const { return m_elt[i]; }
  operator const T*()       { return m_elt; }
  
  // Assignment and in-place arithmetic methods
  //
  inline TVec4& operator+=(const TVec4& v);
  inline TVec4& operator-=(const TVec4& v);
  inline TVec4& operator*=(T s);
  inline TVec4& operator/=(T s);
  
  inline bool operator==(const TVec4& v) const;
  inline bool operator!=(const TVec4& v) const;
  
  inline T norm2(void) const{
    return (*this) * (*this);
  }
  inline T norm(void) const{
    return sqrt(norm2());
  }
  inline void unitize(void) {
    const T denom2 = norm2();
    
    if(denom2 != 1.0 && denom2 != 0.0 ) {
      const T denom = sqrt(denom2);
      m_elt[0] /= denom;      m_elt[1] /= denom;
      m_elt[2] /= denom;      m_elt[3] /= denom;
    }
  }      
};

////////////////////////////////////////////////////////////////////////
//
// Method definitions
//
template<class T> inline TVec4<T>& TVec4<T>::operator+=(const TVec4<T>& v)
{ m_elt[0]+=v[0];  m_elt[1]+=v[1];  m_elt[2]+=v[2];  m_elt[3]+=v[3]; return *this;};

template<class T> inline TVec4<T>& TVec4<T>::operator-=(const TVec4<T>& v)
{ m_elt[0]-=v[0];  m_elt[1]-=v[1];  m_elt[2]-=v[2];  m_elt[3]-=v[3]; return *this;};

template<class T> inline TVec4<T>& TVec4<T>::operator*=(T s)
{ m_elt[0] *= s;   m_elt[1] *= s;   m_elt[2] *= s;  m_elt[3] *= s; return *this; };

template<class T> inline TVec4<T>& TVec4<T>::operator/=(T s)
{ m_elt[0] /= s;   m_elt[1] /= s;   m_elt[2] /= s;  m_elt[3] /= s; return *this; };

template<class T> inline bool TVec4<T>::operator==(const TVec4<T>& v) const{
    if (m_elt[0] == v.m_elt[0] && m_elt[1] == v.m_elt[1] &&
	m_elt[2] == v.m_elt[2] && m_elt[3] == v.m_elt[3])
	return true;
    else
	return false;
};

template<class T> inline bool TVec4<T>::operator!=(const TVec4<T>& v) const{
    return !(*this == v);
};

////////////////////////////////////////////////////////////////////////
//
// Operator definitions
//

template<class T>
inline TVec4<T> operator+(const TVec4<T> &u, const TVec4<T> &v)
{ return TVec4<T>(u[0]+v[0], u[1]+v[1], u[2]+v[2], u[3]+v[3]); };

template<class T>
inline TVec4<T> operator-(const TVec4<T> &u, const TVec4<T>& v)
{ return TVec4<T>(u[0]-v[0], u[1]-v[1], u[2]-v[2], u[3]-v[3]); };

template<class T> inline TVec4<T> operator-(const TVec4<T> &u)
{ return TVec4<T>(-u[0], -u[1], -u[2], -u[3]); };

template<class T, class N> inline TVec4<T> operator*(N s, const TVec4<T> &v)
{ return TVec4<T>(v[0]*s, v[1]*s, v[2]*s, v[3]*s); };
template<class T, class N> inline TVec4<T> operator*(const TVec4<T> &v, N s)
{ return s*v; };

template<class T, class N> inline TVec4<T> operator/(const TVec4<T> &v, N s)
{ return TVec4<T>(v[0]/s, v[1]/s, v[2]/s, v[3]/s); };

template<class T> inline T operator*(const TVec4<T> &u, const TVec4<T> &v)
{ return u[0]*v[0] + u[1]*v[1] + u[2]*v[2] + u[3]*v[3]; };

template<class T>
inline std::ostream &operator<<(std::ostream &out, const TVec4<T>& v)
{ return out <<v[0] <<" " <<v[1] <<" " <<v[2] <<" " <<v[3]; };

template<class T>
inline std::istream &operator>>(std::istream &in, TVec4<T>& v)
{ return in >> v[0] >> v[1] >> v[2] >> v[3]; };

////////////////////////////////////////////////////////////////////////
//
// Misc. function definitions
//
template<class T>
inline TVec4<T> cross(const TVec4<T>& a, const TVec4<T>& b, const TVec4<T>& c)
{
    // Code adapted from VecLib4d.c in Graphics Gems V

    T d1 = (b[2] * c[3]) - (b[3] * c[2]);
    T d2 = (b[1] * c[3]) - (b[3] * c[1]);
    T d3 = (b[1] * c[2]) - (b[2] * c[1]);
    T d4 = (b[0] * c[3]) - (b[3] * c[0]);
    T d5 = (b[0] * c[2]) - (b[2] * c[0]);
    T d6 = (b[0] * c[1]) - (b[1] * c[0]);

    return TVec4<T>(- a[1] * d1 + a[2] * d2 - a[3] * d3,
		      a[0] * d1 - a[2] * d4 + a[3] * d5,
		    - a[0] * d2 + a[1] * d4 - a[3] * d6,
		      a[0] * d3 - a[1] * d5 + a[2] * d6);
};

template<class T>
inline TVec4<T> cross(const TVec4<T>& u, const TVec4<T>& v) {
    // Code adapted from VecLib4d.c in Graphics Gems V
  return TVec4<T>(u[1]*v[2] - v[1]*u[2],
                  -u[0]*v[2] + v[0]*u[2],
                  u[0]*v[1] - v[0]*u[1],
                  0);
};

template<class T> inline T norm2(const TVec4<T>& v) { return v*v; };
template<class T> inline T norm(const TVec4<T>& v)  { return sqrt(norm2(v)); };

template<class T> inline void unitize(TVec4<T>& v)
{
    T l = norm2(v);
    if( l!=1.0 && l!=0.0 )  v /= sqrt(l);
};

template<class T> inline TVec3<T> proj(const TVec4<T>& v)
{
    TVec3<T> u(v[0], v[1], v[2]);
    if( v[3]!=1.0 && v[3]!=0.0 )
	u /= v[3];
    return u;
};


template<class T>
bool predVec40(const TVec4<T>& lhs, const TVec4<T>& rhs) {
  if (lhs[0] < rhs[0])
    return true;
  else
    return false;
};

template<class T>
bool predVec41(const TVec4<T>& lhs, const TVec4<T>& rhs) {
  if (lhs[1] < rhs[1])
    return true;
  else
    return false;
};

template<class T>
bool predVec42(const TVec4<T>& lhs, const TVec4<T>& rhs) {
  if (lhs[2] < rhs[2])
    return true;
  else
    return false;
};

template<class T>
bool predVec43(const TVec4<T>& lhs, const TVec4<T>& rhs) {
  if (lhs[3] < rhs[3])
    return true;
  else
    return false;
};

typedef TVec4<double> Vec4;
typedef TVec4<float>  Vec4f;
typedef TVec4<int>    Vec4i;

template<class T>
struct Svec4cmp {
  bool operator()(const TVec4<T>& lhs, const TVec4<T>& rhs) const {
    if (lhs[0] < rhs[0] ||
	(lhs[0] == rhs[0] && lhs[1] < rhs[1]) ||
        (lhs[0] == rhs[0] && lhs[1] == rhs[1] && lhs[2] < rhs[2]) ||
        (lhs[0] == rhs[0] && lhs[1] == rhs[1] &&
         lhs[2] == rhs[2] && lhs[3] < rhs[3]))
      return true;
    else
      return false;
  }
};

template<class T> inline void ortho(const TVec4<T>& z,
				    TVec4<T>& x, TVec4<T>& y) {
  if (fabs(z[0]) > 0.5) {
    x[0] = z[1];    x[1] = -z[0];    x[2] = 0;
  }
  else if (fabs(z[1]) > 0.5) {
    x[1] = z[2];    x[2] = -z[1];    x[0] = 0;
  }
  else {
    x[2] = z[0];    x[0] = -z[2];    x[1] = 0;
  }
  unitize(x);

  y[0] = z[1] * x[2] - z[2] * x[1];
  y[1] = z[2] * x[0] - z[0] * x[2];
  y[2] = z[0] * x[1] - z[1] * x[0];
};

#endif // VEC4_H
