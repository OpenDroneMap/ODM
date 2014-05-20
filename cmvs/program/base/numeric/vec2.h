#ifndef NUMERIC_VEC2_H
#define NUMERIC_VEC2_H

#include <iostream>
#include <cmath>

template<class T>
class TVec2 {
private:
  T m_elt[2];
    
public:
  // Standard constructors
  //
  TVec2(T s=0) { *this = s; }
  TVec2(T x, T y) { m_elt[0]=x; m_elt[1]=y; }
  
  // Copy constructors & assignment operators
  template<class U> TVec2(const TVec2<U>& v) { *this = v; }
  template<class U> TVec2(const U v[2]) { m_elt[0]=v[0]; m_elt[1]=v[1]; }
  template<class U> TVec2& operator=(const TVec2<U>& v)
  { m_elt[0]=v[0];  m_elt[1]=v[1];  return *this; }
  TVec2& operator=(T s) { m_elt[0]=m_elt[1]=s; return *this; }
  
  // Descriptive interface
  //
  typedef T value_type;
  static int size() { return 2; }
  
  // Access methods
  //
  operator       T*()       { return m_elt; }
  operator const T*() const { return m_elt; }
  
  T& operator[](int i)       { return m_elt[i]; }
  T  operator[](int i) const { return m_elt[i]; }
  operator const T*()       { return m_elt; }
  
  // In-place arithmetic methods
  inline TVec2& operator+=(const TVec2& v);
  inline TVec2& operator-=(const TVec2& v);
  inline TVec2& operator*=(T s);
  inline TVec2& operator/=(T s);
  
  inline bool operator==(const TVec2& v) const;
  inline bool operator!=(const TVec2& v) const;
  
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
      m_elt[0] /= denom;	m_elt[1] /= denom;
    }
  }      
};

////////////////////////////////////////////////////////////////////////
//
// Method definitions
//
template<class T> inline TVec2<T>& TVec2<T>::operator+=(const TVec2<T>& v)
{ m_elt[0] += v[0];   m_elt[1] += v[1];   return *this; };

template<class T> inline TVec2<T>& TVec2<T>::operator-=(const TVec2<T>& v)
{ m_elt[0] -= v[0];   m_elt[1] -= v[1];   return *this; };

template<class T> inline TVec2<T>& TVec2<T>::operator*=(T s)
{ m_elt[0] *= s;   m_elt[1] *= s;   return *this; };

template<class T> inline TVec2<T>& TVec2<T>::operator/=(T s)
{ m_elt[0] /= s;   m_elt[1] /= s;   return *this; };

template<class T> inline bool TVec2<T>::operator==(const TVec2<T>& v) const{
    if (m_elt[0] == v.m_elt[0] && m_elt[1] == v.m_elt[1])
	return true;
    else
	return false;
};

template<class T> inline bool TVec2<T>::operator!=(const TVec2<T>& v) const{
    return !(*this == v);
};

////////////////////////////////////////////////////////////////////////
//
// Operator defintions
//

template<class T>
inline TVec2<T> operator+(const TVec2<T> &u, const TVec2<T> &v)
{ return TVec2<T>(u[0]+v[0], u[1]+v[1]); };

template<class T>
inline TVec2<T> operator-(const TVec2<T> &u, const TVec2<T> &v)
{ return TVec2<T>(u[0]-v[0], u[1]-v[1]); };

template<class T> inline TVec2<T> operator-(const TVec2<T> &v)
{ return TVec2<T>(-v[0], -v[1]); };

template<class T, class N> inline TVec2<T> operator*(N s, const TVec2<T> &v)
{ return TVec2<T>(v[0]*s, v[1]*s); };
template<class T, class N> inline TVec2<T> operator*(const TVec2<T> &v, N s)
{ return s*v; };

template<class T, class N> inline TVec2<T> operator/(const TVec2<T> &v, N s)
{ return TVec2<T>(v[0]/s, v[1]/s); };

template<class T> inline T operator*(const TVec2<T> &u, const TVec2<T>& v)
{ return u[0]*v[0] + u[1]*v[1]; };

template<class T> inline TVec2<T> perp(const TVec2<T> &v)
{ return TVec2<T>(v[1], -v[0]); };

template<class T>
inline std::ostream &operator<<(std::ostream &out, const TVec2<T> &v)
{ return out << v[0] << " " << v[1]; };

template<class T>
inline std::istream &operator>>(std::istream &in, TVec2<T>& v)
{ return in >> v[0] >> v[1]; };

//----------------------------------------------------------------------
template<class T> inline T norm2(const TVec2<T>& v)  { return v*v; };
template<class T> inline T norm(const TVec2<T>& v)   { return sqrt(norm2(v)); };

template<class T> inline void unitize(TVec2<T>& v)
{
    T l = norm2(v);
    if( l!=1.0 && l!=0.0 )  v /= sqrt(l);
};

template<class T>
bool predVec20(const TVec2<T>& lhs, const TVec2<T>& rhs) {
  if (lhs[0] < rhs[0])
    return true;
  else
    return false;
};

template<class T>
bool predVec21(const TVec2<T>& lhs, const TVec2<T>& rhs) {
  if (lhs[1] < rhs[1])
    return true;
  else
    return false;
};

template<class T>
int isOverlapUnsorted(const TVec2<T>& lhs,
                    const TVec2<T>& rhs, TVec2<T>& intersect) {
  TVec2<T> lhs2(min(lhs[0], lhs[1]), max(lhs[0], lhs[1]));
  TVec2<T> rhs2(min(rhs[0], rhs[1]), max(rhs[0], rhs[1]));
  return isOverlap(lhs2, rhs2, intersect);
}

template<class T>
int isOverlap(const TVec2<T>& lhs, const TVec2<T>& rhs,
              TVec2<T>& intersect) {
  if (lhs[1] <= rhs[0] || rhs[1] <= lhs[0])
    return 0;
  else {
    intersect[0] = max(lhs[0], rhs[0]);
    intersect[1] = min(lhs[1], rhs[1]);
    return 1;
  }
}

template<class T>
int overlap(const TVec2<T>& lhs, const TVec2<T>& rhs, TVec2<T>& intersect) {
  if (lhs[1] <= rhs[0] || rhs[1] <= lhs[0])
    return 0;
  else {
    intersect[0] = max(lhs[0], rhs[0]);
    intersect[1] = min(lhs[1], rhs[1]);
    return 1;
  }
}

typedef TVec2<double> Vec2;
typedef TVec2<float>  Vec2f;
typedef TVec2<int>    Vec2i;

template<class T>
struct Svec2cmp {
  bool operator()(const TVec2<T>& lhs, const TVec2<T>& rhs) const {
    if (lhs[0] < rhs[0] ||
	(lhs[0] == rhs[0] && lhs[1] < rhs[1]))
      return true;
    else
      return false;
  }
};

template<class T>
struct Svec2icmp {
  bool operator()(const TVec2<T>& lhs, const TVec2<T>& rhs) const {
    if (lhs[0] > rhs[0] ||
	(lhs[0] == rhs[0] && lhs[1] > rhs[1]))
      return true;
    else
      return false;
  }
};

template<class T>
T cross(const TVec2<T>& lhs, const TVec2<T>& rhs) {
  return lhs[0] * rhs[1] - lhs[1] * rhs[0];
}

// Check if 2d line segments p0p1 and q0q1 intersect or not. One
// exception is that when they share a corner (but lines are not
// overlapping), we consider them to be not intersecting.
template<class T>
T isIntersect(const TVec2<T>& p0, const TVec2<T>& p1,
                const TVec2<T>& q0, const TVec2<T>& q1) {
  const TVec2<T> p0q0 = p0 - q0;  const TVec2<T> p0p1 = p0 - p1;
  const TVec2<T> p0q1 = p0 - q1;  const TVec2<T> q0p0 = q0 - p0;
  const TVec2<T> q0q1 = q0 - q1;  const TVec2<T> q0p1 = q0 - p1;

  // Check p0 q0 p1
  const T itmp0 = cross(p0q0, p0p1);
  // Check p0 p1 q1
  const T itmp1 = cross(p0p1, p0q1);

  // Check q0 p0 q1
  const T itmp2 = cross(q0p0, q0q1);
  // Check q0 q1 p1
  const T itmp3 = cross(q0q1, q0p1);

  // First check if all the four points are colinear
  if (itmp0 == 0 && itmp1 == 0) {
    /*
    if (itmp2 != 0 || itmp3 != 0) {
      std::cerr << "Error " << std::endl;      exit (1);
    }
    */
    // Use p0 -> q0 is the positive direction
    const T v_p0 = 0;              const T v_p1 = p0q0 * p0p1;
    const T v_q0 = p0q0 * p0q0;    const T v_q1 = p0q0 * p0q1;
    
    TVec2<T> intersect;
    if (isOverlapUnsorted(TVec2<T>(v_p0, v_p1),
                          TVec2<T>(v_q0, v_q1), intersect))
      return 1;
    else
      return 0;
  }

  //======================================================================
  // Since they are not colinear, if they share a corner, they do not intersect
  if (p0 == q0 || p0 == q1 || p1 == q0 || p1 == q1)
    return 0;
  
  const TVec2<T> p1q0 = p1 - q0;  const TVec2<T> p1q1 = p1 - q1;
  const TVec2<T> q1p0 = q1 - p0;  const TVec2<T> q1p1 = q1 - p1;
  // Next check if one point is on another line
  // Check if q0 is on p0-p1
  if (itmp0 == 0 && p0q0 * p1q0 < 0)
    return 1;
  // Check if q1 is on p0-p1
  if (itmp1 == 0 && p0q1 * p1q1 < 0)
    return 1;
  // Check if p0 is on q0-q1
  if (itmp2 == 0 && q0p0 * q1p0 < 0)
    return 1;
  // Check if p1 is on q0-q1
  if (itmp3 == 0 && q0p1 * q1p1 < 0)
    return 1;

  // Now we know that there is no colinearlity
  if (itmp0 * itmp1 < 0 || itmp2 * itmp3 < 0)
    return 0;
  
  return 1;
  

  /*
  //======================================================================
  // If sharing a corner
  if (p0 == q0) {
    TVec2<T> p1p0 = p1 - p0;
    TVec2<T> q1q0 = q1 - q0;
    // when intersect
    if (p1p0[0] * q1q0[1] - p1p0[1] * q1q0[0] == 0 &&
        0 <= p1p0 * q1q0)
      return 1;
    else
      return 0;
  }
  if (p0 == q1) {
    TVec2<T> p1p0 = p1 - p0;
    TVec2<T> q0q1 = q0 - q1;
    // when intersect
    if (p1p0[0] * q0q1[1] - p1p0[1] * q0q1[0] == 0 &&
        0 <= p1p0 * q0q1)
      return 1;
    else
      return 0;
  }
  if (p1 == q0) {
    TVec2<T> p0p1 = p0 - p1;
    TVec2<T> q1q0 = q1 - q0;
    // when intersect
    if (p0p1[0] * q1q0[1] - p0p1[1] * q1q0[0] == 0 &&
        0 <= p0p1 * q1q0)
      return 1;
    else
      return 0;
  }
  if (p1 == q1) {
    TVec2<T> p0p1 = p0 - p1;
    TVec2<T> q0q1 = q0 - q1;
    // when intersect
    if (p0p1[0] * q0q1[1] - p0p1[1] * q0q1[0] == 0 &&
        0 <= p0p1 * q0q1)
      return 1;
    else
      return 0;
  }
  
  TVec2<T> p0q1 = q1 - p0;
  TVec2<T> p0q0 = q0 - p0;
  TVec2<T> p0p1 = p1 - p0;
  TVec2<T> q1p1 = p1 - q1;
  TVec2<T> q0p1 = p1 - q0;

  const T itmp0 = p0q1[0] * p0p1[1] - p0q1[1] * p0p1[0];
  const T itmp1 = p0p1[0] * p0q0[1] - p0p1[1] * p0q0[0];

  // Check p0, q1, p1
  if (itmp0 == 0 && 0 <= p0q1 * q1p1)
    return 1;
  // Check p0, q0, p1
  if (itmp1 == 0 && 0 <= p0q0 * q0p1)
    return 1;

  if (itmp0 * itmp1 <= 0)
    //if (itmp0 * itmp1 < 0)
    return 0;
  
  TVec2<T> q0q1 = q1 - q0;
  TVec2<T> q0p0 = p0 - q0;
  
  const T itmp2 = q0p0[0] * q0q1[1] - q0p0[1] * q0q1[0];
  const T itmp3 = q0q1[0] * q0p1[1] - q0q1[1] * q0p1[0];
  // Check q0 p0 q1
  if (itmp2 == 0 && 0 <= (-p0q0) * (p0q1))
    return 1;
  // Check q0 p1 q1
  if (itmp3 == 0 && 0 <= q0p1 * (-q1p1))
    return 1;
  
  if (itmp2 * itmp3 <= 0)
    //if (itmp2 * itmp3 < 0)
    return 0;

  
  return 1;
  */
}

#endif // VEC2_H
