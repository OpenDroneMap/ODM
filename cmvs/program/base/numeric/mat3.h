#ifndef NUMERIC_MAT3_H
#define NUMERIC_MAT3_H

#include "vec3.h"

template <class T>
class TMat3
{
 private:
  TVec3<T> m_row[3];
  
 public:
  // Standard constructors
  //
  TMat3() { *this = 0.0; }
  TMat3(const TVec3<T>& r0,const TVec3<T>& r1,const TVec3<T>& r2)
    { m_row[0]=r0; m_row[1]=r1; m_row[2]=r2; }
  TMat3(const TMat3& m) { *this = m; }
  
  // Descriptive interface
  //
  typedef T value_type;
  typedef TVec3<T> vector_type;
  typedef TMat3 inverse_type;
  static int dim() { return 3; }
  
  // Access methods
  // 
  T& operator()(int i, int j)       { return m_row[i][j]; }
  T  operator()(int i, int j) const { return m_row[i][j]; }
  TVec3<T>&       operator[](int i)       { return m_row[i]; }
  const TVec3<T>& operator[](int i) const { return m_row[i]; }
  inline TVec3<T> col(int i) const {return TVec3<T>(m_row[0][i],m_row[1][i],m_row[2][i]);}
  
  operator       T*()       { return m_row[0]; }
  operator const T*()       { return m_row[0]; }
  operator const T*() const { return m_row[0]; }
  
  // Assignment methods
  //
  inline TMat3& operator=(const TMat3& m);
  inline TMat3& operator=(T s);
  
  inline TMat3& operator+=(const TMat3& m);
  inline TMat3& operator-=(const TMat3& m);
  inline TMat3& operator*=(T s);
  inline TMat3& operator/=(T s);
  
  inline bool operator==(const TMat3& m) const;
  inline bool operator!=(const TMat3& m) const;
  
  // Construction of standard matrices
  //
  static TMat3 I();
  static TMat3 outer_product(const TVec3<T>& u, const TVec3<T>& v);
  static TMat3 outer_product(const TVec3<T>& v);
  
  TMat3 &diag(T d);
  TMat3 &ident() { return diag(1.0); }
};

////////////////////////////////////////////////////////////////////////
//
// Methods definitions
//

template <class T>
inline TMat3<T>& TMat3<T>::operator=(const TMat3<T>& m)
{ m_row[0] = m[0]; m_row[1] = m[1]; m_row[2] = m[2];  return *this; };

template <class T>
inline TMat3<T>& TMat3<T>::operator=(T s)
{ m_row[0]=s;  m_row[1]=s;  m_row[2]=s;  return *this; };

template <class T>
inline TMat3<T>& TMat3<T>::operator+=(const TMat3<T>& m)
{ m_row[0] += m[0]; m_row[1] += m[1]; m_row[2] += m[2]; return *this; };

template <class T>
inline TMat3<T>& TMat3<T>::operator-=(const TMat3<T>& m)
{ m_row[0] -= m[0]; m_row[1] -= m[1]; m_row[2] -= m[2]; return *this; };

template <class T>
inline TMat3<T>& TMat3<T>::operator*=(T s)
{ m_row[0] *= s; m_row[1] *= s; m_row[2] *= s;  return *this; };

template <class T>
inline TMat3<T>& TMat3<T>::operator/=(T s)
{ m_row[0] /= s; m_row[1] /= s; m_row[2] /= s;  return *this; };

template <class T>
inline bool TMat3<T>::operator==(const TMat3<T>& m) const {
  if (m_row[0] == m.m_row[0] && m_row[1] == m.m_row[1] && m_row[2] == m.m_row[2])
    return true;
  else
    return false;
};

template <class T>
inline bool TMat3<T>::operator!=(const TMat3<T>& m) const {
  return !(*this == m);
};

////////////////////////////////////////////////////////////////////////
//
// Operator definitions
//

template <class T>
inline TMat3<T> operator+(const TMat3<T>& n, const TMat3<T>& m)
{ return TMat3<T>(n[0]+m[0], n[1]+m[1], n[2]+m[2]); };

template <class T>
inline TMat3<T> operator-(const TMat3<T>& n, const TMat3<T>& m)
{ return TMat3<T>(n[0]-m[0], n[1]-m[1], n[2]-m[2]); };

template <class T>
inline TMat3<T> operator-(const TMat3<T>& m)
{ return TMat3<T>(-m[0], -m[1], -m[2]); };

template <class T>
inline TMat3<T> operator*(T s, const TMat3<T>& m)
{ return TMat3<T>(m[0]*s, m[1]*s, m[2]*s); };
template <class T>
inline TMat3<T> operator*(const TMat3<T>& m, T s)
{ return s*m; };

template <class T>
inline TMat3<T> operator/(const TMat3<T>& m, T s)
{ return TMat3<T>(m[0]/s, m[1]/s, m[2]/s); };

template <class T>
inline TVec3<T> operator*(const TMat3<T>& m, const TVec3<T>& v)
{ return TVec3<T>(m[0]*v, m[1]*v, m[2]*v); };

template <class T>
inline std::ostream &operator<<(std::ostream &out, const TMat3<T>& M)
{ return out << M[0] << std::endl << M[1] << std::endl << M[2]; };

template <class T>
inline std::istream &operator>>(std::istream &in, TMat3<T>& M)
{ return in >> M[0] >> M[1] >> M[2]; };

////////////////////////////////////////////////////////////////////////
//
// Misc. function definitions
//
template <class T>
TMat3<T> TMat3<T>::I() { return TMat3<T>(TVec3<T>(1,0,0), TVec3<T>(0,1,0), TVec3<T>(0,0,1)); };

template <class T>
TMat3<T> &TMat3<T>::diag(T d)
{
  *this = 0.0;
  m_row[0][0] = m_row[1][1] = m_row[2][2] = d;
  return *this;
};

template <class T>
TMat3<T> diag(const TVec3<T>& v)
{
  return TMat3<T>(TVec3<T>(v[0],0,0),  TVec3<T>(0,v[1],0),  TVec3<T>(0,0,v[2]));
};

template <class T>
TMat3<T> TMat3<T>::outer_product(const TVec3<T>& v)
{
  TMat3<T> A;
  T x=v[0], y=v[1], z=v[2];
  
  A(0,0) = x*x;  A(0,1) = x*y;  A(0,2) = x*z;
  A(1,0)=A(0,1); A(1,1) = y*y;  A(1,2) = y*z;
  A(2,0)=A(0,2); A(2,1)=A(1,2); A(2,2) = z*z;
  
  return A;
};

template <class T>
TMat3<T> TMat3<T>::outer_product(const TVec3<T>& u, const TVec3<T>& v)
{
  TMat3<T> A;
  
  for(int i=0; i<3; i++)
    for(int j=0; j<3; j++)
      A(i, j) = u[i]*v[j];
  
  return A;
};

template <class T>
TMat3<T> operator*(const TMat3<T>& n, const TMat3<T>& m)
{
  TMat3<T> A;
  
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      A(i,j) = n[i]*m.col(j);
  
  return A;
};

template <class T>
TMat3<T> adjoint(const TMat3<T>& m)
{
  return TMat3<T>(m[1]^m[2],
		  m[2]^m[0],
		  m[0]^m[1]);
};

template <class T>
T invert(TMat3<T>& inv, const TMat3<T>& m)
{
  TMat3<T> A = adjoint(m);
  T d = A[0] * m[0];
  
  if( d==0.0 )
    return 0.0;
  
  inv = transpose(A) / d;
  return d;
};

template <class T>
inline T det(const TMat3<T>& m) { return m[0] * (m[1] ^ m[2]); };

template <class T>
inline T trace(const TMat3<T>& m) { return m(0,0) + m(1,1) + m(2,2); };

template <class T>
inline TMat3<T> transpose(const TMat3<T>& m)
{ return TMat3<T>(m.col(0), m.col(1), m.col(2)); };

template <class T>
inline TMat3<T> row_extend(const TVec3<T>& v) { return TMat3<T>(v, v, v); };

/*
template <class T>
inline TMat3<T> rodrigues(TVec3<T> axis, const T theta) {
  TMat3<T> matrix;
  const T cost = cos(theta);
  const T sint = sin(theta);
  const T cost2 = 1 - cost;
  const T& wx = axis[0];  const T& wy = axis[1];  const T& wz = axis[2];
  
  matrix[0][0] = cost + wx * wx * cost2;
  matrix[0][1] = wx * wy * cost2 - wz * sint;
  matrix[0][2] = wy * sint + wx * wz * cost2;

  matrix[1][0] = wz * sint + wx * wy * cost2;
  matrix[1][1] = cost + wy * wy * cost2;
  matrix[1][2] = -wx * sint + wy * wz * cost2;

  matrix[2][0] = -wy * sint + wx * wz * cost2;
  matrix[2][1] = wx * sint + wy * wz * cost2;
  matrix[2][2] = cost + wz * wz * cost2;

  return matrix;
};
*/

template <class T>
inline TMat3<T> rodrigues(const TVec3<T>& axis) {
  const T theta = norm(axis);
  if (theta == 0)
    return TMat3<T>::I();
  
  TMat3<T> wx(TVec3<T>(0,        -axis[2], axis[1]),
              TVec3<T>(axis[2],  0,        -axis[0]),
              TVec3<T>(-axis[1], axis[0],  0));

  const T a = sin(theta) / theta;
  const T b = (1 - cos(theta)) / (theta * theta);

  return TMat3<T>::I() + wx * a + (wx * wx) * b;
};

template <class T>
inline TVec3<T> irodrigues(const TMat3<T>& rot) {
  double theta = (trace(rot) - 1) / 2;
  if (1.0 < theta)
    theta = 1.0;
  else if (theta < -1.0)
    theta = -1.0;
  theta = acos(theta);
  
  const double sint = sin(theta);
  const double denom = 2 * sint;
  if (denom == 0.0)
    return TVec3<T>();
  
  TVec3<T> vec(rot[2][1] - rot[1][2],
               rot[0][2] - rot[2][0],
               rot[1][0] - rot[0][1]);

  return theta / denom * vec;
  /*
  vec = theta / denom * vec;
  const T len = norm(vec);
  const int inum = (int)floor(len / (2 * M_PI));
  */
};

typedef TMat3<double> Mat3;
typedef TMat3<float> Mat3f;

#endif // MAT3_H
