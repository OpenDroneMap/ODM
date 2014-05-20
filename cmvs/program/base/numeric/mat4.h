#ifndef NUMERIC_MAT4_H
#define NUMERIC_MAT4_H

#include <cmath>
#include "vec4.h"
#include "mat3.h"

template <class T>
class TMat4
{
private:
    TVec4<T> m_row[4];

public:
    // Standard constructors
    //
    TMat4() { *this = 0.0; }
    TMat4(const TVec4<T>& r0,const TVec4<T>& r1,const TVec4<T>& r2,const TVec4<T>& r3)
    	{ m_row[0]=r0; m_row[1]=r1; m_row[2]=r2; m_row[3]=r3; }
    TMat4(const TMat4& m) { *this = m; }

    // Descriptive interface
    //
    typedef T value_type;
    typedef TVec4<T> vector_type;
    typedef TMat4 inverse_type;
    static int dim() { return 4; }

    // Access methods
    //
    T& operator()(int i, int j)       { return m_row[i][j]; }
    T  operator()(int i, int j) const { return m_row[i][j]; }
    TVec4<T>&       operator[](int i)       { return m_row[i]; }
    const TVec4<T>& operator[](int i) const { return m_row[i]; }
    inline TVec4<T> col(int i) const
        { return TVec4<T>(m_row[0][i],m_row[1][i],m_row[2][i],m_row[3][i]); }

    operator       T*()       { return m_row[0]; }
    operator const T*()       { return m_row[0]; }
    operator const T*() const { return m_row[0]; }

    // Assignment methods
    //
    inline TMat4& operator=(const TMat4& m);
    inline TMat4& operator=(T s);

    inline TMat4& operator+=(const TMat4& m);
    inline TMat4& operator-=(const TMat4& m);
    inline TMat4& operator*=(T s);
    inline TMat4& operator/=(T s);
    inline bool operator==(const TMat4& m) const;
    inline bool operator!=(const TMat4& m) const;

    static TMat4 I();
};

////////////////////////////////////////////////////////////////////////
//
// Method definitions
//

template <class T>
inline TMat4<T>& TMat4<T>::operator=(const TMat4<T>& m)
{
    m_row[0] = m[0]; m_row[1] = m[1]; m_row[2] = m[2]; m_row[3] = m[3];
    return *this;
};

template <class T>
inline TMat4<T>& TMat4<T>::operator=(T s)
{
    m_row[0]=s;  m_row[1]=s;  m_row[2]=s;  m_row[3]=s;
    return *this;
};

template <class T>
inline TMat4<T>& TMat4<T>::operator+=(const TMat4<T>& m)
{
    m_row[0] += m[0]; m_row[1] += m[1]; m_row[2] += m[2]; m_row[3] += m[3];
    return *this;
};

template <class T>
inline TMat4<T>& TMat4<T>::operator-=(const TMat4<T>& m)
{
    m_row[0] -= m[0]; m_row[1] -= m[1]; m_row[2] -= m[2]; m_row[3] -= m[3];
    return *this;
};

template <class T>
inline TMat4<T>& TMat4<T>::operator*=(T s)
{
    m_row[0] *= s; m_row[1] *= s; m_row[2] *= s; m_row[3] *= s;
    return *this;
};

template <class T>
inline TMat4<T>& TMat4<T>::operator/=(T s)
{
    m_row[0] /= s; m_row[1] /= s; m_row[2] /= s; m_row[3] /= s;
    return *this;
};

template <class T>
inline bool TMat4<T>::operator==(const TMat4<T>& m) const {
    if (m_row[0] == m.m_row[0] && m_row[1] == m.m_row[1] &&
	m_row[2] == m.m_row[2] && m_row[3] == m.m_row[3])
	return true;
    else
	return false;
};

template <class T>
inline bool TMat4<T>::operator!=(const TMat4<T>& m) const {
    return !(*this == m);
};


////////////////////////////////////////////////////////////////////////
//
// Operator definitions
//

template <class T>
inline TMat4<T> operator+(const TMat4<T>& n, const TMat4<T>& m)
{ return TMat4<T>(n[0]+m[0], n[1]+m[1], n[2]+m[2], n[3]+m[3]); };

template <class T>
inline TMat4<T> operator-(const TMat4<T>& n, const TMat4<T>& m)
{ return TMat4<T>(n[0]-m[0], n[1]-m[1], n[2]-m[2], n[3]-m[3]); };

template <class T>
inline TMat4<T> operator-(const TMat4<T>& n)
{ return TMat4<T>(-n[0], -n[1], -n[2], -n[3]); };

template <class T>
inline TMat4<T> operator*(T s, const TMat4<T>& m)
{ return TMat4<T>(m[0]*s, m[1]*s, m[2]*s, m[3]*s); };
template <class T>
inline TMat4<T> operator*(const TMat4<T>& m, T s)
{ return s*m; };

template <class T>
inline TMat4<T> operator/(const TMat4<T>& m, T s)
{ return TMat4<T>(m[0]/s, m[1]/s, m[2]/s, m[3]/s); };

template <class T>
inline TVec4<T> operator*(const TMat4<T>& m, const TVec4<T>& v)
{ return TVec4<T>(m[0]*v, m[1]*v, m[2]*v, m[3]*v); };

//
// Transform a homogeneous 3-vector and reproject into normal 3-space
//
template <class T>
inline TVec3<T> operator*(const TMat4<T>& m, const TVec3<T>& v)
{
    TVec4<T> u=TVec4<T>(v,1);
    T w=m[3]*u;

    if(w==0.0)  return TVec3<T>(m[0]*u, m[1]*u, m[2]*u);
    else        return TVec3<T>(m[0]*u/w, m[1]*u/w, m[2]*u/w);
};

template <class T>
inline std::ostream &operator<<(std::ostream &out, const TMat4<T>& M)
{ return out<<M[0]<<std::endl<<M[1]<<std::endl<<M[2]<<std::endl<<M[3]; };

template <class T>
inline std::istream &operator>>(std::istream &in, TMat4<T>& M)
{ return in >> M[0] >> M[1] >> M[2] >> M[3]; };

////////////////////////////////////////////////////////////////////////
//
// Transformations
//

template <class T>
inline TMat4<T> rotation_matrix_deg(T theta, const TVec3<T>& axis)
{ return rotation_matrix_rad(theta*M_PI/180.0, axis); };

////////////////////////////////////////////////////////////////////////
//
// Misc. function definitions
//
template <class T>
inline T det(const TMat4<T>& m) { return m[0] * cross(m[1], m[2], m[3]); }

template <class T>
inline T trace(const TMat4<T>& m) { return m(0,0)+m(1,1)+m(2,2)+m(3,3); }

template <class T>
inline TMat4<T> transpose(const TMat4<T>& m)
	{ return TMat4<T>(m.col(0), m.col(1), m.col(2), m.col(3)); }

template <class T>
TMat4<T> TMat4<T>::I()
{
    return TMat4<T>(TVec4<T>(1,0,0,0),TVec4<T>(0,1,0,0),TVec4<T>(0,0,1,0),TVec4<T>(0,0,0,1));
}

template <class T>
TMat4<T> translation_matrix(const TVec3<T>& d)
{
    return TMat4<T>(TVec4<T>(1, 0, 0, d[0]),
		TVec4<T>(0, 1, 0, d[1]),
		TVec4<T>(0, 0, 1, d[2]),
		TVec4<T>(0, 0, 0, 1));
}

template <class T>
TMat4<T> scaling_matrix(const TVec3<T>& s)
{
    return TMat4<T>(TVec4<T>(s[0], 0,    0,    0),
		TVec4<T>(0,    s[1], 0,    0),
		TVec4<T>(0,    0,    s[2], 0),
		TVec4<T>(0,    0,    0,    1));
}

template <class T>
TMat4<T> rotation_matrix_rad(T theta, const TVec3<T>& axis)
{
    T c=cos(theta), s=sin(theta),
	xx=axis[0]*axis[0],  yy=axis[1]*axis[1],  zz=axis[2]*axis[2],
	xy=axis[0]*axis[1],  yz=axis[1]*axis[2],  xz=axis[0]*axis[2];

    T xs=axis[0]*s, ys=axis[1]*s, zs=axis[2]*s;

    TMat4<T> M;
    M(0,0)=xx*(1-c)+c;  M(0,1)=xy*(1-c)-zs;  M(0,2)=xz*(1-c)+ys;  M(0,3) = 0;
    M(1,0)=xy*(1-c)+zs;  M(1,1)=yy*(1-c)+c;  M(1,2)=yz*(1-c)-xs;  M(1,3)=0;
    M(2,0)=xz*(1-c)-ys;  M(2,1)=yz*(1-c)+xs;  M(2,2)=zz*(1-c)+c;  M(2,3)=0;
    M(3,0)=0;  M(3,1)=0;  M(3,2)=0;  M(3,3)=1;

    return M;
}

template <class T>
TMat4<T> perspective_matrix(T fovy, T aspect, T zmin, T zmax)
{
    T A, B;
    TMat4<T> M;

    if( zmax==0.0 )
    {
	A = B = 1.0;
    }
    else
    {
	A = (zmax+zmin)/(zmin-zmax);
	B = (2*zmax*zmin)/(zmin-zmax);
    }

    T f = 1.0/tan(fovy*M_PI/180.0/2.0);
    M(0,0) = f/aspect;
    M(1,1) = f;
    M(2,2) = A;
    M(2,3) = B;
    M(3,2) = -1;
    M(3,3) = 0;

    return M;
}

template <class T>
TMat4<T> lookat_matrix(const TVec3<T>& from, const TVec3<T>& at, const TVec3<T>& v_up)
{
    TVec3<T> up = v_up;       unitize(up);
    TVec3<T> f = at - from;   unitize(f);

    TVec3<T> s=f^up;
    TVec3<T> u=s^f;

    // NOTE: These steps are left out of the GL man page!!
    unitize(s);
    unitize(u);

    TMat4<T> M(TVec4<T>(s, 0), TVec4<T>(u, 0), TVec4<T>(-f, 0), TVec4<T>(0, 0, 0, 1));

    return M * translation_matrix(-from);
}

template <class T>
TMat4<T> viewport_matrix(T w, T h)
{
    return scaling_matrix(TVec3<T>(w/2.0, -h/2.0, 1)) *
	translation_matrix(TVec3<T>(1, -1, 0));
}

template <class T>
TMat4<T> operator*(const TMat4<T>& n, const TMat4<T>& m)
{
    TMat4<T> A;
    int i,j;

    for(i=0;i<4;i++)
	for(j=0;j<4;j++)
	    A(i,j) = n[i]*m.col(j);

    return A;
}

template <class T>
TMat4<T> adjoint(const TMat4<T>& m)
{
    TMat4<T> A;

    A[0] = cross( m[1], m[2], m[3]);
    A[1] = cross(-m[0], m[2], m[3]);
    A[2] = cross( m[0], m[1], m[3]);
    A[3] = cross(-m[0], m[1], m[2]);
        
    return A;
}

template <class T>
T invert_cramer(TMat4<T>& inv, const TMat4<T>& m)
{
    TMat4<T> A = adjoint(m);
    T d = A[0] * m[0];

    if( d==0.0 )
	return 0.0;

    inv = transpose(A) / d;
    return d;
}

// Matrix inversion code for 4x4 matrices using Gaussian elimination
// with partial pivoting.  This is a specialized version of a
// procedure originally due to Paul Heckbert <ph@cs.cmu.edu>.
//
// Returns determinant of A, and B=inverse(A)
// If matrix A is singular, returns 0 and leaves trash in B.
//
#define SWAPLOCAL(a, b, t)   {t = a; a = b; b = t;}
template <class T>
T invert(TMat4<T>& B, const TMat4<T>& m)
{
    TMat4<T> A = m;
    int i, j, k;
    T max, t, det, pivot;

    /*---------- forward elimination ----------*/

    for (i=0; i<4; i++)                 /* put identity matrix in B */
        for (j=0; j<4; j++)
            B(i, j) = (T)(i==j);

    det = 1.0;
    for (i=0; i<4; i++) {               /* eliminate in column i, below diag */
        max = -1.;
        for (k=i; k<4; k++)             /* find pivot for column i */
            if (fabs(A(k, i)) > max) {
                max = fabs(A(k, i));
                j = k;
            }
        if (max<=0.) return 0.;         /* if no nonzero pivot, PUNT */
        if (j!=i) {                     /* swap rows i and j */
            for (k=i; k<4; k++)
                SWAPLOCAL(A(i, k), A(j, k), t);
            for (k=0; k<4; k++)
                SWAPLOCAL(B(i, k), B(j, k), t);
            det = -det;
        }
        pivot = A(i, i);
        det *= pivot;
        for (k=i+1; k<4; k++)           /* only do elems to right of pivot */
            A(i, k) /= pivot;
        for (k=0; k<4; k++)
            B(i, k) /= pivot;
        /* we know that A(i, i) will be set to 1, so don't bother to do it */

        for (j=i+1; j<4; j++) {         /* eliminate in rows below i */
            t = A(j, i);                /* we're gonna zero this guy */
            for (k=i+1; k<4; k++)       /* subtract scaled row i from row j */
                A(j, k) -= A(i, k)*t;   /* (ignore k<=i, we know they're 0) */
            for (k=0; k<4; k++)
                B(j, k) -= B(i, k)*t;
        }
    }

    /*---------- backward elimination ----------*/

    for (i=4-1; i>0; i--) {             /* eliminate in column i, above diag */
        for (j=0; j<i; j++) {           /* eliminate in rows above i */
            t = A(j, i);                /* we're gonna zero this guy */
            for (k=0; k<4; k++)         /* subtract scaled row i from row j */
                B(j, k) -= B(i, k)*t;
        }
    }

    return det;
}

template <class T>
inline void Trans2WT(const TMat4<T>& trans, TVec3<T>& w, TVec3<T>& t) {
  for (int y = 0; y < 3; ++y)
    t[y] = trans[y][3];

  const T trace = trans[0][0] + trans[1][1] + trans[2][2];
  const T cosomegalen = std::max(-1.0f, std::min(1.0f, (trace - 1.0f) / 2.0f));
  const T omegalen = acos(cosomegalen);
  const T sinomegalen = sin(omegalen);

  if (sinomegalen == 0.0f)
    w = Vec3f();
  else {
    w[0] = trans[2][1] - trans[1][2];
    w[1] = trans[0][2] - trans[2][0];
    w[2] = trans[1][0] - trans[0][1];

    unitize(w);
    w *= omegalen;      
  }
};

template <class T>
inline void WT2Trans(const TVec3<T>& w, const TVec3<T>& t, TMat4<T>& trans) {
  // set translational component
  for (int y = 0; y < 3; ++y)
    trans[y][3] = t[y];
  trans[3][3] = 1.0;

  // set rotational component
  for (int y = 0; y < 4; ++y)
    for (int x = 0; x < 3; ++x)
      if (x == y)
	trans[y][x] = 1.0;
      else
	trans[y][x] = 0.0;

  const float omegalen = norm(w);
  float omegalenN = omegalen;
  while (2 * M_PI < omegalenN)
    omegalenN -= 2 * M_PI;
  
  if (omegalenN != 0.0) {
    const TVec3<T> wN = w * omegalenN / omegalen;
    const float sinomegalen = sin(omegalenN);
    const float cosomegalen = cos(omegalenN);
    TMat3<T> omegahat = hat(wN);
    TMat3<T> omegahat2 = omegahat * omegahat;

    const float a = sinomegalen / omegalenN;
    const float b = (1.0 - cosomegalen) / (omegalenN * omegalenN);
    
    for (int y = 0; y < 3; ++y)
      for (int x = 0; x < 3; ++x)
	trans[y][x] += a * omegahat[y][x] + b * omegahat2[y][x];
  }
};

template <class T>
inline TMat3<T> hat(const TVec3<T>& vec) {
  return TMat3<T>(TVec3<T>(0.0, -vec[2], vec[1]),
		  TVec3<T>(vec[2], 0.0, -vec[0]),
		  TVec3<T>(-vec[1], vec[0], 0.0));
};

typedef TMat4<double> Mat4;
typedef TMat4<float> Mat4f;

#endif // MAT4_H
