/*****************************************************************************/
/*                                                                           */
/*  Header: sep_float.hpp                                                    */
/*                                                                           */
/*  Accompanies STANN Version 0.5 Beta                                       */
/*  Aug 05, 2008                                                             */
/*                                                                           */
/*  Copyright 2007, 2008                                                     */
/*  Michael Connor and Piyush Kumar                                          */
/*  Florida State University                                                 */
/*  Tallahassee FL, 32306-4532                                               */
/*                                                                           */
/*****************************************************************************/



#ifndef __SEP_FLOAT__
#define __SEP_FLOAT__

#include <cmath>
#include <iostream>

/*! 
  \file sep_float.hpp
  \brief This class stores a float, double or long double as a 
  significand exponent pair.  It allows basic arithmetic operations as 
  well as computing the most significant differing bit (used for z-order 
  calculations)
*/
using namespace std;

template<typename T>
class sep_float;

/*
  This class extends some of the numeric_limits functionality for seperated
  floating point types.  It does not operate in exact accordance with the
  standard C++ numeric_limits for two reasons.

  First: It does not implement overloads for all numeric_limits functions,
  mainly because they are not all needed for STANN
  
  Second: numeric_limits<T>::min() returns the largest negative value, 
  not the smallest represtentable positive value.
*/

namespace std
{
template<>
class numeric_limits <sep_float<float> >
{
public:
  static const bool is_specialized = true;
  static float max() throw() {return numeric_limits<float>::max();}
  static float min() throw() {return -numeric_limits<float>::max();}
};

template<>
class numeric_limits <sep_float<double> >
{
public:
  static const bool is_specialized = true;
  static double max() throw() {return numeric_limits<double>::max();}
  static double min() throw() {return -numeric_limits<double>::max();}

};

template<>
class numeric_limits <sep_float<long double> >
{
public:
  static const bool is_specialized = true;
  static long double max() throw() {return numeric_limits<long double>::max();}
  static long double min() throw() {return -numeric_limits<long double>::max();}

};
}

//! Seperated float significand
/*! 
  This union stores a seperated float significand.
  It allows access to the significand as a floating 
  point type or as an unsigned long integer
*/
template<typename T>
union sep_float_sig
{
  /*! Union accessor of floating point type */
  T d;
  /*! Union accessor as an array of integer types */
  unsigned long int i[0];
};

//! XOR function
/*! 
  This templated function computes a bitwise XOR of two
  sep_float_sig unions, based on the size of the underlying 
  floating point data type.
  \param a First value to be XORed
  \param b Second value to be XORed
  \param c return value
  \param zero A baseline value used to restore the floating point structure after XOR, as far as I know it should always be 0.5
*/
template<typename T1>
void xor_sep_float_sig(T1 &a, T1 &b, T1 &c, const T1 &zero)
{
  for(unsigned int i=0;i < sizeof(T1)/sizeof(unsigned long int);++i)
    {
      c.i[i]=(a.i[i]^b.i[i])|zero.i[i];
    }
}

/*! sep_float type
  \brief This class stores a float, long or double as an integer significand
  and an integer exponent.
*/
template<typename FLT>
class sep_float
{
  /*! sep_float overload of output stream function
    \brief This will put the floating point number on the
    output stream, not the seperated number
    \param os Output stream to be appended to
    \param x seperated float to be added to output stream
    \return output stream with floating point number appended
   */
  friend ostream& operator<<(ostream& os, const sep_float<FLT> &x)
  {
    os << ldexp(x.sig.d, x.exp);
    return os;
  }
  /*! msdb function
    \brief This function computes the most significant differing 
    bit of two seperated floating point numbers.  The return value is
    the exponent of the highest order bit that differs between the two numbers.
    Note: The msdb of the two floating point numbers is NOT computed based
    on the internal representation of the floating point number.  The answer
    is computed based on a theoretical integer representation of the numbers.
    \param x First value to be compared.
    \param y Second value to be compared.
    \return Most significant differing bit of x and y
  */
  friend int msdb(sep_float<FLT> x, sep_float<FLT> y)
  {
    const sep_float_sig<FLT> lzero = {0.5};

    if(x.val == y.val)
      return numeric_limits<int>::min();
    else if(x.exp == y.exp)
      {
	xor_sep_float_sig(x.sig, y.sig, x.sig, lzero);
	frexp(x.sig.d-0.5, &y.exp);
	return x.exp+y.exp;
      }
    else if(x.exp > y.exp)
      return x.exp;
    else
      return y.exp;
  }
  
  template <typename T>
  friend sep_float<FLT>& operator+=(sep_float<FLT> &y, T &x)
  {
    y = (y.val+x);
    return y;
  }
  template <typename T>
  friend sep_float<FLT>& operator-=(sep_float<FLT> &y, T &x)
  {
    y = (y.val-x);
    return y;
  }
public:
  typedef FLT flt_type;
  sep_float(){;}
  sep_float(FLT x)
  {
    sep_set_val(x);
  }
  
  sep_float(const sep_float<FLT> &x)
  {
    sep_float_copy(x);
  }
  ~sep_float(){;}
  sep_float<FLT>& operator=(const sep_float<FLT> &x)
  {
    sep_float_copy(x);
    return *this;
  }
  template <typename T>
  sep_float<FLT>& operator+=(const T& x)
  {
    sep_set_val(val+x);
    return *this;
  }
  template <typename T>
  sep_float<FLT>& operator-=(const T& x)
  {
    sep_set_val(val-x);
    return *this;
  }
  template <typename T>
  sep_float<FLT>& operator*=(const T& x)
  {
    sep_set_val(val*x);
    return *this;
  }
  template <typename T>
  sep_float<FLT>& operator/=(const T& x)
  {
    sep_set_val(val/x);
    return *this;
  }
  FLT get_flt()
  {
    return val;
  }
  double get_sig()
  {
    return sig.d;
  }
  int get_exp()
  {
    return exp;
  }
  operator double()
  {
    return (double)val;
  }
private:
  void sep_set_val(const FLT &x)
  {
    val = x;
    sig.d = frexp(x, &exp);
  }
  void sep_float_copy(const sep_float<FLT> &x)
  {
    sig.d = x.sig.d;
    exp   = x.exp;
    val   = x.val;
  }
  sep_float_sig<FLT> sig;
  int                exp;
  FLT                val;
};

#endif
