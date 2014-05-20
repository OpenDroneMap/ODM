/*****************************************************************************/
/*                                                                           */
/*  Header: zorder_type_traits.hpp                                           */
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





#ifndef __ZORDER_TYPE_TRAITS__
#define __ZORDER_TYPE_TRAITS__

#include <iostream>
#include "sep_float.hpp"
using namespace std;

struct zorder_t {};
struct zorder_f {};

template<typename TYPE>
class zorder_traits
{
public:
  static void check_type()
  {
    //cerr << "Error: Type traits not defined." << endl;
    //cerr << "Please ensure the appropriate type is added to zorder_traits.hpp" << endl;
    //cerr << "and contact author for update." << endl;
    //exit(1);
  }
  typedef zorder_t     is_signed;
  typedef zorder_t     is_integral;
  typedef int          unsigned_type;
  typedef unsigned int signed_type;
};

template<>
class zorder_traits <int>
{
public:
  static void check_type(){};
  typedef zorder_t     is_signed;
  typedef zorder_t     is_integral;
  typedef zorder_f     is_seperated;
  typedef unsigned int unsigned_type;
  typedef int          signed_type;
};

template<>
class zorder_traits <unsigned int>
{
public:
  static void check_type(){};
  typedef zorder_f     is_signed;
  typedef zorder_t     is_integral;
  typedef zorder_f     is_seperated;
  typedef unsigned int unsigned_type;
  typedef int          signed_type;
};

template<>
class zorder_traits <char>
{
public:
  static void check_type(){};
  typedef zorder_t      is_signed;
  typedef zorder_t      is_integral;
  typedef zorder_f      is_seperated;
  typedef unsigned char unsigned_type;
  typedef char          signed_type;
};

template<>
class zorder_traits <unsigned char>
{
public:
  static void check_type(){};
  typedef zorder_f      is_signed;
  typedef zorder_t      is_integral;
  typedef zorder_f      is_seperated;
  typedef unsigned char unsigned_type;
  typedef char          signed_type;
};

template<>
class zorder_traits <short>
{
public:
  static void check_type(){};
  typedef zorder_t       is_signed;
  typedef zorder_t       is_integral;
  typedef zorder_f       is_seperated;
  typedef unsigned short unsigned_type;
  typedef short          signed_type;
};

template<>
class zorder_traits <unsigned short>
{
public:
  static void check_type(){};
  typedef zorder_f       is_signed;
  typedef zorder_t       is_integral;
  typedef zorder_f       is_seperated;
  typedef unsigned short unsigned_type;
  typedef unsigned short signed_type;
};

template<>
class zorder_traits <long>
{
public:
  static void check_type(){};
  typedef zorder_t     is_signed;
  typedef zorder_t     is_integral;
  typedef zorder_f     is_seperated;
  typedef long unsigned int unsigned_type;
  typedef long int          signed_type;
};

template<>
class zorder_traits <unsigned long>
{
public:
  static void check_type(){};
  typedef zorder_f     is_signed;
  typedef zorder_t     is_integral;
  typedef zorder_f     is_seperated;
  typedef long unsigned int unsigned_type;
  typedef long unsigned int signed_type;
};

template<>
class zorder_traits <float>
{
public:
  static void check_type(){};
  typedef zorder_t       is_signed;
  typedef zorder_f       is_integral;
  typedef zorder_f       is_seperated;
  typedef float          unsigned_type;
  typedef float          signed_type;
};

template<>
class zorder_traits <double>
{
public:
  static void check_type(){};
  typedef zorder_t       is_signed;
  typedef zorder_f       is_integral;
  typedef zorder_f       is_seperated;
  typedef double         unsigned_type;
  typedef double         signed_type;
};

template<>
class zorder_traits <long double>
{
public:
  static void check_type(){};
  typedef zorder_t       is_signed;
  typedef zorder_f       is_integral;
  typedef zorder_f       is_seperated;
  typedef long double    unsigned_type;
  typedef long double    signed_type;
};

template<>
class zorder_traits <sep_float<float> >
{
public:
  static void check_type(){};
  typedef zorder_t       is_signed;
  typedef zorder_f       is_integral;
  typedef zorder_t       is_seperated;
};

template<>
class zorder_traits <sep_float<double> >
{
public:
  static void check_type(){};
  typedef zorder_t       is_signed;
  typedef zorder_f       is_integral;
  typedef zorder_t       is_seperated;
};

template<>
class zorder_traits <sep_float<long double> >
{
public:
  static void check_type(){};
  typedef zorder_t       is_signed;
  typedef zorder_f       is_integral;
  typedef zorder_t       is_seperated;
};

#endif
