/*****************************************************************************/
/*                                                                           */
/*  Header: zorder_lt.hpp                                                    */
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



#ifndef __SFCNN_ZORDER_LT__
#define __SFCNN_ZORDER_LT__

#include "dpoint.hpp"
#include <cmath>
#include <climits>
#include <iostream>
#include "zorder_type_traits.hpp"
#include "sep_float.hpp"
#include "pair_iter.hpp"

using namespace std;

template<typename Point>
class zorder_lt;
template<typename Point, typename CType, typename sign_trait, typename integral_trait, typename sep_trait>
class zorder_lt_worker;

//Z Order spec for unsigned integral types
template<typename Point, typename CType>
class zorder_lt_worker <Point, CType, zorder_f, zorder_t, zorder_f>
{
public:
  zorder_lt_worker(){;}
  ~zorder_lt_worker(){;}
  bool operator()(const Point &p, const Point &q)
  {
    return lt_func(p,q);
  }
  
  double dist_sq_to_quad_box(Point &q, Point &p1, Point &p2)
  {
    unsigned int j;
    int i;
    CType x,y;
    double z;
    z=0;
    for(j=x=0;j < Point::__DIM;++j)
      {
	y = p1[j]^p2[j];
	if(less_msb(x, y)) 
	  {
	    x = y;
	  }
      }
    frexp((float)x, &i);
    for(j=0;j < Point::__DIM;++j)
      {
	x = (((p1)[j])>>i)<<i;
	y = x+(1 << i);
	if(q[j] < x) 
	  z+= pow(((double) q[j]-(double) x), 2.0);
	else if(q[j] > y) 
	  z+= pow(((double) q[j]-(double) y), 2.0);
      }
    //cout << q << endl << p1 << endl << p2 << endl << z << endl;
    return z;
  }
  

private:
  bool lt_func(const Point &p, const Point &q)
  {
    CType j,x,y,k;
    for(j=k=x=0;k < Point::__DIM;++k)
      {
	y = (p[k])^(q[k]);
	if(less_msb(x,y))
	  {
	    j=k;
	    x=y;
	  }
      }
    return p[j] < q[j];
  }

  bool less_msb(CType x, CType y)
  {
    return (x < y) && (x < (x^y));
  }
};


//Z Order spec for signed integral types
template<typename Point, typename CType>
class zorder_lt_worker <Point, CType, zorder_t, zorder_t, zorder_f>
{
public:
  zorder_lt_worker(){;}
  ~zorder_lt_worker(){;}
  bool operator()(const Point &p, const Point &q)
  {
    return lt_func(p,q);
  }

  double dist_sq_to_quad_box(Point &q, Point &p1, Point &p2)
  {
    unsigned int j;
    int i;
    CType x,y;
    double z, X, Y;
    
    x = numeric_limits<CType>::min();
    for(j=0;j < Point::__DIM;++j)
      {
	if((p1[j] < 0) != (p2[j] < 0))
	  {
	    //cout << "Break out" << endl;
	    //cout << "P1: " << p1 << endl << "P2: " << p2 << endl;
	    return 0;
	  }
	y = p1[j]^p2[j];
	if(less_msb(x,y)) x = y;
      }
    frexp((double)x, &i);
    //cout << "i: " << i << endl;
    //int exp;
    for(z=j=0;j < Point::__DIM;++j)
      {
	X = (p1[j]>>i)<<i;
	//frexp(p1[j], &exp);
	//cout << "p1[j]" << (p1[j] >> i) << endl;
	//cout << "exp: " << exp << endl;
	//cout << "X: " << X << endl;
	Y = X+(1 << i);
	if(q[j] < X) 
	  z+=pow(((double) X - (double) q[j]), 2.0);
	else if(q[j] > Y) 
	  z+=pow(((double) q[j] - (double) Y), 2.0);
      }
    return z;
  }
  
private:
  bool lt_func(const Point &p, const Point &q)
  {
    CType j,x,y;
    unsigned int k;
    x = numeric_limits<CType>::min();
    for(j=k=0;k < Point::__DIM;++k)
      {
	if((p[k] < 0) != (q[k] < 0))
	  return p[k] < q[k];
	y = (p[k])^(q[k]);
	if(less_msb(x,y))
	  {
	    j=k;
	    x=y;
	  }
      }
    return p[j] < q[j];
  }
  
  bool less_msb(CType x, CType y)
  {
    return (x < y) && (x < (x^y));
  }

};


//Z Order spec for seperated floating point types
template<typename Point, typename CType>
class zorder_lt_worker <Point, CType, zorder_t, zorder_f, zorder_t>
{
public: 
  zorder_lt_worker()
  {
  }
  ~zorder_lt_worker(){;}
  bool operator()(const Point &p, const Point &q)
  {
    return lt_func(p,q);
  }
  double dist_sq_to_quad_box(Point &q, Point &p1, Point &p2)
  {
    unsigned int j;
    int x,y;
    double box_edge_1, box_edge_2;
    double z;
    typename CType::flt_type box_dist;

    z = 0;
    x = 0;
    //cout << "X: " << x << endl;
    for(j=0;j < Point::__DIM;++j)
      {
	if((p1[j].get_flt() < 0) != (p2[j].get_flt() < 0))
	  return 0;
	y = msdb(p1[j], p2[j]);
	if(y > x)
	  {
	    x = y;
	  }
      }
    box_dist = pow(2.0,x);
    for(j=0;j < Point::__DIM;++j)
      {
	//cout << "p1[j]/2**i: " << floor(p1[j] / box_dist) << endl;
	//box_edge_1 = p1[j].get_flt() - fmod(p1[j].get_flt(),box_dist);
	box_edge_1 = floor(p1[j] / box_dist) * box_dist;
	//cout << "Box1: " << box_edge_1 << endl;
	box_edge_2 = box_edge_1+box_dist;
	
	if(q[j].get_flt() < box_edge_1) 
	  z+=(q[j].get_flt()-box_edge_1)*(q[j].get_flt()-box_edge_1);
	else if(q[j].get_flt() > box_edge_2) 
	  z+= (q[j].get_flt()-box_edge_2)*(q[j].get_flt()-box_edge_2);
      }
    return z;
  }

private:
  bool lt_func(const Point &p, const Point &q)
  {
    int y,x;
    unsigned int k,j;
    j = 0;
    x = 0;
    for(k=0;k < Point::__DIM;++k)
      {
	if((p[k].get_flt() < 0) != (q[k].get_flt() < 0))
	  return p[k].get_flt() < q[k].get_flt();
	y = msdb(p[k], q[k]);
	if(x < y)
	  {
	    j = k;
	    x = y;
	  }
      }
    return p[j] < q[j];
  }
};

template<typename Point>
class zorder_lt
{
public:
  zorder_lt()
  {
    zorder_traits<typename Point::__NumType>::check_type();
  }
  ~zorder_lt(){;}
  bool operator()(const Point &p, const Point &q)
  {
    return lt(p, q);
  }
  bool operator()(const Mypair<typename vector<Point>::iterator,typename vector<long unsigned int>::iterator> &p, const Mypair<typename vector<Point>::iterator, typename vector<long unsigned int>::iterator> &q)
  {
    return lt(p.val1, q.val1);
  }
  double dist_sq_to_quad_box(Point &q, Point &p1, Point &p2)
  {
    return lt.dist_sq_to_quad_box(q,p1,p2);
  }
private:
  zorder_lt_worker<Point, typename Point::__NumType, typename zorder_traits<typename Point::__NumType>::is_signed, typename zorder_traits<typename Point::__NumType>::is_integral, typename zorder_traits<typename Point::__NumType>::is_seperated> lt;
};
#endif
