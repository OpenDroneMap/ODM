/*****************************************************************************/
/*                                                                           */
/*  Header: sfcnn_knng.hpp                                                   */
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

#ifndef __KNNGRAPH__
#define __KNNGRAPH__

#include <cstdlib>
#include <cmath>
#include <climits>
#include <vector>
#include <queue>
#include <algorithm>

#include <pthread.h>
#ifdef _OPENMP
#include <omp.h>
#endif

#include "pair_iter.hpp"
#include "qknn.hpp"
#include "zorder_lt.hpp"
#include "bsearch.hpp"


/*!
  \file sfcnn_knng.hpp
  \brief Implements K-Nearest Neighbor Graph construction using SFC nearest neighbor algorithm.  Construction is done in parallel using OpenMP.
*/

using namespace std;

template <typename Point>
class sfcnn_knng_work
{
public:
  
  sfcnn_knng_work(){};
  ~sfcnn_knng_work(){};
  
  void sfcnn_knng_work_init(long int N, unsigned int k, int num_threads);
  
  vector<vector<long unsigned int> > answer;
  vector<Point> points;
  vector<long unsigned int> pointers;
  
private:
  void compute_bounding_box(Point q, Point &q1, Point &q2, double r);
  void recurse(int s,   // Starting index
	       int n,   // Number of points
	       long int q,         // Query point
	       qknn &ans, // Answer que
	       Point &bound_box_lower_corner,
	       Point &bound_box_upper_corner,
	       int initial_scan_lower_range,
	       int initial_scan_upper_range,
	       zorder_lt<Point> &lt);
  
  typename Point::__NumType max, min;
};


template<typename Point>
void sfcnn_knng_work<Point>::sfcnn_knng_work_init(long int N, unsigned int k, int num_threads)
{
  zorder_lt<Point> lt;
  Point bound_box_lower_corner,
	bound_box_upper_corner;
  double distance;
  long int range_b;
  long int range_e;
  
  max = numeric_limits<typename Point::__NumType>::max();
  min = numeric_limits<typename Point::__NumType>::min();
  answer.resize(N);
  
#ifdef _OPENMP
  long int chunk = N/num_threads;
  omp_set_num_threads(num_threads);
#endif

  pair_iter<typename vector<Point>::iterator,
    typename vector<long unsigned int>::iterator> a(points.begin(), pointers.begin()),
    b(points.end(), pointers.end());
  sort(a,b,lt);

  vector<qknn> que;
  que.resize(N);

#ifdef _OPENMP  
#pragma omp parallel private(distance, range_b, range_e) 
#endif
  {
#ifdef _OPENMP  
#pragma omp for schedule(static, chunk)
#endif
    for(long int i=0;i < N;++i)
      {
	range_b = i-k;
	if(range_b < 0) range_b = 0;
	range_e = i+k+1;
	if(range_e > N) range_e = N;
	que[i].set_size(k);
	for(long int j=range_b;j < i;++j)
	  {
	    distance = points[i].sqr_dist(points[j]);
	    que[i].update(distance, pointers[j]);
	  }
	for(long int j=i+1;j < range_e;++j)
	  {
	    distance = points[i].sqr_dist(points[j]);
	    que[i].update(distance, pointers[j]);
	  }
      }
    }
#ifdef _OPENMP  
#pragma omp parallel private(distance, range_b, range_e, bound_box_lower_corner, bound_box_upper_corner) 
#endif
  {
#ifdef _OPENMP  
#pragma omp for schedule(static, chunk)
#endif
      for(long int i=0;i < N;++i)
	{
	  range_b = i-k;
	  if(range_b < 0) range_b = 0;
	  range_e = i+k+1;
	  if(range_e > N) range_e = N;
	  compute_bounding_box(points[i], bound_box_lower_corner, bound_box_upper_corner, sqrt(que[i].topdist()));
	  
	  if(!lt(bound_box_upper_corner, points[range_e]) || !lt(points[range_b], bound_box_lower_corner))
	    {
	      recurse(0, points.size(), i, que[i], 
		      bound_box_lower_corner, 
		      bound_box_upper_corner,
		      range_b,
		      range_e,
		      lt);
	      
	    }
	  que[i].answer(answer[pointers[i]]);
	}
  }
  points.clear();
  pointers.clear();
}

template<typename Point>
void sfcnn_knng_work<Point>::compute_bounding_box(Point q, Point &q1, Point &q2, double R)
{
  typename Point::__NumType radius;
  radius = (typename Point::__NumType) ceil(R);
  for(unsigned int i=0;i<Point::__DIM;++i)
    {
      
      if(q[i] < (min+radius)) q1[i] = min;
      else q1[i] = q[i]-radius;

      if(q[i] > (max-radius)) q2[i] = max;
      else q2[i] = q[i]+radius;
    }
}

template<typename Point>
void sfcnn_knng_work<Point>::recurse(int s,   // Starting index
				     int n,   // Number of points
				     long int q,
				     qknn &ans, // Answer que
				     Point &bound_box_lower_corner,
				     Point &bound_box_upper_corner,
				     int initial_scan_lower_range,
				     int initial_scan_upper_range,
				     zorder_lt<Point> &lt)
{	
  double distance;
  if(n < 4)
    {
      if(n == 0) return;
		
      bool update=false;
      for(int i=0;i < n;++i)
	{
	  if((s+i >= initial_scan_lower_range) 
	     && (s+i < initial_scan_upper_range))
	    continue;
	  distance = points[q].sqr_dist(points[s+i]);
	  update = ans.update(distance, pointers[s+i]) || update;
	}
      if(update)
	compute_bounding_box(points[q], bound_box_lower_corner, bound_box_upper_corner, sqrt(ans.topdist()));
      return;
    }

  if((s+n/2 >= initial_scan_lower_range) && (s+n/2 < initial_scan_upper_range))
    {
    }
  else 
    {
      distance = points[q].sqr_dist(points[s+n/2]);
      if(ans.update(distance, pointers[s+n/2]))
	compute_bounding_box(points[q], bound_box_lower_corner, bound_box_upper_corner, sqrt(ans.topdist()));
    }
	
  if((lt.dist_sq_to_quad_box(points[q],points[s], points[s+n-1])) > ans.topdist())
    return;
	
	
  if(lt(points[q],points[s+n/2]))
    {
      recurse(s, n/2, q, ans, 
	      bound_box_lower_corner, 
	      bound_box_upper_corner, 
	      initial_scan_lower_range, 
	      initial_scan_upper_range,
	      lt);
      if(lt(points[s+n/2],bound_box_upper_corner))
	recurse(s+n/2+1,n-n/2-1, q, ans, 
		bound_box_lower_corner, 
		bound_box_upper_corner, 
		initial_scan_lower_range, 
		initial_scan_upper_range,
		lt);
    }
  else
    {
      recurse(s+n/2+1, n-n/2-1, q, ans, 
	      bound_box_lower_corner, 
	      bound_box_upper_corner, 
	      initial_scan_lower_range, 
	      initial_scan_upper_range,
	      lt);
      if(lt(bound_box_lower_corner,points[s+n/2]))
	recurse(s, n/2, q, ans, 
		bound_box_lower_corner, 
		bound_box_upper_corner, 
		initial_scan_lower_range, 
		initial_scan_upper_range,
		lt);
    }
};

template <typename Point, unsigned int Dim, typename NumType>
class sfcnn_knng
{
public:
  sfcnn_knng(){};
  sfcnn_knng(Point *points, long int N, unsigned int k, int num_threads=1)
  {
    sfcnn_knng_init(points,N,k,num_threads);
  };
  ~sfcnn_knng(){};
  vector<long unsigned int>& operator[](long unsigned int i)
  {
    return NN.answer[i];
  };
private:
  void sfcnn_knng_init(Point *points, long int N, unsigned int k, int num_threads=1)
  {
    NN.points.resize(N);
    NN.pointers.resize(N);
#ifdef _OPENMP
    long int chunk = N/num_threads;
    omp_set_num_threads(num_threads);
#pragma omp parallel for schedule(static, chunk)
#endif
    for(int i=0;i < N;++i)
      {
	NN.pointers[i] = (i);
	for(unsigned int j=0;j < Dim;++j)
	  NN.points[i][j] = points[i][j];
      }
    NN.sfcnn_knng_work_init(N, k, num_threads);
  };
  sfcnn_knng_work<reviver::dpoint<NumType, Dim> > NN;
};

template <typename Point, unsigned int Dim>
class sfcnn_knng <Point, Dim, float>
{
public:
  sfcnn_knng(){};
  sfcnn_knng(Point *points, long int N, unsigned int k, int num_threads=1)
  {
    sfcnn_knng_init(points,N,k,num_threads);
  };
  ~sfcnn_knng(){};
  vector<long unsigned int>& operator[](long unsigned int i)
  {
    return NN.answer[i];
  };
private:
  void sfcnn_knng_init(Point *points, long int N, unsigned int k, int num_threads=1)
  {
    NN.points.resize(N);
    NN.pointers.resize(N);
    
#ifdef _OPENMP
    long int chunk = N/num_threads;
    omp_set_num_threads(num_threads);
#pragma omp parallel for schedule(static, chunk)
#endif
    for(int i=0;i < N;++i)
      {
	NN.pointers[i] = i;
	for(unsigned int j=0;j < Dim;++j)
	  {
	    NN.points[i][j] = points[i][j];
	  }
      }
    NN.sfcnn_knng_work_init(N,k,num_threads);
  };
  sfcnn_knng_work<reviver::dpoint<sep_float<float>, Dim> > NN;
};

template <typename Point, unsigned int Dim>
class sfcnn_knng <Point, Dim, double>
{
public:
  sfcnn_knng(){};
  sfcnn_knng(Point *points, long int N, unsigned int k, int num_threads=1)
  {
    sfcnn_knng_init(points,N,k,num_threads);
  };
  ~sfcnn_knng(){};
  vector<long unsigned int>& operator[](long unsigned int i)
  {
    return NN.answer[i];
  };
private:
  void sfcnn_knng_init(Point *points, long int N, unsigned int k, int num_threads=1)
  {
    NN.points.resize(N);
    NN.pointers.resize(N);
    
#ifdef _OPENMP
    long int chunk = N/num_threads;
    omp_set_num_threads(num_threads);
#pragma omp parallel for schedule(static, chunk)
#endif
    for(int i=0;i < N;++i)
      {
	NN.pointers[i] = i;
	for(unsigned int j=0;j < Dim;++j)
	  NN.points[i][j] = points[i][j];
      }
    NN.sfcnn_knng_work_init(N,k,num_threads);
  };
  sfcnn_knng_work<reviver::dpoint<sep_float<double>, Dim> > NN;
};

template <typename Point, unsigned int Dim>
class sfcnn_knng <Point, Dim, long double>
{
public:
  sfcnn_knng(){};
  sfcnn_knng(Point *points, long int N, unsigned int k, int num_threads=1)
  {
    sfcnn_knng_init(points,N,k,num_threads);
  };
  ~sfcnn_knng(){};
  vector<long unsigned int>& operator[](long unsigned int i)
  {
    return NN.answer[i];
  };
private:
  void sfcnn_knng_init(Point *points, long int N, unsigned int k, int num_threads=1)
  {
    NN.points.resize(N);
    NN.pointers.resize(N);

#ifdef _OPENMP
    long int chunk = N/num_threads;
    omp_set_num_threads(num_threads);
#pragma omp parallel for schedule(static, chunk)
#endif
    for(int i=0;i < N;++i)
      {
	NN.pointers[i] = i;
	for(unsigned int j=0;j < Dim;++j)
	  NN.points[i][j] = points[i][j];
      }
    NN.sfcnn_knng_work_init(N,k,num_threads);
  };
  sfcnn_knng_work<reviver::dpoint<sep_float<long double>, Dim> > NN;
};
#endif
