/*****************************************************************************/
/*                                                                           */
/*  Header: sfcnn.hpp                                                        */
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



#ifndef __SFCNN___
#define __SFCNN___
#include <cstdlib>
#include <cmath>
#include <climits>
#include <vector>
#include <queue>
#include <algorithm>
#include <pthread.h>

#include "pair_iter.hpp"
#include "qknn.hpp"
#include "zorder_lt.hpp"
#include "bsearch.hpp"
/*!
	\mainpage STANN Doxygen Index Page
	
	This Doxygen API is intended for users interested in modifying 
	the STANN library.  For instructions on using STANN, see the
	STANN webpage at http://www.compgeom.com/~stann or the README.txt
	file located in the main directory of the STANN distribution.

	The API is still being updated.  For requests/comments/errors, 
	please contact us at stann @ compgeom dot com
*/

/*! 
  \file
  \brief Space filling curve nearest neighbor search
  This file contains the implementation of a space filling curve
  nearest neighbor search data structure
*/

using namespace std;

/*! 
  \brief A space filling curve nearest neighbor class.  
  
  This is the workhorse class for the sfcnn class.
  The Space Filling Curve Nearest Neighbor (SFCNN) algorithm sorts the
  input data set into 2-order Morton ordering. Nearest neighbors are then
  calculated based on that curve. The algorithm has a runtime of   O(ln(N)), 
  a construction time of O(Nlog(N)), and a space requirement of
  O(N). The query functions of the algorithm are thread-safe.
*/
template <typename Point>
class sfcnn_work 
{
  template<typename P, unsigned int D, typename N> friend class sfcnn;
public:
  /*!
    \brief Constructor
  */
  sfcnn_work();
  
  /*!
    \brief Destructor
  */
  ~sfcnn_work();

private:
  /*!
    \brief Nearest Neighbor search function
    Searches for the k nearest neighbors to the point q.  The answer
    vector returned will contain the indexes to the answer points
    This function is thread-safe.  
    \param q The query point
    \param k The number of neighbors to return
    \param nn_idx Answer vector
    \param eps Error tolerence, default of 0.0.
    \param sl Unused, for backwards compatibility
  */
  void ksearch(Point q, unsigned int k, vector<long unsigned int> &nn_idx, float eps = 0);
  void ksearch(Point q, unsigned int k, vector<long unsigned int> &nn_idx, vector<double> &dist, float eps = 0);

  vector<Point> points;
  vector<long unsigned int> pointers;
  zorder_lt<Point> lt;



  float eps;
  typename Point::__NumType max, min;

  void compute_bounding_box(Point q, Point &q1, Point &q2, double r);
  void sfcnn_work_init(int num_threads);

  void ksearch_common(Point q, unsigned int k, long unsigned int j, qknn &que, float Eps);	
  
  inline void recurse(int s, int n, Point q, 
		      qknn &ans, Point &q1, Point &q2, int bb, int bt);
	
};

template<typename Point>
sfcnn_work<Point>::sfcnn_work()
{
}

template<typename Point>
void sfcnn_work<Point>::sfcnn_work_init(int num_threads)
{
  max = numeric_limits<typename Point::__NumType>::max();
  min = numeric_limits<typename Point::__NumType>::min();

  pair_iter<typename vector<Point>::iterator, 
    typename vector<long unsigned int>::iterator> a(points.begin(), pointers.begin()),
    b(points.end(), pointers.end());
  sort(a,b,lt);
}

template<typename Point>
sfcnn_work<Point>::~sfcnn_work()
{
}

template<typename Point>
void sfcnn_work<Point>::recurse(int s,     // Starting index
				int n,     // Number of points
				Point q,  // Query point
				qknn &ans, // Answer que
				Point &bound_box_lower_corner,
				Point &bound_box_upper_corner,
				int initial_scan_lower_range,
				int initial_scan_upper_range)
{
  if(n < 4)
    {
      if(n == 0) return;
		
      bool update=false;
      for(int i=0;i < n;++i)
	{
	  if((s+i >= initial_scan_lower_range) 
	     && (s+i < initial_scan_upper_range))
	    continue;
	  update = ans.update(points[s+i].sqr_dist(q), pointers[s+i]) || update;
	}
      if(update)
	compute_bounding_box(q, bound_box_lower_corner, bound_box_upper_corner, sqrt(ans.topdist()));
      return;
    }
  
  if((s+n/2 >= initial_scan_lower_range) && (s+n/2 < initial_scan_upper_range))
    {
    }
  else if(ans.update(points[s+n/2].sqr_dist(q), pointers[s+n/2]))
    compute_bounding_box(q, bound_box_lower_corner, bound_box_upper_corner, sqrt(ans.topdist()));
  
  double dsqb = lt.dist_sq_to_quad_box(q,points[s], points[s+n-1]);
  
  //cout << "dsqb: " << dsqb << endl;
  //cout << "dist: " << ans.topdist() << endl;
  //cout << "p1  : " << points[s] << endl;
  //cout << "p2  : " << points[s+n-1] << endl;
  if(dsqb > ans.topdist())
    return;
  
	
  if(lt(q,points[s+n/2]))
    {
      //search_queue.push(pair<int, int>(s, n/2));
      recurse(s, n/2, q, ans, 
	      bound_box_lower_corner, 
	      bound_box_upper_corner, 
	      initial_scan_lower_range, 
	      initial_scan_upper_range);
      if(lt(points[s+n/2],bound_box_upper_corner))
	//search_queue.push(pair<int, int>(s+n/2+1, n-n/2-1));
	recurse(s+n/2+1,n-n/2-1, q, ans, 
		bound_box_lower_corner, 
		bound_box_upper_corner, 
		initial_scan_lower_range, 
		initial_scan_upper_range);
    }
  else
    {
      recurse(s+n/2+1, n-n/2-1, q, ans, 
	      bound_box_lower_corner, 
	      bound_box_upper_corner, 
	      initial_scan_lower_range, 
	      initial_scan_upper_range);
      //search_queue.push(pair<int, int>(s+n/2+1, n-n/2-1));
      if(lt(bound_box_lower_corner,points[s+n/2]))
	//search_queue.push(pair<int, int>(s, n/2));
	recurse(s, n/2, q, ans, 
		bound_box_lower_corner, 
		bound_box_upper_corner, 
		initial_scan_lower_range, 
		initial_scan_upper_range);
    }
}

template<typename Point>
void sfcnn_work<Point>::compute_bounding_box(Point q, Point &q1, Point &q2, double R)
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
void sfcnn_work<Point>::ksearch_common(Point q, unsigned int k, long unsigned int query_point_index, qknn &que, float Eps)
{
  Point bound_box_lower_corner, bound_box_upper_corner;
  Point low, high;
  
  que.set_size(k);
  eps=1.0+Eps;
  if(query_point_index >= (k)) query_point_index -= (k);
  else query_point_index=0;
  
  int initial_scan_upper_range=query_point_index+2*k+1;
  if(initial_scan_upper_range > (int)points.size())
    initial_scan_upper_range = points.size();
  
  low = points[query_point_index];
  high = points[initial_scan_upper_range-1];
  for(int i=query_point_index;i<initial_scan_upper_range;++i)
    {
      que.update(points[i].sqr_dist(q), pointers[i]);
    }
  compute_bounding_box(q, bound_box_lower_corner, bound_box_upper_corner, sqrt(que.topdist()));
  
  if(lt(bound_box_upper_corner, high) && lt(low,bound_box_lower_corner))
    {
      //cout << "Inital search break!" << endl;
      //cout << "Bb1: " << bound_box_lower_corner << endl;
      //cout << "Bb2: " << bound_box_upper_corner << endl;
      return;
    }
  
  //Recurse through the entire set
  recurse(0, points.size(), q, que, 
	  bound_box_lower_corner, 
	  bound_box_upper_corner,
	  query_point_index,
	  initial_scan_upper_range);
  //cout << "Bb1: " << bound_box_lower_corner << endl;
  //cout << "Bb2: " << bound_box_upper_corner << endl;
}

template<typename Point>
void sfcnn_work<Point>::ksearch(Point q, unsigned int k, 
			      vector<long unsigned int> &nn_idx, float Eps)
{
  long unsigned int query_point_index;  
  qknn que;
  query_point_index = BinarySearch(points, q, lt);
  ksearch_common(q, k, query_point_index, que, Eps);
  que.answer(nn_idx);
}

template<typename Point>
void sfcnn_work<Point>::ksearch(Point q, unsigned int k, 
				vector<long unsigned int> &nn_idx, vector<double> &dist, float Eps)
{
  long unsigned int query_point_index;  
  qknn que;
  query_point_index = BinarySearch(points, q, lt);
  ksearch_common(q, k, query_point_index, que, Eps);
  que.answer(nn_idx, dist);
}

/*!
  \brief Nearest Neighbor search class
  
  This class is a wrapper class for the sfcnn_work class.
  There are several template specializations for this class
  that select the appropriate seperated float type to be used
  for various floating point coordinate types.
  \param Point Data type that stores user points
  \param Dim Dimension of user points
  \param NumType Data type that stores coordinates of user points
*/
template <typename Point, unsigned int Dim, typename NumType>
class sfcnn
{
public:
  /*! 
    \brief Default Constructor
  */
  sfcnn(){};
  /*! 
    \brief Constructor
    
    Constructs a nearest neighbor data structure using the
    given data points.  
    \param *points Pointer to the first data point
    \param N number of data points
    \param num_threads Currently unused.  (multiple processor construction soon!)
  */
  sfcnn(Point *points, long int N, int num_threads=1)
  {
    sfcnn_init(points,N,num_threads);
  };
  ~sfcnn(){};

  /*!
    \brief Nearest Neighbor search function
    
    Searches for the k nearest neighbors to the point q.  The answer
    vector returned will contain the indexes to the answer points
    This function is thread-safe.  
    \param q The query point
    \param k The number of neighbors to return
    \param nn_idx Answer vector
    \param eps Error tolerence, default of 0.0.
  */
  void ksearch(Point q, unsigned int k, vector<long unsigned int> &nn_idx, float eps = 0)
  {
    NN.ksearch(q,k,nn_idx,eps);
  };
  /*!
    \brief Nearest Neighbor search function
    
    Searches for the k nearest neighbors to the point q.  The answer
    vector returned will contain the indexes to the answer points.
    The distance vector contains the square distances to the answer points.
    This function is thread-safe.  
    \param q The query point
    \param k The number of neighbors to return
    \param nn_idx Answer vector
    \param dist Distance vector
    \param eps Error tolerence, default of 0.0.
  */
  void ksearch(Point q, unsigned int k, vector<long unsigned int> &nn_idx, vector<double> &dist, float eps=0)
  {
    NN.ksearch(q,k,nn_idx,dist,eps);
  };
private:
  sfcnn_work<reviver::dpoint<NumType, Dim> > NN;
  void sfcnn_init(Point *points, long int N, int num_threads=1)
  {
    NN.points.resize(N);
    NN.pointers.resize(N);
    
    for(long int i=0;i < N;++i)
      {
	NN.pointers[i] = (i);
	for(unsigned int j=0;j < Dim;++j)
	  NN.points[i][j] = points[i][j];
      }
    NN.sfcnn_work_init(num_threads);
  };

};

template <typename Point, unsigned int Dim>
class sfcnn <Point, Dim, float>
{
public:
  sfcnn(){};
  sfcnn(Point *points, long int N, int num_threads=1)
  {
    sfcnn_init(points,N,num_threads);
  };
  ~sfcnn(){};
  void ksearch(Point q, unsigned int k, vector<long unsigned int> &nn_idx, float eps = 0)
  {
    reviver::dpoint<sep_float<float>, Dim> Q;
    for(unsigned int i=0;i < Dim;++i)
      Q[i] = q[i];
    NN.ksearch(Q,k,nn_idx,eps);
  };
  void ksearch(Point q, unsigned int k, vector<long unsigned int> &nn_idx, vector<double> &dist, float eps=0)
  {
    reviver::dpoint<sep_float<float>, Dim> Q;
    for(unsigned int i=0;i < Dim;++i)
      Q[i] = q[i];
    NN.ksearch(Q,k,nn_idx,dist,eps);
  };
;
private:
  sfcnn_work<reviver::dpoint<sep_float<float>, Dim> > NN;
  void sfcnn_init(Point *points, long int N, int num_threads=1)
  {
    NN.points.resize(N);
    NN.pointers.resize(N);
    
    for(long int i=0;i < N;++i)
      {
	NN.pointers[i] = i;
	for(unsigned int j=0;j < Dim;++j)
	  {
	    NN.points[i][j] = points[i][j];
	  }
      }
    NN.sfcnn_work_init(num_threads);
  };

};

template <typename Point, unsigned int Dim>
class sfcnn <Point, Dim, double>
{
public:
  sfcnn(){};
  sfcnn(Point *points, long int N, int num_threads=1)
  {
    sfcnn_init(points,N,num_threads);
  };
  ~sfcnn(){};
  void ksearch(Point q, unsigned int k, vector<long unsigned int> &nn_idx, float eps = 0)
  {
    reviver::dpoint<sep_float<double>, Dim> Q;
    for(unsigned int i=0;i < Dim;++i)
      Q[i] = q[i];
    NN.ksearch(Q,k,nn_idx,eps);
  };
  void ksearch(Point q, unsigned int k, vector<long unsigned int> &nn_idx, vector<double> &dist, float eps=0)
  {
    reviver::dpoint<sep_float<double>, Dim> Q;
    for(unsigned int i=0;i < Dim;++i)
      Q[i] = q[i];
    NN.ksearch(Q,k,nn_idx,dist,eps);
  };
private:
  void sfcnn_init(Point *points, long int N, int num_threads=1)
  {
    NN.points.resize(N);
    NN.pointers.resize(N);
    
    for(long int i=0;i < N;++i)
      {
	NN.pointers[i] = i;
	for(unsigned int j=0;j < Dim;++j)
	  NN.points[i][j] = points[i][j];
      }
    NN.sfcnn_work_init(num_threads);
  };
  sfcnn_work<reviver::dpoint<sep_float<double>, Dim> > NN;
};

template <typename Point, unsigned int Dim>
class sfcnn <Point, Dim, long double>
{
public:
  sfcnn(){};
  sfcnn(Point *points, long int N, int num_threads=1)
  {
    sfcnn_init(points,N,num_threads);
  };
  ~sfcnn(){};
  void ksearch(Point q, unsigned int k, vector<long unsigned int> &nn_idx, float eps = 0)
  {
    reviver::dpoint<sep_float<long double>, Dim> Q;
    for(unsigned int i=0;i < Dim;++i)
      Q[i] = q[i];
    NN.ksearch(Q,k,nn_idx,eps);
  };
  void ksearch(Point q, unsigned int k, vector<long unsigned int> &nn_idx, vector<double> &dist, float eps=0)
  {
    reviver::dpoint<sep_float<long double>, Dim> Q;
    for(unsigned int i=0;i < Dim;++i)
      Q[i] = q[i];
    NN.ksearch(Q,k,nn_idx,dist,eps);
  };
private:
  void sfcnn_init(Point *points, long int N, int num_threads=1)
  {
    NN.points.resize(N);
    NN.pointers.resize(N);
    
    for(long int i=0;i < N;++i)
      {
	NN.pointers[i] = i;
	for(unsigned int j=0;j < Dim;++j)
	  NN.points[i][j] = points[i][j];
      }
    NN.sfcnn_work_init(num_threads);
  };
  sfcnn_work<reviver::dpoint<sep_float<long double>, Dim> > NN;
};
#endif
