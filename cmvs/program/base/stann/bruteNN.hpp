/*****************************************************************************/
/*                                                                           */
/*  Header: bruteNN.hpp                                                      */
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


/*! 
  \file
  \brief Brute force nearest neighbor search implementation
  This file contains a simple brute force NN algorithm implementation.
  It is very innefficient and should be used only for accuracy tests.
*/
#include <vector>

#include "qknn.hpp"

using namespace std;

/*!
  \brief A Brute force NN search class
  This class is a simple brute force algorithm for computing nearest neighbors.
  It is very innefficient and should be used only for accuracy tests.
*/
template<typename Point>
class bruteNN
{
public:
  /*! 
    \brief Constructor
    Constructs a brute force nearest neighbor search structure out
    of the given data points.
    \param *points Pointer to list of input points 
    \param N number of input points
  */
  bruteNN(Point *points, long int N);
   /*!
    \brief Destructor
  */
  ~bruteNN();
  /*!
    \brief Nearest neighbor search function
    
    This function returns the nearest k neighbors from the data
    structure to the given query point.  The return vector contains
    the indexes to the answer points in the original data array passed
    at construction time.
    \param q Query point
    \param k number of neighbors to search for
    \param nn_idx Vector in which answer is written
  */
  void ksearch(Point q, int k, 
	       vector<long unsigned int> &nn_idx);
  /*!
    \brief Nearest neighbor search function
    
    This function returns the nearest k neighbors from the data
    structure to the given query point, as well as the squared 
    distance to the query point.  The return vector contains
    the indexes to the answer points in the original data array 
    passed at construction time.
    \param q Query point
    \param k number of neighbors to search for
    \param nn_idx Vector in which answer is written
    \param d_index Vector in which square distances are written
  */
  void ksearch(Point q, int k, 
	       vector<long unsigned int> &nn_idx,
	       vector<double> &d_index);
  
private:
  
  vector<Point> points;
};

template<typename Point>
bruteNN<Point>::bruteNN(Point *p, long int N)
{
  points.resize(N);
  for(int i=0;i < N;++i)
    {
      points[i] = p[i];
    }
}

template<typename Point>
bruteNN<Point>::~bruteNN()
{
}

template<typename Point>
void bruteNN<Point>::ksearch(Point q, int k, 
			     vector<long unsigned int> &nn_idx)
{
  qknn que;
  double distance;
  que.set_size(k);
  for(unsigned int i=0;i < points.size();++i)
    {
      distance = q.sqr_dist(points[i]);
      que.update(distance, i);
    }
  que.answer(nn_idx);
}

template<typename Point>
void bruteNN<Point>::ksearch(Point q, int k, 
			     vector<long unsigned int> &nn_idx, vector<double> &d_indx)
{
  qknn que;
  double distance;
  for(int i=0;i < points.size();++i)
    {
      distance = q.sqr_dist(points[i]);
      que.update(i, distance);
    }
  que.answer(nn_idx, d_indx);
}
