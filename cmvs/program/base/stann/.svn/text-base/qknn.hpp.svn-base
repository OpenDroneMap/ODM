/*****************************************************************************/
/*                                                                           */
/*  Header: qknn.hpp                                                         */
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



#ifndef __QKNN_INT__
#define __QKNN_INT__
#include <vector>
#include <queue>
#include <algorithm>
using namespace std;

/*! \file qknn.hpp
\brief Implements priority queue functions for dpoints and point pairs */

//! Priority Queue element comparator
/*! This class orders priority queue elements based on the distance
  given as the first item in the pair 
*/

class q_intelementCompare {
public:

  //! Less than operator
  /*! 
    Compares two priority queue elements based on thier distance
    \param p1 First element to be compared
    \param p2 Second element to be compared
    \return Returns true if p1 distance is less than p2 distance
  */
  bool operator()( const pair<double, long int> p1,  
                   const pair<double, long int> p2 ){
    return p1.first < p2.first;
  }
};

//! Distance Priority Queue
/*! 
  Implements a priority queue for pairs of floating point 
  distances and array indexes.  The priority queue is ordered 
  based on the squared distance stored in the first element
  of the pair.
*/

class qknn 
{
private:
  int K;
  typedef pair<double, long int> q_intelement;
  typedef priority_queue<q_intelement, vector<q_intelement>, q_intelementCompare>  
          PQ;
  PQ pq;
  
public:
  
  //! Constructor
  /*! 
    Creates an empty priority  queue.
   */
  qknn(){};
  
  //! Largest distance
  /*! 
    Returns the largest distance value stored in the priority queue
    \return Largest distance value
  */
  double topdist(void)
  {
    return pq.top().first;
  }
  
  //! Set Size
  /*! 
    Sets the size of the priority queue.  This should be set before the
    queue is used
    \param k The maximum number of elements to be stored in the queue.
  */
  void set_size(int k)
  {
    K = k;
  }
  
  //! Point with largest distance
  /*!
    Returns the index associated with the largest element in the queue.
    \return Index of largest (most distant) element
  */
  long int top()
  {
    return pq.top().second;
  }

  //! Update queue
  /*! 
    Updates the queue with the given distance and point
    \param dist Distance of point to be added
    \param p index of point to be added
    \return True if a point was added to the queue
  */
  bool update(double dist, long int p)
  {
    if(size() < K)
      {
	q_intelement tq(dist, p);
	pq.push(tq);
	return true;
      }
    else if(topdist() > dist)
      {
	pq.pop();
	q_intelement tq(dist, p);
	pq.push(tq);
	return true;
      }
    return false;
  }
  
  //! Create answer
  /*! 
    Transforms the queue into a vector of indeces to points and returns it
    \param pl Vector which will hold the answer after function completes 
  */
  void answer(vector<long unsigned int>& pl)
  {
    pl.resize(K);
    for(int i=pl.size()-1;i >= 0;--i)
      {
	pl[i] = pq.top().second;
	pq.pop();
      }
  };
  //! Create answer
  /*! 
    Transforms the queue into a vector of indeces to points and a 
    vector of squared distances
    \param pl Vector which holds the point indeces after function completes
    \param pd Vector which holds the squared distances from query point
  */
  void answer(vector<long unsigned int>& pl, vector<double> &pd)
  {
    pl.resize(K);
    pd.resize(K);
    for(int i=pl.size()-1;i >= 0; --i)
      {
	pl[i] = pq.top().second;
	pd[i] = pq.top().first;
	pq.pop();
      }
  }
  //! Size function
  /*! 
    Returns the current size of the queue 
    \return Size
  */  
  int size(){ return pq.size(); }
};

#endif
