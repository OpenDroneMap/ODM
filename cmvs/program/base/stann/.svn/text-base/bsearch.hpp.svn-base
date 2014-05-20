/*****************************************************************************/
/*                                                                           */
/*  Header: bsearch.hpp                                                      */
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

#ifndef __SFCNN_BSEARCH__
#define __SFCNN_BSEARCH__

#include <vector>
#include "zorder_lt.hpp"

/*! \file
  \brief Binary search functions
  This file contains binary search functions for z-order operations.
*/

//! Binary search function
/*!
  This function executes a binary search on a vector of pointers to points
  \param A Vector of pointers to search
  \param *q Pointer to query point
  \param lt A less_than comparetor
  \return If found: index of point. Otherwise: index of first smaller point
*/
template<typename Point>
long int BinarySearch(vector<Point *> &A, Point *q, zorder_lt<Point> lt) 
{
	long int low = 0;
	long int high = A.size()-1;
	long int middle;

	while(low <= high)
	{
	  middle = (low+high)/2;
	  if(q == A[middle])
	    return middle;
	  else if(lt(q, A[middle]))
	    high = middle-1;
	  else
	    low = middle+1;
	}
	return middle;
}  

//! A Binary Search function
/*!
  This function conducts a binary search for two points at the same time.
  \param A Reference to the vector of points being searched
  \param q1 pointer to first point to be searched for
  \param q2 pointer to second point to be searched for
  \param lt less than comparetor
  \param p1 reference to return value for q1 location
  \param p2 reference to return value for q2 location
*/

template<typename Point>
void PairBinarySearch(vector<Point> &A, Point q1, 
		      Point q2, zorder_lt<Point> lt, int &p1, int &p2) 
{
  int low_q1=0;
  int low_q2=0;
  int high_q1 = A.size()-1;
  int high_q2 = A.size()-1;
  int middle = 0;
  int middle_store;
  
  p1 = -2;
  p2 = -2;
  
  while((low_q1 == low_q2) && (high_q1 == high_q2) && (p1 == -2) && (p2 == -2))
    {
      middle = (low_q1+high_q1)/2;
      if(q1 == A[middle])
	p1 = middle;
      else if(lt(q1, A[middle]))
	high_q1 = middle-1;
      else
	low_q1 = middle+1;
      if(q2 == A[middle])
	p2 = middle;
      else if(lt(q2, A[middle]))
	high_q2 = middle-1;
      else
	low_q2 = middle+1;
    }
  middle_store = middle;
  while(low_q1 <= high_q1)
    {
      middle = (low_q1+high_q1)/2;
      if(q1 == A[middle])
	break;
      else if(lt(q1, A[middle]))
	high_q1 = middle-1;
      else
	low_q1 = middle+1;
    }
  p1 = middle;
  middle = middle_store;
  while(low_q2 <= high_q2)
    {
      middle = (low_q2+high_q2)/2;
      if(q2 == A[middle])
	break;
      else if(lt(q2, A[middle]))
	high_q2 = middle-1;
      else
	low_q2 = middle+1;
    }
  p2 = middle;
}  

//! A binary search Function.
/*
  This function executes a binary search on a vector of points
  \param A Vector of points to search
  \param q Query point
  \param lt A less_than comparetor
  \return If found: index of point. Otherwise: index of first smaller point
*/
template<typename Point>
long int BinarySearch(vector<Point> &A, Point q, zorder_lt<Point> lt) 
{
	long int low = 0;
	long int high = A.size()-1;
	long int middle = 0;

	while(low <= high)
	{
		middle = (low+high)/2;
		if(q == A[middle])
			return middle;
		else if(lt(q, A[middle]))
			high = middle-1;
		else
			low = middle+1;
	}
	return middle;
}

#endif
