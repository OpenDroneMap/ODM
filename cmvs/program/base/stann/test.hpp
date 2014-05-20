/*****************************************************************************/
/*                                                                           */
/*  Header: test.hpp                                                         */
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



#ifndef STANN_TEST
#define STANN_TEST
#include <iostream>
#include <vector>

#include "bruteNN.hpp"
#include "dpoint.hpp"
#include "rand.hpp"
#include "sfcnn.hpp"
#include "sfcnn_knng.hpp"

using namespace std;
/*! \file test.hpp
\brief Header file for the test.cpp program.*/

template<typename Point, typename T>
Point newRandomPoint(T Min, T Max)
{
  double d;
  Point a;
  double max, min;

  max = (double) Max / (double) numeric_limits<T>::max();
  min = (double) Min / (double) numeric_limits<T>::max();
  for(unsigned int i=0;i < Point::__DIM;++i)
    {
      d = __drand48__();
      d = d*(max-min)-max;
      d *= (double) numeric_limits<T>::max();
      d *= -1;
      a[i] = (T) d;
    }
  return a;
}

template<typename T, unsigned DIM>
bool testNN(unsigned int Size, unsigned int k, T min, T max)
{
  typedef reviver::dpoint<T, DIM> Point;
  vector<Point> data;
  vector<Point> query;
  vector<long unsigned int> sfcnn_ans;
  vector<long unsigned int> bf_ans;

  data.resize(Size);
  query.resize(Size);
  
  for(unsigned int i=0;i < data.size();++i)
    {
      data[i]  = newRandomPoint<Point, T>(min, max);
      query[i] = newRandomPoint<Point, T>(min, max);
    }

  bruteNN<Point> BF(&data[0], data.size());
  sfcnn<Point, DIM, T> SFC(&data[0], data.size());

  for(unsigned int i=0;i < data.size();++i)
    {
      BF.ksearch(query[i], k, bf_ans);
      SFC.ksearch(query[i], k, sfcnn_ans); 
      
      for(unsigned int j=0;j < Point::__DIM;++j)
	{
	  if(bf_ans[j] != sfcnn_ans[j])
	    {
	      /*
	      cerr << "SFCNN:" << endl;
	      for(unsigned int q=0;q < sfcnn_ans.size();++q)
		{
		  cerr << sfcnn_ans[q] << endl;
		}
	      cerr << "BF:" << endl;
	      for(unsigned int q=0;q < bf_ans.size();++q)
		{
		  cerr << bf_ans[q] << endl;
		}
	      */
	      return false;
	    }
	}
    }
  return true;
}

template<typename T, unsigned DIM>
bool testKNNG(unsigned int Size, unsigned int k, T min, T max, int num_threads)
{
    typedef reviver::dpoint<T, DIM> Point;
  vector<Point> data;
  vector<long unsigned int> bf_ans;

  data.resize(Size);
  
  for(unsigned int i=0;i < data.size();++i)
    {
      data[i]  = newRandomPoint<Point, T>(min, max);
    }

  bruteNN<Point> BF(&data[0], data.size());
  sfcnn_knng<Point, DIM, T> SFC(&data[0], data.size(), k, num_threads);

  for(unsigned int i=0;i < data.size();++i)
    {
      BF.ksearch(data[i], k+1, bf_ans);
      for(unsigned int j=1;j < k+1;++j)
	{
	  if(bf_ans[j] != SFC[i][j-1])
	    {
	      
	      cerr << "SFCNN:" << endl;
	      for(unsigned int q=0;q < SFC[i].size();++q)
		{
		  cerr << SFC[i][q] << endl;
		}
	      cerr << "BF:" << endl;
	      for(unsigned int q=0;q < bf_ans.size();++q)
		{
		  cerr << bf_ans[q] << endl;
		}
	      
	      return false;
	    }
	}
    }
  return true;
}
#endif
