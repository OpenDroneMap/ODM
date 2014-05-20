/*****************************************************************************/
/*                                                                           */
/*  Header: pair_iter.hpp                                                    */
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



#ifndef __PAIR_ITER__
#define __PAIR_ITER__

#include <utility>
using namespace std;

/*! \file pair_iter.hpp
\brief Implementation of an iterator class designed to traverse 
two identical vectors simultaneously */

/*! \brief The Mypair class is a utility class for the pair_iter class.  
  It should not be used outside the pair_iter class */
template <typename Iter1, typename Iter2>
class Mypair
{
  typedef typename iterator_traits<Iter1>::value_type value_type1;
  typedef typename iterator_traits<Iter2>::value_type value_type2;
public:
  Mypair(Iter1 a, Iter2 b)
  {
    iter1=a;
    iter2=b;
    val1 = *a;
    val2 = *b;
  }
  Mypair()
  {
    //cout << "Mypair constructor2" << endl;
  };
  ~Mypair()
  {
  };
  
  bool operator<(Mypair<Iter1, Iter2> a) const
  {
    return val1<a.val1;
  }
  bool operator>(Mypair<Iter1, Iter2> a) const
  {
    return val1>a.val1;
   }
  bool operator>=(Mypair<Iter1, Iter2> a) const
  {
    return val1>=a.val1;
   }
  bool operator<=(Mypair<Iter1, Iter2> a) const
  {
    return val1<=a.val1;
   }
  bool operator==(Mypair<Iter1, Iter2> a) const
  {
    return val1==a.val1;
  }
  Mypair& operator=(Mypair<Iter1, Iter2> a)
  {
    //cout << (first()) << "=" << (a.first()) << endl;
    //cout << (second()) << "=" << (a.second()) << endl;
    *iter1 = a.val1;
    *iter2 = a.val2;
    return *this;
  }

  friend void swap(Mypair<Iter1, Iter2> a, Mypair<Iter1, Iter2> b)
  {
    a = b;
    b = a;
  }
  Iter1 iter1;
  Iter2 iter2;
  value_type1 val1;
  value_type2 val2;
};
//! Pair iterator class
/*! 
  The pair_iter class is designed to traverse two vectors 
  simultaneously.  It was intended to be used in conjunction with 
  stl::sort to order two vectors based upon the value of the first.  
  Dereferencing this iterator returns a Mypair object.  This class has not been 
  robustly tested outside the STANN framework, and is not garunteed to work 
  correctly for other uses.
*/
template<typename Iter1, typename Iter2>
class pair_iter
{
public:
  typedef Mypair<Iter1, Iter2> value_type;
  typedef typename iterator_traits<Iter1>::difference_type difference_type;
  typedef value_type* pointer;
  typedef value_type& reference;
  typedef random_access_iterator_tag iterator_category;

  friend pair_iter<Iter1, Iter2> operator+(int n, pair_iter<Iter1, Iter2> a)
  {
    return pair_iter<Iter1, Iter2>(a.iter1+n, a.iter2+n);
  };
  

  //------------------------------------
  //Constructor and Destructor
  //------------------------------------
  pair_iter(){};

  pair_iter(const pair_iter<Iter1, Iter2> &a)
  {
    iter1 = a.iter1;
    iter2 = a.iter2;
  };

  pair_iter(Iter1 i1, Iter2 i2)
  {
    //cout << "Constructor1" << endl;
    iter1=i1;
    iter2=i2;
  };
  pair_iter(Iter1 i1, Iter2 i2, Iter1 i1begin, Iter2 i2begin)
  {
    iter1=i1;
    iter2=i2;
  }
  ~pair_iter(){};
  
  //-------------------------------------
  //Assignment Operator
  //-------------------------------------
  pair_iter<Iter1, Iter2>& operator=(pair_iter<Iter1, Iter2> a)
  {
    //cout << "operator=" << endl;
    iter1 = a.iter1;
    iter2 = a.iter2;
    
    return *this;
  };

  //--------------------------------------
  //Equality Operators
  //--------------------------------------
  bool operator==(pair_iter<Iter1, Iter2> a)
  {
    //cout << "operator==" << endl;
    return iter1==a.iter1;
  };
  
  bool operator!=(pair_iter<Iter1, Iter2> a)
  {
    //cout << "operator!=" << endl;
    return iter1 != a.iter1;
  };
  //--------------------------------------
  //Comparrison Operators
  //--------------------------------------
  bool operator<(pair_iter<Iter1, Iter2> a)
  {
    //cout << "operator<" << endl;
    return iter1 < a.iter1;
  }
  bool operator<=(pair_iter<Iter1, Iter2> a)
  {
    //cout << "operator<=" << endl;
    return iter1 <= a.iter1;
  }
  bool operator>(pair_iter<Iter1, Iter2> a)
  {
    //cout << "operator>" << endl;
    return iter1 > a.iter1;
  }
  bool operator>=(pair_iter<Iter1, Iter2> a)
  {
    //cout << "operator>=" << endl;
    return iter1 >= a.iter1;
  }
  //--------------------------------------
  //Dereference Operator
  //--------------------------------------
  value_type operator*()
  {
    //cout << "operator*" << endl;
    return value_type(iter1, iter2);
    //return *iter1;
  };
  value_type& operator[](int n)
  {
    //cout << "operator[]" << endl;
    Mypair<Iter1, Iter2> ref_pair;
    ref_pair.first=iter1+n;
    ref_pair.second=iter2+n;
    return ref_pair;
    //return *iter1;
  }
  //---------------------------------------
  //Increment and Decrement Operators
  //---------------------------------------
  pair_iter<Iter1, Iter2>& operator++()
  {
    //cout << "operator++" << endl;
    ++iter1;
    ++iter2;
    return *this;
  }

  pair_iter<Iter1, Iter2> operator++(int)
  {

    //cout << "++operator" << endl;
    pair_iter<Iter1, Iter2> c = *this;
    iter1++;
    iter2++;
    return c;
  }
  
  pair_iter<Iter1, Iter2>& operator--()
  {
    //cout << "operator--" << endl;
    --iter1;
    --iter2;
    return *this;
  }
  pair_iter<Iter1, Iter2> operator--(int)
  {
    //cout << "--operator" << endl;
    pair_iter<Iter1, Iter2> c = *this;
    iter1--;
    iter2--;
    return c;
  }
  //---------------------------------------
  //Iterator arithmetic
  //---------------------------------------
  pair_iter<Iter1, Iter2>& operator+=(int n)
  {
    //cout << "operator+=" << endl;
    iter1+=n;
    iter2+=n;
    return *this;
  }
  pair_iter<Iter1, Iter2> operator+(int n)
  {
    //cout << "operator+" << endl;
    return pair_iter<Iter1, Iter2>(iter1+n, iter2+n);
  }

  pair_iter<Iter1, Iter2>& operator-=(int n)
  {
    //cout << "operator-=" << endl;
    iter1-=n;
    iter2-=n;
    return *this;
  }
  pair_iter<Iter1, Iter2> operator-(int n)
  {
    //cout << "operator-" << endl;
    return pair_iter<Iter1, Iter2>(iter1-n, iter2-n);
  }
  
  difference_type operator-(pair_iter<Iter1, Iter2> a)
  {
    //cout << "operator-2" << endl;
    return iter1-a.iter1;
  }
  //---------------------------------------
  //Iterator variables
  //---------------------------------------
  Iter1 iter1;
  Iter2 iter2;
};
#endif
