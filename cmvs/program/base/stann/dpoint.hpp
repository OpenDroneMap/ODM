/*****************************************************************************/
/*                                                                           */
/*  Header: dpoint.hpp                                                       */
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


#ifndef REVIVER_POINT_HPP
#define REVIVER_POINT_HPP

/*! \file dpoint.hpp
     \brief Optimized point class file for fixed small dimensional points. 
*/

#include "assert.hpp"
#include <iostream>
#include <valarray>

/*! \brief N-dimensional point class namespace.
*/
namespace reviver {
// \cond DPOINT
/*
 Forward Declaration of the main Point Class Eucledian d-dimensional point. 
 The distance is L_2
*/
template<typename NumType, unsigned D>
class dpoint;


///////////////////////////////////////////////////////
// Origin of d-dimensional point
///////////////////////////////////////////////////////
template< typename NumType, unsigned D, unsigned I > struct origin
{
   static inline void eval( dpoint<NumType,D>& p )
   {
	  p[I] = 0.0;
      origin< NumType, D, I-1 >::eval( p );
   }
};


// Partial Template Specialization
template <typename NumType, unsigned D> struct origin<NumType, D, 0>
{
   static inline void eval( dpoint<NumType,D>& p )
   {
	  p[0] = 0.0;
   }
};


///////////////////////////////////////////////////////
// Squared Distance of d-dimensional point
///////////////////////////////////////////////////////
template< typename NumType, unsigned D, unsigned I > struct Distance
{
   static inline double eval( const dpoint<NumType,D>& p, 
                              const dpoint<NumType,D>& q )
   {
     double sum = ( (double) p[I] - (double) q[I] );
     sum = sum * sum;
     return sum + Distance< NumType, D, I-1 >::eval( p,q );
   }
};


// Partial Template Specialization
template <typename NumType, unsigned D> struct Distance<NumType, D, 0>
{
   static inline double eval( const dpoint<NumType,D>& p, 
                               const dpoint<NumType,D>& q )
   {
     double sum = ((double) p[0] - (double) q[0]);
     return sum * sum;
   }
};

///////////////////////////////////////////////////////
// Dot Product of two d-dimensional points
///////////////////////////////////////////////////////
template< typename NumType, unsigned D, unsigned I > struct DotProd
{
   static inline NumType eval( const dpoint<NumType,D>& p, 
                               const dpoint<NumType,D>& q )
   {
	  NumType sum = ( p[I] * q[I] );
	  return sum + DotProd< NumType, D, I-1 >::eval( p,q );
   }
};


// Partial Template Specialization
template <typename NumType, unsigned D> struct DotProd<NumType, D, 0>
{
   static inline NumType eval( const dpoint<NumType,D>& p, 
                               const dpoint<NumType,D>& q )
   {
	  return (p[0] * q[0]);
   }
};


///////////////////////////////////////////////////////
// Equality of two d-dimensional points
///////////////////////////////////////////////////////
template< typename NumType, unsigned D, unsigned I > struct IsEqual
{
   static inline bool eval( const dpoint<NumType,D>& p, 
                            const dpoint<NumType,D>& q )
   {
	  if( p[I]  != q[I] ) return false;
	  else return IsEqual< NumType, D, I-1 >::eval( p,q );
   }
};


// Partial Template Specialization
template <typename NumType, unsigned D> struct IsEqual<NumType, D, 0>
{
   static inline NumType eval( const dpoint<NumType,D>& p, 
                               const dpoint<NumType,D>& q )
   {
	   return (p[0] == q[0])?1:0;
   }
};


///////////////////////////////////////////////////////
// Equate two d-dimensional points
///////////////////////////////////////////////////////
template< typename NumType, unsigned D, unsigned I > struct Equate
{
   static inline void eval( dpoint<NumType,D>& p,
                            const dpoint<NumType,D>& q )
   {
	  p[I]  = q[I];
	  Equate< NumType, D, I-1 >::eval( p,q );
   }
};


// Partial Template Specialization
template <typename NumType, unsigned D> struct Equate<NumType, D, 0>
{
   static inline void eval( dpoint<NumType,D>& p,
                            const dpoint<NumType,D>& q )
   {
	   p[0] = q[0];
   }
};



///////////////////////////////////////////////////////
// Add two d-dimensional points
///////////////////////////////////////////////////////
template< typename NumType, unsigned D, unsigned I > struct Add
{
   static inline void eval( dpoint<NumType,D>& result, 
                            const dpoint<NumType,D>& p, 
                            const dpoint<NumType,D>& q )
   {
	  result[I] = p[I]  + q[I];
	  Add< NumType, D, I-1 >::eval( result,p,q );
   }
};


// Partial Template Specialization
template <typename NumType, unsigned D> struct Add<NumType, D, 0>
{
   static inline void eval( dpoint<NumType,D>& result, 
                            const dpoint<NumType,D>& p, 
                            const dpoint<NumType,D>& q )
   {
	   result[0] = p[0] + q[0];
   }
};


///////////////////////////////////////////////////////
// Subtract two d-dimensional points
///////////////////////////////////////////////////////
// Could actually be done using scalar multiplication
// and addition
template< typename NumType, unsigned D, unsigned I > struct Subtract
{
   static inline void eval( dpoint<NumType,D>& result, 
                            const dpoint<NumType,D>& p, 
                            const dpoint<NumType,D>& q )
   {
	  result[I] = p[I]  - q[I];
	  Subtract< NumType, D, I-1 >::eval( result,p,q );
   }
};


// Partial Template Specialization
template <typename NumType, unsigned D> struct Subtract<NumType, D, 0>
{
   static inline void eval( dpoint<NumType,D>& result, 
                            const dpoint<NumType,D>& p, 
                            const dpoint<NumType,D>& q )
   {
	   result[0] = p[0] - q[0];
   }
};

///////////////////////////////////////////////////////
//  Mutiply scalar with d-dimensional point
///////////////////////////////////////////////////////
// Could actually be done using scalar multiplication
// and addition
template< typename NumType, unsigned D, unsigned I > struct Multiply
{
   static inline void eval( dpoint<NumType,D>& result, 
                            const dpoint<NumType,D>& p, NumType k)
   {
	  result[I] = p[I] * k;
	  Multiply< NumType, D, I-1 >::eval( result,p,k );
   }
};


// Partial Template Specialization
template <typename NumType, unsigned D> struct Multiply<NumType, D, 0>
{
   static inline void eval( dpoint<NumType,D>& result, 
                            const dpoint<NumType,D>& p, NumType k )
   {
	   result[0] = p[0] * k;
   }
};

// \endcond
///////////////////////////////////////////////////////
// Main d dimensional Point Class
///////////////////////////////////////////////////////
// NumType = Floating Point Type
// D   = Dimension of Point
/*!
 \brief Point Class Eucledian d-dimensional point. The distance is L_2
*/
template<typename NumType, unsigned D>
class dpoint {

		// Makes Swap operation fast
		NumType  x[D];

public:
	// For access from outside
	/*! Storage type for point coordinates */
	typedef NumType   __NumType;
	/*! Dimension of points */
	static const unsigned int  __DIM =  D;

	/*! Initialize all coordinates to zero. */
	inline void move2origin(){ origin<NumType, D, D-1>::eval(*this); };

	dpoint(){ 
		Assert( (D >= 1), "Dimension < 1 not allowed" ); 
		// move2origin(); 
	};

	// 1 D Point
	dpoint(NumType x0){ x[0] = x0; };

	// 2 D Point
	dpoint(NumType x0,NumType x1){ x[0] = x0;  x[1] = x1; };

	// 3 D Point
	dpoint(NumType x0,NumType x1,NumType x2){  
               x[0] = x0;  x[1] = x1; x[2] = x2; 
        };

	// Array Initialization
	dpoint(NumType ax[]){ for(int i =0; i < D; ++i) x[i] = ax[i]; };

	// Initialization from another point : Copy Constructor
	dpoint(const dpoint<NumType,D>& p){  
		Equate<NumType,D,D-1>::eval((*this),p);	
	};

	// Destructor
	~dpoint(){};

	/*! Returns the dimension */
	inline int      dim() const { return D; };

	/*! Returns the squared distance to a point 
	\param q Point to compute the distance to */
	inline double  sqr_dist(const dpoint<NumType,D> q) const ;

	/*! Returns the distance to a point
	\param q Point to compute distance to */
	inline NumType  distance(const dpoint<NumType,D> q) const ;

	/*! Returns the dot product of two points
	\param q Point to compute dot product with */
	inline NumType  dotprod (const dpoint<NumType,D> q) const ;
	inline NumType  sqr_length(void)  const;
	/*! Normalize the length of a vector defined by the given point to 1, 
            and sets current point to the result */
	inline void     normalize (void);

	/*! Dereference the coordinate at the given index */
	inline NumType& operator[](int i);

	/*! Dereference the coordinate at the given index */
	inline NumType  operator[](int i) const;

	/*! Assignment operator */
	inline dpoint&  operator= (const dpoint<NumType,D>& q);

	/*! Compute the difference between two points */
	template<typename NT, unsigned __DIM>
	friend dpoint<NT,__DIM>   operator- (const dpoint<NT,__DIM>& p, 
                                             const dpoint<NT,__DIM>& q);

	/*! Compute the sum of two points */
	template<typename NT, unsigned __DIM>
	friend dpoint<NT,__DIM>   operator+ (const dpoint<NT,__DIM>& p, 
             				     const dpoint<NT,__DIM>& q);

	/*! Return true if the coordinates of two points are equal */
	template<typename NT, unsigned __DIM>
	friend bool   operator== (const dpoint<NT,__DIM>& p, 
			          const dpoint<NT,__DIM>& q);

	/*! Return true if the coordinates of two points are not equal */
	template<typename NT, unsigned __DIM>
	friend bool   operator!= (const dpoint<NT,__DIM>& p, 
 				  const dpoint<NT,__DIM>& q);


//	inline dpoint&  operator= (const valarray<NumType>& v);
//	inline operator valarray<NumType>() const;

	template<typename __NT,unsigned __DIM>
	friend void iswap(dpoint<__NT,__DIM>& p,dpoint<__NT,__DIM>& q);
};

template<typename NumType, unsigned D>
void dpoint<NumType,D>::normalize (void){
	NumType len = sqrt(sqr_length());
	if (len > 0.00001)
	for(int i = 0; i < D; ++i){
		x[i] /= len;
	}
}

/*
template<typename NumType, unsigned D>
dpoint<NumType,D>::operator valarray<NumType>() const{
	valarray<NumType> result((*this).x , D);
	return result;
}

//Warning : Valarray should be of size D
//TODO: Unwind this for loop into a template system
template<typename NumType, unsigned D>
dpoint<NumType,D>&
dpoint<NumType,D>::operator= (const valarray<NumType>& v){
	dpoint<NumType,D> result;
	for(int i = 0; i < D; i++) (*this).x[i] = v[i];
	return (*this);
}
*/

template<typename NT, unsigned __DIM>
dpoint<NT,__DIM>
operator+ (const dpoint<NT,__DIM>& p, const dpoint<NT,__DIM>& q){
	dpoint<NT,__DIM> result;
	Add<NT,__DIM,__DIM-1>::eval(result,p,q);	
	return result;
}

template<typename NT, unsigned __DIM>
dpoint<NT,__DIM>
operator- (const dpoint<NT,__DIM>& p, const dpoint<NT,__DIM>& q){
	dpoint<NT,__DIM> result;
	Subtract<NT,__DIM,__DIM-1>::eval(result,p,q);	
	return result;
}

template<typename NT, unsigned __DIM>
bool
operator== (const dpoint<NT,__DIM>& p, const dpoint<NT,__DIM>& q){
	return IsEqual<NT,__DIM,__DIM-1>::eval(p,q);	
}

template<typename NT, unsigned __DIM>
bool
operator!= (const dpoint<NT,__DIM>& p, const dpoint<NT,__DIM>& q){
	return !(IsEqual<NT,__DIM,__DIM-1>::eval(p,q));	
}

template<typename NT, unsigned __DIM>
dpoint<NT,__DIM>
operator* (const dpoint<NT,__DIM>& p, const NT k){
	dpoint<NT,__DIM> result;
	Multiply<NT,__DIM,__DIM-1>::eval(result,p,k);	
	return result;
}

template<typename NT, unsigned __DIM>
dpoint<NT,__DIM>
operator/ (const dpoint<NT,__DIM>& p, const NT k){
	Assert( (k != 0), "Hell division by zero man...\n");
	dpoint<NT,__DIM> result;
	Multiply<NT,__DIM,__DIM-1>::eval(result,p,((double)1.0)/k);	
	return result;
}

template < typename NumType, unsigned D >
dpoint<NumType,D>&
dpoint<NumType,D>::operator=(const dpoint<NumType,D> &q)
{
  Assert((this != &q), "Error p = p");
  Equate<NumType,D,D-1>::eval(*this,q);	
  return *this;
}

template < typename NumType, unsigned D >
NumType
dpoint<NumType,D>::operator[](int i) const
{ return x[i]; }

template < typename NumType, unsigned D >
NumType&
dpoint<NumType,D>::operator[](int i)
{ return x[i]; }


template<typename NumType, unsigned D>
double
dpoint<NumType,D>::sqr_dist (const dpoint<NumType,D> q) const {
	return Distance<NumType,D,D-1>::eval(*this,q);	
}

template<typename NumType, unsigned D>
NumType 
dpoint<NumType,D>::distance (const dpoint<NumType,D> q) const {
	return sqrt(
		static_cast<double>(Distance<NumType,D,D-1>::eval(*this,q))
	);	
}


template<typename NumType, unsigned D>
NumType 
dpoint<NumType,D>::dotprod (const dpoint<NumType,D> q) const {
	return DotProd<NumType,D,D-1>::eval(*this,q);	
}

template<typename NumType, unsigned D>
NumType 
dpoint<NumType,D>::sqr_length (void) const {
	return DotProd<NumType,D,D-1>::eval(*this,*this);	
}

template < class NumType, unsigned D >
std::ostream&
operator<<(std::ostream& os,const dpoint<NumType,D> &p)
{
     os << "Point (d=";
	 os << D << ", (";
	 for (int i=0; i<(int)D-1; ++i)
		os << p[i] << ", ";
	return os << p[D-1] << "))";
    
}

template < class NumType, unsigned D >
std::istream&
operator>>(std::istream& is,dpoint<NumType,D> &p)
{
	 for (int i=0; i<D; ++i)
		 if(!(is >> p[i])){
			 if(!is.eof()){
			   std::cerr << "Error Reading Point:" 
				     << is << std::endl;
				exit(1);
			 }
		 }
		 
	return is;
    
}

/*
template<typename __NT,unsigned __DIM>
static inline void iswap(dpoint<__NT,__DIM>& p,dpoint<__NT,__DIM>& q){
	__NT *y;
	y = p.x;
	p.x = q.x;
	q.x = y;
}
*/
}	// Namespace Ends here
#endif
