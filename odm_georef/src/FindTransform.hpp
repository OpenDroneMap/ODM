// C++
#include <math.h>
#include <string>
#include <iomanip>
#include <sstream>
#include <iostream>

/*!
  * \brief Handles basic 3d vector math.
  **/
struct Vec3
{
    Vec3(double x = 0.0, double y = 0.0, double z = 0.0);
    Vec3(const Vec3 &o);
    
    double x_,y_,z_;    /**< The x, y and z values of the vector. **/
    
    /*!
      * \brief cross     The cross product between two vectors.
      **/
    Vec3 cross(Vec3 o) const;
    
    /*!
      * \brief dot     The scalar product between two vectors.
      **/
    double dot(Vec3 o) const;
    
    /*!
      * \brief length     The length of the vector.
      **/
    double length() const;
    
    /*!
      * \brief norm     Returns a normalized version of this vector.
      **/
    Vec3 norm() const;
    
    /*!
      * \brief Scales this vector.
      **/
    Vec3 operator*(double d) const;
    
    /*!
      * \brief Addition between two vectors.
      **/
    Vec3 operator+(Vec3 o) const;
    
    /*!
      * \brief Subtraction between two vectors.
      **/
    Vec3 operator-(Vec3 o) const;
    
    friend std::ostream & operator<<(std::ostream &os, Vec3 v)
    {
        return os << "[" << std::setprecision(8) << v.x_ << ", " << std::setprecision(4) << v.y_ << ", " << v.z_ << "]";
    }
};

/*!
  * \brief Describes a 3d orthonormal matrix.
  **/
class OnMat3
{
public:
    OnMat3(Vec3 r1, Vec3 r2, Vec3 r3);
    OnMat3(const OnMat3 &o);

    Vec3 r1_;   /**< The first row of the matrix. **/
    Vec3 r2_;   /**< The second row of the matrix. **/
    Vec3 r3_;   /**< The third row of the matrix. **/
    Vec3 c1_;   /**< The first column of the matrix. **/
    Vec3 c2_;   /**< The second column of the matrix. **/
    Vec3 c3_;   /**< The third column of the matrix. **/
    
    /*!
      * \brief The determinant of the matrix.
      **/
    double det() const;
    
    /*!
      * \brief The transpose of the OnMat3 (equal to inverse).
      **/
    OnMat3 transpose() const;
    
    /*!
      * \brief Matrix multiplication between two ON matrices.
      **/ 
    OnMat3 operator*(OnMat3 o) const;
    
    /*!
      * \brief Right side multiplication with a 3d vector.
      **/ 
    Vec3 operator*(Vec3 o);
    
    friend std::ostream & operator<<(std::ostream &os, OnMat3 m)
    {
        return os << "[" << std::endl << m.r1_ << std::endl << m.r2_ << std::endl << m.r3_ << std::endl << "]" << std::endl;
    }
};

/*!
  * \brief Describes an affine transformation.
  **/
class Mat4
{
public:
    Mat4();
    Mat4(OnMat3 rotation, Vec3 translation, double scaling);
    
    /*!
      * \brief Right side multiplication with a 3d vector.
      **/ 
    Vec3 operator*(Vec3 o);
    
    double r1c1_;   /**< Matrix element 0 0 **/
    double r1c2_;   /**< Matrix element 0 1 **/
    double r1c3_;   /**< Matrix element 0 2 **/
    double r1c4_;   /**< Matrix element 0 3 **/
    double r2c1_;   /**< Matrix element 1 0 **/
    double r2c2_;   /**< Matrix element 1 1 **/
    double r2c3_;   /**< Matrix element 1 2 **/
    double r2c4_;   /**< Matrix element 1 3 **/
    double r3c1_;   /**< Matrix element 2 0 **/
    double r3c2_;   /**< Matrix element 2 1 **/
    double r3c3_;   /**< Matrix element 2 2 **/
    double r3c4_;   /**< Matrix element 2 3 **/
    double r4c1_;   /**< Matrix element 3 0 **/
    double r4c2_;   /**< Matrix element 3 1 **/
    double r4c3_;   /**< Matrix element 3 2 **/
    double r4c4_;   /**< Matrix element 3 3 **/
    
    friend std::ostream & operator<<(std::ostream &os, Mat4 m)
    {
        std::stringstream ss;
        ss.precision(8);
        ss.setf(std::ios::fixed, std::ios::floatfield);
        
        ss << "[ " << m.r1c1_ << ",\t" << m.r1c2_ << ",\t" << m.r1c3_ << ",\t" << m.r1c4_ << " ]" << std::endl << 
              "[ " << m.r2c1_ << ",\t" << m.r2c2_ << ",\t" << m.r2c3_ << ",\t" << m.r2c4_ << " ]" << std::endl << 
              "[ " << m.r3c1_ << ",\t" << m.r3c2_ << ",\t" << m.r3c3_ << ",\t" << m.r3c4_ << " ]" << std::endl << 
              "[ " << m.r4c1_ << ",\t" << m.r4c2_ << ",\t" << m.r4c3_ << ",\t" << m.r4c4_ << " ]";
        
        return os << ss.str();
    }
    
};

class FindTransform
{
public:
    /*!
      * \brief findTransform    Generates an affine transform from the three 'from' vector to the three 'to' vectors.
      *                         The transform is such that transform * fromA = toA,
      *                                                    transform * fromB = toB,
      *                                                    transform * fromC = toC,
      **/ 
    void findTransform(Vec3 fromA, Vec3 fromB, Vec3 fromC, Vec3 toA, Vec3 toB, Vec3 toC);
    
    /*!
      * \brief error     Returns the distance beteween the 'from' and 'to' vectors, after the transform has been applied.
      **/ 
    double error(Vec3 fromA, Vec3 toA);
    
    Mat4 transform_;    /**< The affine transform. **/
};
