#ifndef NUMERIC_MYLAPACK_H
#define NUMERIC_MYLAPACK_H

#include <vector>

class Cmylapack {
 public:


  // Solve Ax = 0.
  // Values contain singular values stored in an increasing order
  static void hlls(const std::vector<std::vector<float> >& A,
                   std::vector<float>& vec,
                   std::vector<float>& values);

  static void hlls(const std::vector<std::vector<double> >& A,
                   std::vector<double>& vec,
                   std::vector<double>& values);
  
  // Solve Ax = b
  static void lls(const std::vector<std::vector<float> >& A,
                  const std::vector<float>& b,
                  std::vector<float>& x);
  
  static void lls(const std::vector<std::vector<double> >& A,
                  const std::vector<double>& b,
                  std::vector<double>& x);

  static void lls(std::vector<float>& A,
                  std::vector<float>& b,
                  long int width, long int height);
  
  static void lls(std::vector<double>& A,
                  std::vector<double>& b,
                  long int width, long int height);

  // SVD
  // A = U Sigma V^T
  static void svd(const std::vector<std::vector<float> >& A,
                  std::vector<std::vector<float> >& U,
                  std::vector<std::vector<float> >& VT,
                  std::vector<float>& S);
  
};

#endif // NUMERIC_MYLAPACK_H
