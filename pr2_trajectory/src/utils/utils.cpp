//author: Arjun G Menon
//email:  agmenon@andrew.cmu.edu OR ajn.menon@gmail.com

#include "utils/utils.hpp"
#include <cmath>
bool float_utils::epsilonCompare(double A, double B, double eps)
{
  double absA = std::abs(A);
  double absB = std::abs(B);
  double diff = std::abs(A-B);

  if(A==B)
    return true;
  else if( A*B == 0 )
    return diff < (eps*eps);
  else
    return diff/(absA+absB) < eps;
}
