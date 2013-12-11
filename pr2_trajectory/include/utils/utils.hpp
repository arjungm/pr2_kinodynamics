//author: Arjun G Menon
//email:  agmenon@andrew.cmu.edu OR ajn.menon@gmail.com

#ifndef AGM_UTILS_HPP
#define AGM_UTILS_HPP

#include "common_includes.h"

namespace string_utils{
  inline void split(  const std::string &s, char delim, std::vector< std::string > &elems ) {
    elems.clear();
    std::stringstream ss(s);
    std::string item;
    while(std::getline(ss, item, delim)) {
      elems.push_back(item);
    }
  }

}//end namespace

namespace float_utils{
  bool epsilonCompare(double A, double B, double eps);
}
#endif
