//author: Arjun G Menon
//email:  agmenon@andrew.cmu.edu OR ajn.menon@gmail.com

#ifndef AGM_EIG_UTILS_HPP
#define AGM_EIG_UTILS_HPP

#include "common_includes.h"
#include <Eigen/Core>
#include <Eigen/LU>

namespace utils_eigen{
  void print_single_line(const Eigen::VectorXd &vec, int indents);
  void print_single_line_matrix(const Eigen::MatrixXd &mat, int indents);
}

#endif
