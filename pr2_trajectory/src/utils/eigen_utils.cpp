//author: Arjun G Menon
//email:  agmenon@andrew.cmu.edu OR ajn.menon@gmail.com

#include "utils/eigen_utils.h"

void utils_eigen::print_single_line(const Eigen::VectorXd &vec, int indents)
{
  for(int k = 0; k<indents; k++)
    std::cout << "\t";
  
  std::cout << "(";
  for(int i = 0; i<vec.rows()-1; i++)
    std::cout << vec(i,0) << ", ";
  std::cout << vec(vec.rows()-1,0) << ")" << std::endl;
}

void utils_eigen::print_single_line_matrix(const Eigen::MatrixXd &mat, int indents)
{
  for(int k = 0; k<indents; k++)
    std::cout << "\t";
  
  std::cout << "[";
  for(int i = 0; i<mat.rows()-1; i++){
    //print a row
    for(int j = 0; j<mat.cols()-1; j++)
      std::cout << mat(i,j) << ", ";
    //print last elem in row
    std::cout << mat(i,mat.cols()-1) << "; ";
  }
  //print last row
  for(int j = 0; j<mat.cols()-1; j++)
    std::cout << mat(mat.rows()-1,j) << ", ";
  //print last elem in row
  std::cout << mat(mat.rows()-1,mat.cols()-1) << "]" << std::endl;
}
