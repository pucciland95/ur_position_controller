#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <iostream>
#include <string>

// ROS
#include "ros/ros.h"

// Eigen
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>

// KDL
#include "kdl_parser/kdl_parser.hpp"
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <eigen_conversions/eigen_kdl.h>

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

class Kinematics
{
public:
  Kinematics(std::string robot_description);
  void setChain(std::string start, std::string end);
  bool computeFk(Vector6d joints, Eigen::Affine3d& pose);
  bool computeJac(Vector6d joints, Matrix6d& J);

private:

  std::string robot_description;
  KDL::Tree tree;
  KDL::Chain robot_chain;
  
  // Solvers
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> pFKinSolver;
  std::shared_ptr<KDL::ChainJntToJacSolver> pJacSolver;
};

#endif