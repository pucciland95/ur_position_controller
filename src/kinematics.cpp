#include "../include/kinematics.hpp"

Kinematics::Kinematics(std::string robot_description)
{
  if (!kdl_parser::treeFromString(robot_description, this->tree))
  {
    ROS_ERROR("Failed to construct kdl tree");
    return;
  }
}

void Kinematics::setChain(std::string start, std::string end)
{
  this->tree.getChain(start, end, this->robot_chain);
  this->pFKinSolver = std::make_shared<KDL::ChainFkSolverPos_recursive>(robot_chain);
  this->pJacSolver = std::make_shared<KDL::ChainJntToJacSolver>(robot_chain);
}

bool Kinematics::computeFk(Vector6d joints, Eigen::Affine3d& pose)
{
  // Consistency check
  unsigned int nj = robot_chain.getNrOfJoints();

  if (nj != joints.size())
  {
    ROS_ERROR("Error: number of input joints %ld does not coincides with set chain %d", joints.size(), nj);
    return false;
  }

  // Changing datatype
  KDL::JntArray jointpositions = KDL::JntArray(nj);
  for (unsigned int i = 0; i < nj; i++)
    jointpositions(i) = (double)joints[i];

  // Computing FK
  KDL::Frame cartpos;
  int status = pFKinSolver->JntToCart(jointpositions, cartpos);

  if (status < 0)
  {
    ROS_ERROR("Error: could not calculate forward kinematics :(. Error status: %d", status); // See: https://docs.ros.org/en/kinetic/api/orocos_kdl/html/solveri_8hpp_source.html 
    return false;
  }

  // Changing datatype
  tf::transformKDLToEigen(cartpos, pose);

  return true;
}

bool Kinematics::computeJac(Vector6d joints, Matrix6d& J)
{
  // Consistency check
  unsigned int nj = robot_chain.getNrOfJoints();
  if (nj != joints.size())
  {
    ROS_ERROR("Error: number of input joints does not coincides with set chain"); 
    return false;
  }

  // Changing datatype
  KDL::JntArray jointpositions = KDL::JntArray(nj);
  for (unsigned int i = 0; i < nj; i++)
    jointpositions(i) = (double)joints[i];

  KDL::Jacobian jac(nj);
  int status = pJacSolver->JntToJac(jointpositions, jac);

  if (status < 0)
  {
    ROS_ERROR("Error: could not calculate jacobian :(. Error status: %d", status); // See https://docs.ros.org/en/kinetic/api/orocos_kdl/html/solveri_8hpp_source.html 
    return false;
  }

  J = jac.data;

  return true;
}
