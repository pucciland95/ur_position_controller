#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include <iostream>
#include <tuple>
#include <functional>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include "chrono"

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/JointState.h"

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

class TimeLaw
{
public:
  virtual ~TimeLaw() = default;
  virtual double evaluate(double t) = 0;
  virtual double evaluateDerivative(double t) = 0;
  virtual std::vector<double> getParameters() = 0;

  double trajectory_duration = 0;
};

typedef std::shared_ptr<TimeLaw> TimeLawShrPtr;

class QuinticTimeLaw : public TimeLaw
{
public:
  QuinticTimeLaw(Eigen::Vector3d final_position, Eigen::Vector3d start_position, 
                 Eigen::Vector3d final_vel, Eigen::Vector3d start_vel,
                 Eigen::Vector3d final_acc, Eigen::Vector3d start_acc,
                  double delta_time)
  {
    double s_ti = 0.0;
    double s_tf = (final_position - start_position).norm();
    double s_dot_ti = start_vel.norm();
    double s_dot_tf = final_vel.norm();
    double s_dot_dot_ti = start_acc.norm();
    double s_dot_dot_tf = final_acc.norm();

    this->initialise(s_ti, s_tf, s_dot_ti, s_dot_tf, s_dot_dot_ti, s_dot_dot_tf, delta_time);
  }
  
  QuinticTimeLaw(double final_orientation, double start_orientation, 
                 double final_omega, double start_omega,
                 double final_omega_dot, double start_omega_dot, double delta_time)
  {
    double s_ti = start_orientation;
    double s_tf = final_orientation - start_orientation;
    double s_dot_ti = start_omega;
    double s_dot_tf = final_omega;
    double s_dot_dot_ti = start_omega_dot;
    double s_dot_dot_tf = final_omega_dot;

    this->initialise(s_ti, s_tf, s_dot_ti, s_dot_tf, s_dot_dot_ti, s_dot_dot_tf, delta_time);
  }

  double evaluate(double t)
  {
    double s = a0 + a1*t + a2*t*t + a3*t*t*t + a4*t*t*t*t +  a5*t*t*t*t*t;
    return s;
  }

  double evaluateDerivative(double t)
  {
    return a1 + 2*a2*t + 3*a3*t*t + 4*a4*t*t*t + 5*a5*t*t*t*t;
  }

  std::vector<double> getParameters()
  {
    std::vector<double> parameters = {a0, a1, a2, a3, a4, a5};
    return parameters;
  }

private:

  void initialise(double s_ti, double s_tf, double s_dot_ti, double s_dot_tf, double s_dot_dot_ti, double s_dot_dot_tf, double delta_time)
  {
    this->a0 = s_ti;
    this->a1 = s_dot_ti;
    this->a2 = 0.5 * s_dot_dot_ti;
    this->a3 = ( 20*(s_tf-s_ti) - (8*s_dot_tf+12*s_dot_ti)*delta_time - (3*s_dot_dot_tf - s_dot_dot_ti)*delta_time*delta_time ) / delta_time / delta_time / delta_time / 0.5;
    this->a4 = ( 30*(s_ti-s_tf) + (14*s_dot_tf+16*s_dot_ti) * delta_time + (3*s_dot_dot_tf-2*s_dot_dot_ti)*delta_time*delta_time ) / delta_time / delta_time / delta_time / delta_time / 2.0;
    this->a5 = ( 12 * (s_tf-s_ti) - 6*(s_dot_tf+s_dot_ti)*delta_time - (s_dot_dot_tf-s_dot_dot_ti)*delta_time*delta_time ) / delta_time / delta_time / delta_time / delta_time / delta_time / 2.0;
  }

  double a0;
  double a1;
  double a2;
  double a3;
  double a4;
  double a5;
  ros::Time initialTime;
};

class CubicalTimeLaw : public TimeLaw
{
public:
  CubicalTimeLaw(Eigen::Vector3d final_position, Eigen::Vector3d start_position, 
                 Eigen::Vector3d final_vel, Eigen::Vector3d start_vel, double delta_time)
  {
    double s_ti = 0.0;
    double s_tf = (final_position - start_position).norm();
    double s_dot_ti = start_vel.norm();
    double s_dot_tf = final_vel.norm();

    this->initialise(s_ti, s_tf, s_dot_ti, s_dot_tf, delta_time);
  }

  
  CubicalTimeLaw(double final_orientation, double start_orientation, 
                 double final_omega, double start_omega, double delta_time)
  {
    double s_ti = start_orientation;
    double s_tf = final_orientation - start_orientation;
    double s_dot_ti = start_omega;
    double s_dot_tf = final_omega;

    this->initialise(s_ti, s_tf, s_dot_ti, s_dot_tf, delta_time);
  }

  double evaluate(double t)
  {
    double s = a0 + a1*t + a2*t*t + a3*t*t*t;

    return s;
  }

  double evaluateDerivative(double t)
  {
    return a1 + 2*a2*t + 3*a3*t*t;
  }

  std::vector<double> getParameters()
  {
    std::vector<double> parameters = {a0, a1, a2, a3};
    return parameters;
  }

private:

  void initialise(double s_ti, double s_tf, double s_dot_ti, double s_dot_tf, double delta_time)
  {
    this->a0 = s_ti;
    this->a1 = s_dot_ti;
    this->a2 = (-3.0 * (s_ti - s_tf) - (2.0 * s_dot_ti + s_dot_tf) * delta_time) / delta_time / delta_time;
    this->a3 = (2.0 * (s_ti - s_tf) + (s_dot_ti + s_dot_tf) * delta_time) / delta_time / delta_time / delta_time;

    this->trajectory_duration = delta_time;
  }

  double a0;
  double a1;
  double a2;
  double a3;

};

class LinearTimeLaw : public TimeLaw
{
public:
  LinearTimeLaw(Eigen::Vector3d final_position, Eigen::Vector3d start_position, double delta_time)
  {
    double s_ti = 0.0;
    double s_tf = (final_position - start_position).norm() / delta_time;

    this->initialise(s_ti, s_tf, delta_time);
  }

  
  LinearTimeLaw(double final_orientation, double start_orientation, double delta_time)
  {
    double s_ti = start_orientation;
    double s_tf = final_orientation - start_orientation;

    this->initialise(s_ti, s_tf, delta_time);
  }

  double evaluate(double t)
  {
    double s = a0 + a1*t;

    return s;
  }

  double evaluateDerivative(double t)
  {
    return a1;
  }

  std::vector<double> getParameters()
  {
    std::vector<double> parameters = {a0, a1};
    return parameters;
  }

private:

  void initialise(double s_ti, double s_tf, double delta_time)
  {
    this->a0 = s_ti;
    this->a1 = s_tf;

    this->trajectory_duration = delta_time;
  }

  double a0;
  double a1;

};

template<typename ReturnType>
class Interpolator
{
public:
  Interpolator(){}

  Interpolator(std::function<ReturnType(double)> pathFunction, double trajectory_duration)
  {
    this->pathFunction = pathFunction;
    this->trajectory_duration = trajectory_duration;
  };

  std::tuple<std::list<ReturnType>, double> discretiseTrajectory(double rate, double start_time)
  {
    double sampling_time = 1.0/rate;
    std::list<ReturnType> discretised_trajectory;
    double time_remaining;

    double time = start_time;

    while( time < trajectory_duration )
    {
      ReturnType sample = pathFunction(time);

      discretised_trajectory.push_back(sample);
      time_remaining = std::max(0.0, trajectory_duration - time );

      time+=sampling_time;
    }

    return std::make_tuple(discretised_trajectory, time_remaining);
  }

  bool isInitialised(){ return pathFunction!=nullptr?true:false; }

private: 
  std::function<ReturnType(double)> pathFunction = nullptr;
  double trajectory_duration;
};

class Controller
{
public:
  Controller(ros::NodeHandle& nh, double control_loop_rate);
  void ComputeControlAction();

private:
  void JointStateCallback(const sensor_msgs::JointStatePtr& msg);
  void DesiredPoseCallback(const geometry_msgs::PoseStamped& msg);

  geometry_msgs::Transform GetTransform(std::string parent, std::string child);
  Matrix6d ComputeGeometricalJacobian();
  Eigen::Vector3d ComputeOrientationError(Eigen::Matrix3d& tcp_orientation, Eigen::Matrix3d& desired_orientation);

  // ------------------------------------- //
  // ---------------- ROS ---------------- //
  // ------------------------------------- //
  ros::NodeHandle nh;

  // Publishers
  ros::Publisher joint_velocity_pub;

  // Subscribers
  ros::Subscriber joint_states_sub;
  ros::Subscriber desired_pose_sub;

  // Timers
  ros::Timer control_input_timer;

  // Tf listener
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener* pTfListener;

  // ------------------------------------- //
  // --------- Class parameters ---------- //
  // ------------------------------------- //
  // TF names: with respect to which TF to move,\
                 the TF to move (robot's tcp) and \
                 the TF to reach (desired pose)
  std::string tf_base_name;
  std::string tf_reference_name;
  std::string tf_tcp_name;
  Eigen::Vector3d desired_position_prev;
  Eigen::Matrix3d desired_R_prev;

  Eigen::Vector3d desired_vel_prev = Eigen::Vector3d::Zero();
  double desired_omega_prev = 0;

  Eigen::Vector3d desired_acc_prev = Eigen::Vector3d::Zero();
  double desired_omega_dot_prev = 0;

  // Robot state (joints, tcp pose and velocity)
  Vector6d joint_position;
  Vector6d joint_velocity;
  Eigen::Vector3d tcp_position;
  Eigen::Matrix3d tcp_orientation;

  // Robot parameters
  double robot_loop_rate; // TODO: parametrise

  // Controller gains
  Eigen::Vector3d Kp;
  Eigen::Vector3d Ko;

  // Time
	std::chrono::time_point<std::chrono::high_resolution_clock> prev_time;
	// ros::Time prev_time;
	double start_time = 0.0;

  std::list<Eigen::Vector3d> buffered_position;
  std::list<Eigen::Matrix3d> buffered_orientation;

  Interpolator<Eigen::Vector3d> interpolator_position;
  Interpolator<Eigen::Matrix3d> interpolator_orientation;

  // Debugging variables that will be cancelled
  ros::Publisher desired_pose_interpolated_pub;
};

#endif