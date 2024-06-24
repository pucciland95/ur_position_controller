#include "../include/controller.hpp"

/**
 * @brief Get the Transform object
 *
 * This function returns a Transform object that represents the transformation from the parent frame to the child frame.
 *
 * @param parent The name of the parent frame.
 * @param child The name of the child frame.
 * @return geometry_msgs::Transform The Transform object representing the transformation from the parent frame to the
 * child frame.
 */
geometry_msgs::Transform Controller::GetTransform(std::string parent, std::string child)
{
  geometry_msgs::Transform transform;

  try
  {
    transform = tfBuffer.lookupTransform(parent, child, ros::Time(0), ros::Duration(0.002)).transform;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    std::string error_message = "Cannot find transform from " + parent + " to " + child;
    throw std::runtime_error(error_message);
  }

  return transform;
}

/**
 * @brief Computes the geometrical Jacobian of the robot.
 *
 * This function calculates the geometrical Jacobian of the robot by obtaining the transformation
 * matrices of each link with respect to the base frame. It then uses these transformation matrices
 * to compute the position and orientation Jacobian.
 *
 * @return The geometrical Jacobian matrix.
 */
Matrix6d Controller::ComputeGeometricalJacobian()
{
  Matrix6d jacobian;

  geometry_msgs::Transform link1;
  geometry_msgs::Transform link2;
  geometry_msgs::Transform link3;
  geometry_msgs::Transform link4;
  geometry_msgs::Transform link5;
  geometry_msgs::Transform link6;

  try
  {
    link1 = this->GetTransform(this->tf_reference_name, "shoulder_link");
    link2 = this->GetTransform(this->tf_reference_name, "upper_arm_link");
    link3 = this->GetTransform(this->tf_reference_name, "forearm_link");
    link4 = this->GetTransform(this->tf_reference_name, "wrist_1_link");
    link5 = this->GetTransform(this->tf_reference_name, "wrist_2_link");
    link6 = this->GetTransform(this->tf_reference_name, "wrist_3_link");
  }
  catch(const std::exception& e)
  {
    ROS_WARN("An error occurred when computing Jacobian: %s", e.what());
    return jacobian.setZero();
  }

  Eigen::Vector3d p = this->tcp_position;

  std::vector<geometry_msgs::Transform> links = { link1, link2, link3, link4, link5, link6 };

  for (int i = 0; i < 6; i++)
  {
    geometry_msgs::Transform eachLink = links[i];
    Eigen::Vector3d p_i = Eigen::Vector3d(eachLink.translation.x, eachLink.translation.y, eachLink.translation.z);
    Eigen::Quaterniond q;
    fromMsg(eachLink.rotation, q);

    Eigen::Vector3d z_i = q.toRotationMatrix().block<3, 1>(0, 2);

    jacobian.block<3, 1>(0, i) = z_i.cross(p - p_i);
    jacobian.block<3, 1>(3, i) = z_i;
  }

  return jacobian;
}

/**
 * @brief Compute the orientation error between two orientations.
 *
 * This function computes the orientation error between a desired and current rotation matrix orientation.
 *
 * @param desired The desired orientation in quaternion form.
 * @param current The current orientation in quaternion form.
 * @return Eigen::Vector3d The orientation error.
 */
Eigen::Vector3d Controller::ComputeOrientationError(Eigen::Matrix3d& tcp_orientation,
                                                    Eigen::Matrix3d& desired_orientation)
{
  Eigen::Matrix3d delta_R = desired_orientation * tcp_orientation.transpose();
  Eigen::Quaterniond delta_q = Eigen::Quaterniond(delta_R);
  Eigen::Quaterniond delta_q_negated = Eigen::Quaterniond(-delta_q.w(), -delta_q.x(), -delta_q.y(), -delta_q.z());

  double angle_delta_q = 2 * acos(delta_q.w());
  double angle_delta_q_negated = 2 * acos(delta_q_negated.w());

  if (abs(angle_delta_q) <= abs(angle_delta_q_negated))
    delta_q = delta_q;
  else
    delta_q = delta_q_negated;

  Eigen::Vector3d orientation_error = Eigen::Vector3d(delta_q.x(), delta_q.y(), delta_q.z());

  return orientation_error;
}

// ----------------------------------------------------------------- //
// --------------------------- Callbacks --------------------------- //
// ----------------------------------------------------------------- //

void Controller::DesiredPoseCallback(const geometry_msgs::PoseStamped& msg)
{
  auto now = std::chrono::high_resolution_clock::now();
  double delta_time = std::chrono::duration_cast<std::chrono::microseconds>(now - this->prev_time).count() / 1000000.0;
  // ros::Time now = msg.header.stamp;
  // double delta_time = (now - this->prev_time).toSec();

  Eigen::Affine3d desired_pose;
  Eigen::fromMsg(msg.pose, desired_pose);
  Eigen::Vector3d desired_position = desired_pose.translation();
  Eigen::Vector3d desired_position_prev = this->desired_position_prev;
  Eigen::Vector3d desired_vel = (desired_position - this->desired_position_prev) / delta_time;

  // Orientation part
  Eigen::Matrix3d desired_R = desired_pose.rotation();
  Eigen::Matrix3d desired_R_prev = this->desired_R_prev;

  Eigen::Matrix3d delta_R = desired_R_prev.transpose() * desired_R;
  double desired_angle = Eigen::AngleAxisd(delta_R).angle();
  double desired_omega = desired_angle / delta_time;

  // Interpolator
  // Position part
  // TimeLawShrPtr pTimeLawPosition = std::make_shared<QuinticTimeLaw>(desired_position, desired_position_prev, desired_vel, desired_vel_prev, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), delta_time);
  TimeLawShrPtr pTimeLawPosition = std::make_shared<CubicalTimeLaw>(desired_position, desired_position_prev, desired_vel, desired_vel_prev, delta_time);
  auto positionPathFunction = [desired_position, desired_position_prev, pTimeLawPosition](double time) -> Eigen::Vector3d
  {
    Eigen::Vector3d diff = desired_position - desired_position_prev;
    Eigen::Vector3d p;
    if(diff.norm() == 0)
      p = desired_position;
    else
    {
      double s = pTimeLawPosition->evaluate(time);
      p = desired_position_prev + pTimeLawPosition->evaluate(time) * (diff) / (diff).norm();
    }
    return p;
  };

  this->interpolator_position = Interpolator<Eigen::Vector3d>(positionPathFunction, delta_time);
  auto position_result = this->interpolator_position.discretiseTrajectory(this->robot_loop_rate, this->start_time);
  std::list<Eigen::Vector3d> new_positions = std::get<0>(position_result);
  double position_start_time = std::get<1>(position_result);
  this->buffered_position.insert(this->buffered_position.end(), new_positions.begin(), new_positions.end());

  // Orientation part
  Eigen::Vector3d desired_axis = Eigen::AngleAxisd(delta_R).axis();

  // TimeLawShrPtr pTimeLawOrientation = std::make_shared<QuinticTimeLaw>(desired_angle, 0.0, desired_omega, desired_omega_prev, 0.0, 0.0, delta_time);
  TimeLawShrPtr pTimeLawOrientation = std::make_shared<CubicalTimeLaw>(desired_angle, 0.0, desired_omega, desired_omega_prev, delta_time);
  auto orientationPathFunction = [desired_R_prev, desired_angle, desired_axis, desired_omega, pTimeLawOrientation](double time) -> Eigen::Matrix3d
  {
    double current_angle;;
    if( desired_angle == 0 )
      current_angle = desired_angle;
    else
      current_angle = 0 + pTimeLawOrientation->evaluate(time) * (desired_angle - 0) / (desired_angle - 0);

    Eigen::AngleAxisd axis_angle = Eigen::AngleAxisd(current_angle, desired_axis);

    return desired_R_prev * axis_angle.matrix();
  };

  this->interpolator_orientation = Interpolator<Eigen::Matrix3d>(orientationPathFunction, delta_time);
  auto orientation_result = this->interpolator_orientation.discretiseTrajectory(this->robot_loop_rate, this->start_time);
  std::list<Eigen::Matrix3d> new_orientation = std::get<0>(orientation_result);
  double orientation_start_time = std::get<1>(orientation_result);
  this->buffered_orientation.insert(this->buffered_orientation.end(), new_orientation.begin(), new_orientation.end());
  
  // Updating desired position, orientation, velocity, angular velocity and time_prev
  this->desired_position_prev = desired_position;
  this->desired_R_prev = desired_R;
  this->desired_vel_prev = desired_vel;
  this->desired_omega_prev = desired_omega;
  this->prev_time = now;
  this->start_time = position_start_time;

  return;
}

void Controller::ComputeControlAction()
{
  if(this->buffered_position.empty() || this->buffered_orientation.empty())
    return;

  // Getting feedback
  try
  {
    geometry_msgs::Transform tcp_pose = this->GetTransform(this->tf_reference_name, this->tf_tcp_name);
    this->tcp_position = tf2::transformToEigen(tcp_pose).translation();
    this->tcp_orientation = tf2::transformToEigen(tcp_pose).rotation();
  }
  catch(const std::exception& e)
  {
    ROS_WARN("An error occurred when getting transform between %s and %s: %s", this->tf_reference_name.c_str(), this->tf_tcp_name.c_str(), e.what());
  }
  
  Eigen::Vector3d setpoint_position = this->buffered_position.front();
  Eigen::Matrix3d setpoint_orientation = this->buffered_orientation.front();
  this->buffered_position.pop_front();
  this->buffered_orientation.pop_front();
  
  // Debuggin stuff that needs to be removed
  geometry_msgs::Pose temp;
  temp.position.x = setpoint_position.x();
  temp.position.y = setpoint_position.y();
  temp.position.z = setpoint_position.z();
  this->desired_pose_interpolated_pub.publish(temp);

  // ---------------------------------------

  // std::cout << "buffered_position size " << this->buffered_position.size() << std::endl;

  // Computing error
  Eigen::Vector3d position_error = setpoint_position - this->tcp_position;
  Eigen::Vector3d orientation_error = ComputeOrientationError(this->tcp_orientation, setpoint_orientation);

  if( std::isnan(position_error[0]) || std::isnan(position_error[1]) || std::isnan(position_error[2]) ||
      std::isnan(orientation_error[0]) || std::isnan(orientation_error[1]) || std::isnan(orientation_error[2]) )
    return;

  if( position_error.norm() == 0.0 && orientation_error.norm() == 0 )
    return;
    
  // Computing control action
  Eigen::Vector3d control_input_p = Kp.cwiseProduct(position_error);
  Eigen::Vector3d control_input_o = Ko.cwiseProduct(orientation_error);
  Vector6d control_input;
  control_input.block<3, 1>(0, 0) = control_input_p;
  control_input.block<3, 1>(3, 0) = control_input_o;

  // Computing Jacobian
  Matrix6d Jg = this->ComputeGeometricalJacobian();

  if(Jg.determinant() == 0 || Jg.isZero())
    return;

  Vector6d joint_velocities = Jg.inverse() * control_input;

  // Changing datatype
  std_msgs::Float64MultiArray q_dot;
  q_dot.data.reserve(6);
  q_dot.data.push_back(joint_velocities[0]);
  q_dot.data.push_back(joint_velocities[1]);
  q_dot.data.push_back(joint_velocities[2]);
  q_dot.data.push_back(joint_velocities[3]);
  q_dot.data.push_back(joint_velocities[4]);
  q_dot.data.push_back(joint_velocities[5]);

  // Uncomment to die
  joint_velocity_pub.publish(q_dot);
}

// ----------------------------------------------------------------- //
// -------------------------- Constructor -------------------------- //
// ----------------------------------------------------------------- //

Controller::Controller(ros::NodeHandle& nh, double control_loop_rate)
{
  this->nh = nh;

  // Loop rate
  this->robot_loop_rate = control_loop_rate;  // Robot update loop (Hz) // TODO: parametrise

  // Getting parameters

  // Publisher
  std::string topic_input_commands_name = "/joint_group_vel_controller/command";  // TODO: parametrise
  this->joint_velocity_pub = nh.advertise<std_msgs::Float64MultiArray>(topic_input_commands_name, 1);
  this->desired_pose_interpolated_pub = nh.advertise<geometry_msgs::Pose>("desired_pose_interpolated", 1);

  // Subscriber
  this->desired_pose_sub = nh.subscribe("desired_pose", 1000, &Controller::DesiredPoseCallback, this);

  // Getting first msg for desired pose
  boost::shared_ptr<geometry_msgs::PoseStamped const> sharedPtr =
      ros::topic::waitForMessage<geometry_msgs::PoseStamped>("desired_pose", ros::Duration(5));
  if (sharedPtr == NULL)
    ROS_INFO("No desired_pose messages received");
  else
  {
    Eigen::Affine3d desired_pose;
    Eigen::fromMsg(sharedPtr->pose, desired_pose);
    this->desired_position_prev = desired_pose.translation();
    this->desired_R_prev = desired_pose.rotation();
  }

  // Creating the TF listener
  this->pTfListener = new tf2_ros::TransformListener(this->tfBuffer);

  // Getting the controller gains from param file
  this->Kp = Eigen::Vector3d(2.0, 2.0, 2.0);  // TODO: Parametrise
  this->Ko = Eigen::Vector3d(1.0, 1.0, 1.0);  // TODO: Parametrise

  this->tf_reference_name = "tabletop"; // TODO: Parametrise -> Set it accordingly to the frame_id of the message coming from topic /desired_pose?
  this->tf_tcp_name = "flange";         // TODO: Parametrise

  // Setting time
  this->prev_time = std::chrono::high_resolution_clock::now();
  // this->prev_time = ros::Time::now();

  this->buffered_position = std::list<Eigen::Vector3d>();
  this->buffered_orientation = std::list<Eigen::Matrix3d>();
}
