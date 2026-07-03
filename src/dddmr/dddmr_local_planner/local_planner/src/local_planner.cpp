/*
* BSD 3-Clause License

* Copyright (c) 2024, DDDMobileRobot

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:

* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.

* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.

* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.

* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <local_planner/local_planner.h>
#include <algorithm>
#include <limits>
#include <numeric>
#include <sstream>

namespace local_planner {

namespace
{

std::string formatRejectedTrajectoryReport(
  const std::map<std::string, std::vector<base_trajectory::Trajectory>>& rejected_trajectories,
  std::size_t total_trajectories);

}  // namespace

Local_Planner::Local_Planner(const std::string& name): Node(name)
{
  name_ = name;
  clock_ = this->get_clock();
  got_odom_ = false;
  last_valid_prune_plan_ = clock_->now();
  mppi_rng_ = std::mt19937(std::random_device{}());
  mppi_nominal_initialized_ = false;
}

void Local_Planner::initial(
      const std::shared_ptr<perception_3d::Perception3D_ROS>& perception_3d,
      const std::shared_ptr<mpc_critics::MPC_Critics_ROS>& mpc_critics,
      const std::shared_ptr<trajectory_generators::Trajectory_Generators_ROS>& trajectory_generators){

  declare_parameter("odom_topic", rclcpp::ParameterValue("odom"));
  this->get_parameter("odom_topic", odom_topic_);
  RCLCPP_INFO(this->get_logger(), "odom_topic: %s", odom_topic_.c_str());

  declare_parameter("odom_topic_qos", rclcpp::ParameterValue("reliable"));
  this->get_parameter("odom_topic_qos", odom_topic_qos_);
  RCLCPP_INFO(this->get_logger(), "odom_topic_qos: %s", odom_topic_qos_.c_str());

  declare_parameter("ackermann_drive_topic", rclcpp::ParameterValue("ackermann_drive"));
  this->get_parameter("ackermann_drive_topic", ackermann_topic_);
  RCLCPP_INFO(this->get_logger(), "ackermann_drive_topic: %s", ackermann_topic_.c_str());

  declare_parameter("compute_best_trajectory_in_odomCb", rclcpp::ParameterValue(false));
  this->get_parameter("compute_best_trajectory_in_odomCb", compute_best_trajectory_in_odomCb_);
  RCLCPP_INFO(this->get_logger(), "compute_best_trajectory_in_odomCb: %d", compute_best_trajectory_in_odomCb_);

  declare_parameter("forward_prune", rclcpp::ParameterValue(1.0));
  this->get_parameter("forward_prune", forward_prune_);
  RCLCPP_INFO(this->get_logger(), "forward_prune: %.2f", forward_prune_);

  declare_parameter("backward_prune", rclcpp::ParameterValue(0.5));
  this->get_parameter("backward_prune", backward_prune_);
  RCLCPP_INFO(this->get_logger(), "backward_prune: %.2f", backward_prune_);

  declare_parameter("heading_tracking_distance", rclcpp::ParameterValue(0.5));
  this->get_parameter("heading_tracking_distance", heading_tracking_distance_);
  RCLCPP_INFO(this->get_logger(), "heading_tracking_distance: %.2f", heading_tracking_distance_);

  declare_parameter("heading_align_angle", rclcpp::ParameterValue(0.5));
  this->get_parameter("heading_align_angle", heading_align_angle_);
  RCLCPP_INFO(this->get_logger(), "heading_align_angle: %.2f", heading_align_angle_);

  declare_parameter("prune_plan_max_deviation", rclcpp::ParameterValue(1.0));
  this->get_parameter("prune_plan_max_deviation", prune_plan_max_deviation_);
  RCLCPP_INFO(this->get_logger(), "prune_plan_max_deviation: %.2f", prune_plan_max_deviation_);

  declare_parameter("prune_plane_timeout", rclcpp::ParameterValue(3.0));
  this->get_parameter("prune_plane_timeout", prune_plane_timeout_);
  RCLCPP_INFO(this->get_logger(), "prune_plane_timeout: %.2f", prune_plane_timeout_);

  declare_parameter("xy_goal_tolerance", rclcpp::ParameterValue(0.3));
  this->get_parameter("xy_goal_tolerance", xy_goal_tolerance_);
  RCLCPP_INFO(this->get_logger(), "xy_goal_tolerance: %.2f", xy_goal_tolerance_);

  declare_parameter("yaw_goal_tolerance", rclcpp::ParameterValue(0.3));
  this->get_parameter("yaw_goal_tolerance", yaw_goal_tolerance_);
  RCLCPP_INFO(this->get_logger(), "yaw_goal_tolerance: %.2f", yaw_goal_tolerance_);

  declare_parameter("controller_frequency", rclcpp::ParameterValue(10.0));
  this->get_parameter("controller_frequency", controller_frequency_);
  RCLCPP_INFO(this->get_logger(), "controller_frequency: %.2f", controller_frequency_);

  declare_parameter("mppi.batch_size", rclcpp::ParameterValue(256));
  this->get_parameter("mppi.batch_size", mppi_batch_size_);
  RCLCPP_INFO(this->get_logger(), "mppi.batch_size: %d", mppi_batch_size_);

  declare_parameter("mppi.iterations", rclcpp::ParameterValue(2));
  this->get_parameter("mppi.iterations", mppi_iterations_);
  RCLCPP_INFO(this->get_logger(), "mppi.iterations: %d", mppi_iterations_);

  declare_parameter("mppi.horizon_steps", rclcpp::ParameterValue(15));
  this->get_parameter("mppi.horizon_steps", mppi_horizon_steps_);
  RCLCPP_INFO(this->get_logger(), "mppi.horizon_steps: %d", mppi_horizon_steps_);

  declare_parameter("mppi.dt", rclcpp::ParameterValue(0.08));
  this->get_parameter("mppi.dt", mppi_dt_);
  RCLCPP_INFO(this->get_logger(), "mppi.dt: %.2f", mppi_dt_);

  declare_parameter("mppi.lambda", rclcpp::ParameterValue(0.35));
  this->get_parameter("mppi.lambda", mppi_lambda_);
  RCLCPP_INFO(this->get_logger(), "mppi.lambda: %.2f", mppi_lambda_);

  declare_parameter("mppi.noise_vx", rclcpp::ParameterValue(0.10));
  this->get_parameter("mppi.noise_vx", mppi_noise_vx_);
  RCLCPP_INFO(this->get_logger(), "mppi.noise_vx: %.2f", mppi_noise_vx_);

  declare_parameter("mppi.noise_wz", rclcpp::ParameterValue(0.25));
  this->get_parameter("mppi.noise_wz", mppi_noise_wz_);
  RCLCPP_INFO(this->get_logger(), "mppi.noise_wz: %.2f", mppi_noise_wz_);

  declare_parameter("mppi.max_vel_x", rclcpp::ParameterValue(0.32));
  this->get_parameter("mppi.max_vel_x", mppi_max_vel_x_);
  RCLCPP_INFO(this->get_logger(), "mppi.max_vel_x: %.2f", mppi_max_vel_x_);

  declare_parameter("mppi.min_vel_x", rclcpp::ParameterValue(0.03));
  this->get_parameter("mppi.min_vel_x", mppi_min_vel_x_);
  RCLCPP_INFO(this->get_logger(), "mppi.min_vel_x: %.2f", mppi_min_vel_x_);

  declare_parameter("mppi.max_vel_theta", rclcpp::ParameterValue(0.75));
  this->get_parameter("mppi.max_vel_theta", mppi_max_vel_theta_);
  RCLCPP_INFO(this->get_logger(), "mppi.max_vel_theta: %.2f", mppi_max_vel_theta_);

  declare_parameter("mppi.acc_lim_x", rclcpp::ParameterValue(0.6));
  this->get_parameter("mppi.acc_lim_x", mppi_acc_lim_x_);
  RCLCPP_INFO(this->get_logger(), "mppi.acc_lim_x: %.2f", mppi_acc_lim_x_);

  declare_parameter("mppi.acc_lim_theta", rclcpp::ParameterValue(1.2));
  this->get_parameter("mppi.acc_lim_theta", mppi_acc_lim_theta_);
  RCLCPP_INFO(this->get_logger(), "mppi.acc_lim_theta: %.2f", mppi_acc_lim_theta_);

  declare_parameter("mppi.weight_path", rclcpp::ParameterValue(4.0));
  this->get_parameter("mppi.weight_path", mppi_weight_path_);
  RCLCPP_INFO(this->get_logger(), "mppi.weight_path: %.2f", mppi_weight_path_);

  declare_parameter("mppi.weight_goal", rclcpp::ParameterValue(3.0));
  this->get_parameter("mppi.weight_goal", mppi_weight_goal_);
  RCLCPP_INFO(this->get_logger(), "mppi.weight_goal: %.2f", mppi_weight_goal_);

  declare_parameter("mppi.weight_heading", rclcpp::ParameterValue(0.8));
  this->get_parameter("mppi.weight_heading", mppi_weight_heading_);
  RCLCPP_INFO(this->get_logger(), "mppi.weight_heading: %.2f", mppi_weight_heading_);

  declare_parameter("mppi.weight_obstacle", rclcpp::ParameterValue(2.5));
  this->get_parameter("mppi.weight_obstacle", mppi_weight_obstacle_);
  RCLCPP_INFO(this->get_logger(), "mppi.weight_obstacle: %.2f", mppi_weight_obstacle_);

  declare_parameter("mppi.weight_smooth", rclcpp::ParameterValue(0.25));
  this->get_parameter("mppi.weight_smooth", mppi_weight_smooth_);
  RCLCPP_INFO(this->get_logger(), "mppi.weight_smooth: %.2f", mppi_weight_smooth_);

  declare_parameter("mppi.weight_effort", rclcpp::ParameterValue(0.05));
  this->get_parameter("mppi.weight_effort", mppi_weight_effort_);
  RCLCPP_INFO(this->get_logger(), "mppi.weight_effort: %.2f", mppi_weight_effort_);

  declare_parameter("mppi.forward_reward", rclcpp::ParameterValue(0.6));
  this->get_parameter("mppi.forward_reward", mppi_forward_reward_);
  RCLCPP_INFO(this->get_logger(), "mppi.forward_reward: %.2f", mppi_forward_reward_);

  declare_parameter("mppi.lookahead_distance", rclcpp::ParameterValue(1.2));
  this->get_parameter("mppi.lookahead_distance", mppi_lookahead_distance_);
  RCLCPP_INFO(this->get_logger(), "mppi.lookahead_distance: %.2f", mppi_lookahead_distance_);

  declare_parameter("mppi.target_heading_distance", rclcpp::ParameterValue(0.8));
  this->get_parameter("mppi.target_heading_distance", mppi_target_heading_distance_);
  RCLCPP_INFO(this->get_logger(), "mppi.target_heading_distance: %.2f", mppi_target_heading_distance_);

  declare_parameter("mppi.obstacle_margin", rclcpp::ParameterValue(0.10));
  this->get_parameter("mppi.obstacle_margin", mppi_obstacle_margin_);
  RCLCPP_INFO(this->get_logger(), "mppi.obstacle_margin: %.2f", mppi_obstacle_margin_);


  //@Initialize transform listener and broadcaster
  tf_listener_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  tf2Buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface(),
    tf_listener_group_);
  tf2Buffer_->setCreateTimerInterface(timer_interface);
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer_);

  perception_3d_ros_ = perception_3d;
  mpc_critics_ros_ = mpc_critics;
  trajectory_generators_ros_ = trajectory_generators;

  robot_frame_ = perception_3d_ros_->getGlobalUtils()->getRobotFrame();
  global_frame_ = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  parseCuboid(); //after robot_frame is got
  
  pub_robot_cuboid_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("robot_cuboid", 1);  
  pub_aggregate_observation_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("aggregated_pc", 1);  
  pub_prune_plan_ = this->create_publisher<nav_msgs::msg::Path>("prune_plan", 1);
  pub_accepted_trajectory_pose_array_ = this->create_publisher<geometry_msgs::msg::PoseArray>("accepted_trajectory", 1);
  pub_best_trajectory_pose_ = this->create_publisher<geometry_msgs::msg::PoseArray>("best_trajectory", 2);
  pub_trajectory_pose_array_ = this->create_publisher<geometry_msgs::msg::PoseArray>("trajectory", 2);
  //pub_pc_normal_ = pnh_.advertise<visualization_msgs::MarkerArray>("normal_marker", 2, true);
  //pub_trajectory_cuboids_ = pnh_.advertise<sensor_msgs::PointCloud2>("trajectory_cuboids", 2, true);

  cbs_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cbs_group_;
  
  if(odom_topic_qos_=="reliable" || odom_topic_qos_=="Reliable"){
    odom_ros_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().reliable(),
      std::bind(&Local_Planner::cbOdom, this, std::placeholders::_1), sub_options);
  }
  else{
    odom_ros_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort(),
      std::bind(&Local_Planner::cbOdom, this, std::placeholders::_1), sub_options);
  }
  
  ackermann_drive_ros_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      ackermann_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort(),
      std::bind(&Local_Planner::cbAckermannDrive, this, std::placeholders::_1), sub_options);

  //@Initial pcl ptr
  pcl_global_plan_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  kdtree_global_plan_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
}

Local_Planner::~Local_Planner(){

  perception_3d_ros_.reset();
  mpc_critics_ros_.reset();
  trajectory_generators_ros_.reset();
  trajectories_.reset();
  tf2Buffer_.reset();

}

std::string Local_Planner::getControlFrame(){
  return perception_3d_ros_->getGlobalUtils()->getRobotFrame();
};

void Local_Planner::parseCuboid(){
  marker_edge_.header.frame_id = perception_3d_ros_->getGlobalUtils()->getRobotFrame();;
  marker_edge_.header.stamp = clock_->now();
  marker_edge_.action = visualization_msgs::msg::Marker::ADD;
  marker_edge_.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_edge_.pose.orientation.w = 1.0;
  marker_edge_.ns = "edges";
  marker_edge_.id = 3; marker_edge_.scale.x = 0.03;
  marker_edge_.color.r = 0.9; marker_edge_.color.g = 1; marker_edge_.color.b = 0; marker_edge_.color.a = 0.8;
  //@ parse cuboid, currently the cuboid in local planner is just for visualization
  RCLCPP_INFO(this->get_logger().get_child(name_), "Start to parse cuboid.");
  std::vector<std::string> cuboid_vertex_queue = {"cuboid.flb", "cuboid.frb", "cuboid.flt", "cuboid.frt", "cuboid.blb", "cuboid.brb", "cuboid.blt", "cuboid.brt"};
  std::map<std::string, std::vector<double>> cuboid_vertex_parameter_map;
  base_cuboid_vertices_.clear();

  for(auto it=cuboid_vertex_queue.begin(); it!=cuboid_vertex_queue.end();it++){
    std::vector<double> p;
    geometry_msgs::msg::Point pt;
    pcl::PointXYZ pcl_pt;
    this->declare_parameter(*it, rclcpp::PARAMETER_DOUBLE_ARRAY);
    rclcpp::Parameter cuboid_param= this->get_parameter(*it);
    p = cuboid_param.as_double_array();
    pt.x = p[0];pt.y = p[1];pt.z = p[2];
    pcl_pt.x = p[0];pcl_pt.y = p[1];pcl_pt.z = p[2];
    marker_edge_.points.push_back(pt);
    base_cuboid_vertices_.push_back(pcl_pt);
    cuboid_vertex_parameter_map[*it] = p;
  }
  RCLCPP_INFO(this->get_logger().get_child(name_), "Cuboid vertex are loaded, start to connect edges.");
  std::vector<std::string> cuboid_vertex_connect = {"cuboid.flb", "cuboid.blb", "cuboid.flt", "cuboid.blt", "cuboid.frb", "cuboid.brb", "cuboid.frt", "cuboid.brt",
                                                      "cuboid.flt", "cuboid.flb", "cuboid.frt", "cuboid.frb", "cuboid.blt", "cuboid.blb", "cuboid.brt", "cuboid.brb"};
  for(auto it=cuboid_vertex_connect.begin(); it!=cuboid_vertex_connect.end();it++){
    auto p = cuboid_vertex_parameter_map[*it];
    geometry_msgs::msg::Point pt;
    pt.x = p[0];pt.y = p[1];pt.z = p[2];
    marker_edge_.points.push_back(pt);
  }
}

void Local_Planner::cbOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_state_ = *msg;
  updateGlobalPose();
  if(compute_best_trajectory_in_odomCb_){
    base_trajectory::Trajectory best_traj;
    computeVelocityCommand("differential_drive_simple", best_traj);
  }
  got_odom_ = true;
}

void Local_Planner::cbAckermannDrive(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg){
  ackermann_drive_state_ = *msg;
  updateGlobalPose();
}

double Local_Planner::getShortestAngleFromPose2RobotHeading(tf2::Transform m_pose){

  //@Transform trans_gbl2b_ to tf2; Get baselink to global, so that we later can get base_link2gbl * gbl2lastpose
  tf2::Stamped<tf2::Transform> tf2_trans_gbl2b;
  tf2::fromMsg(trans_gbl2b_, tf2_trans_gbl2b);
  auto tf2_trans_gbl2b_inverse = tf2_trans_gbl2b.inverse();
  //@Get baselink to last pose
  tf2::Transform tf2_baselink2prunelastpose;
  tf2_baselink2prunelastpose.mult(tf2_trans_gbl2b_inverse, m_pose);
  //@Get RPY
  tf2::Matrix3x3 m(tf2_baselink2prunelastpose.getRotation());
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  //@Although the test shows that yaw is already the shortest, we will use shortest_angular_distance anyway.
  yaw = angles::shortest_angular_distance(0.0, yaw);
  
  return yaw;

}

bool Local_Planner::isInitialHeadingAligned(){

  const double yaw = getPathHeadingDeviation();
  RCLCPP_DEBUG(this->get_logger().get_child(name_), "Heading difference from the prune plan starting at %.2f is %.2f", heading_tracking_distance_, yaw);

  if(fabs(yaw) < heading_align_angle_)
    return true;
  else
    return false;
}

double Local_Planner::getPathHeadingDeviation(){

  prunePlan(heading_tracking_distance_, 0.0);
  if(prune_plan_.poses.size()<3){
    RCLCPP_WARN_THROTTLE(this->get_logger().get_child(name_), *clock_, 5000, "Prune plan is too short when checking initial heading.");
    return 0.0;
  }
  
  //@ Get first/last pose from prune plan
  geometry_msgs::msg::PoseStamped first_pose = prune_plan_.poses.front();
  geometry_msgs::msg::PoseStamped last_pose = prune_plan_.poses.back();

  //@ Generate a pose pointing from first pose to last pose
  double vx,vy;
  vx = last_pose.pose.position.x - first_pose.pose.position.x;
  vy = last_pose.pose.position.y - first_pose.pose.position.y;

  double yaw_to_path = atan2(vy, vx);
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_to_path);

  tf2::Transform tf2_prune_pointing_pose;
  //@Transform last pose to tf2 type
  //tf2::Quaternion(q.getX(), q.getY(), q.getZ(), q.getW())
  tf2_prune_pointing_pose.setRotation(q);
  tf2_prune_pointing_pose.setOrigin(tf2::Vector3(first_pose.pose.position.x, first_pose.pose.position.y, first_pose.pose.position.z));

  //@Update the value to critics that allow the robot to turn by shortest angle
  double yaw = getShortestAngleFromPose2RobotHeading(tf2_prune_pointing_pose);
  mpc_critics_ros_->getSharedDataPtr()->heading_deviation_ = yaw;
  return yaw;
}

bool Local_Planner::isGoalHeadingAligned(){

  if(global_plan_.empty()){
    return false;
  }

  geometry_msgs::msg::PoseStamped final_pose;
  final_pose = global_plan_.back();

  geometry_msgs::msg::TransformStamped final_pose_ts;
  final_pose_ts.header = final_pose.header;
  final_pose_ts.transform.translation.x = final_pose.pose.position.x;
  final_pose_ts.transform.translation.y = final_pose.pose.position.y;
  final_pose_ts.transform.translation.z = final_pose.pose.position.z;
  final_pose_ts.transform.rotation.x = final_pose.pose.orientation.x;
  final_pose_ts.transform.rotation.y = final_pose.pose.orientation.y;
  final_pose_ts.transform.rotation.z = final_pose.pose.orientation.z;
  final_pose_ts.transform.rotation.w = final_pose.pose.orientation.w;
  tf2::Stamped<tf2::Transform> tf2_trans_gbl2goal;
  tf2::fromMsg(final_pose_ts, tf2_trans_gbl2goal);  

  //@Update the value to critics that allow the robot to turn by shortest angle
  double yaw = getShortestAngleFromPose2RobotHeading(tf2_trans_gbl2goal);
  mpc_critics_ros_->getSharedDataPtr()->heading_deviation_ = yaw;
  
  RCLCPP_DEBUG(this->get_logger().get_child(name_), "Heading difference to goal is %.2f", yaw);

  if(fabs(yaw) < yaw_goal_tolerance_)
    return true;
  else
    return false;
}

bool Local_Planner::isGoalReached(){
  if(global_plan_.empty()){
    return false;
  }
  geometry_msgs::msg::PoseStamped final_pose;
  final_pose = global_plan_.back();
  double dx = trans_gbl2b_.transform.translation.x - final_pose.pose.position.x;
  double dy = trans_gbl2b_.transform.translation.y - final_pose.pose.position.y;
  double dz = trans_gbl2b_.transform.translation.z - final_pose.pose.position.z;
  double distance = sqrt(dx*dx + dy*dy + dz*dz);
  if(xy_goal_tolerance_>distance)
    return true;
  else
    return false;
}

double Local_Planner::clampValue(double value, double min_value, double max_value) const{
  return std::max(min_value, std::min(max_value, value));
}

double Local_Planner::clampVelocityByAcceleration(
  double desired,
  double previous,
  double acc_limit,
  double dt,
  double min_value,
  double max_value) const
{
  const double min_reachable = previous - acc_limit * dt;
  const double max_reachable = previous + acc_limit * dt;
  return clampValue(desired, std::max(min_value, min_reachable), std::min(max_value, max_reachable));
}

void Local_Planner::initializeMPPINominalControl(double current_linear_speed, double current_angular_speed){
  if(mppi_nominal_initialized_ &&
     static_cast<int>(mppi_nominal_vx_.size()) == mppi_horizon_steps_ &&
     static_cast<int>(mppi_nominal_wz_.size()) == mppi_horizon_steps_){
    return;
  }

  mppi_nominal_vx_.assign(
    mppi_horizon_steps_,
    clampValue(current_linear_speed, 0.0, 0.6));
  mppi_nominal_wz_.assign(
    mppi_horizon_steps_,
    clampValue(current_angular_speed, -1.0, 1.0));
  mppi_nominal_initialized_ = true;
}

void Local_Planner::shiftMPPINominalControl(){
  if(!mppi_nominal_initialized_ || mppi_nominal_vx_.empty() || mppi_nominal_wz_.empty()){
    return;
  }
  std::rotate(mppi_nominal_vx_.begin(), mppi_nominal_vx_.begin() + 1, mppi_nominal_vx_.end());
  std::rotate(mppi_nominal_wz_.begin(), mppi_nominal_wz_.begin() + 1, mppi_nominal_wz_.end());
  mppi_nominal_vx_.back() = mppi_nominal_vx_[mppi_nominal_vx_.size() - 2];
  mppi_nominal_wz_.back() = mppi_nominal_wz_[mppi_nominal_wz_.size() - 2];
}

bool Local_Planner::getLookaheadTarget(
  double lookahead_distance,
  geometry_msgs::msg::PoseStamped& target_pose,
  double& target_yaw)
{
  if(prune_plan_.poses.size() < 2){
    return false;
  }

  target_pose = prune_plan_.poses.front();
  double accumulated_distance = 0.0;
  std::size_t target_index = 0;
  while(target_index + 1 < prune_plan_.poses.size() && accumulated_distance < lookahead_distance){
    accumulated_distance += getDistanceBTWPoseStamp(
      prune_plan_.poses[target_index],
      prune_plan_.poses[target_index + 1]);
    ++target_index;
  }
  target_pose = prune_plan_.poses[target_index];

  std::size_t heading_index = target_index;
  double heading_distance = 0.0;
  while(heading_index + 1 < prune_plan_.poses.size() &&
        heading_distance < mppi_target_heading_distance_){
    heading_distance += getDistanceBTWPoseStamp(
      prune_plan_.poses[heading_index],
      prune_plan_.poses[heading_index + 1]);
    ++heading_index;
  }

  if(heading_index != target_index){
    const double dx =
      prune_plan_.poses[heading_index].pose.position.x - target_pose.pose.position.x;
    const double dy =
      prune_plan_.poses[heading_index].pose.position.y - target_pose.pose.position.y;
    if(std::hypot(dx, dy) > 1e-4){
      target_yaw = std::atan2(dy, dx);
      return true;
    }
  }

  tf2::Quaternion q;
  tf2::convert(target_pose.pose.orientation, q);
  double roll, pitch;
  tf2::Matrix3x3(q).getRPY(roll, pitch, target_yaw);
  return true;
}

pcl::PointCloud<pcl::PointXYZ> Local_Planner::buildCuboidForPose(
  const geometry_msgs::msg::PoseStamped& pose) const
{
  pcl::PointCloud<pcl::PointXYZ> cuboid;
  cuboid.points.reserve(base_cuboid_vertices_.size());

  tf2::Transform transform;
  tf2::fromMsg(pose.pose, transform);
  for(const auto& vertex : base_cuboid_vertices_){
    tf2::Vector3 point(vertex.x, vertex.y, vertex.z);
    point = transform * point;
    pcl::PointXYZ pcl_point;
    pcl_point.x = point.x();
    pcl_point.y = point.y();
    pcl_point.z = point.z();
    cuboid.points.push_back(pcl_point);
  }
  return cuboid;
}

base_trajectory::cuboid_min_max_t Local_Planner::getCuboidMinMax(
  const pcl::PointCloud<pcl::PointXYZ>& cuboid) const
{
  pcl::PointXYZ min_point;
  pcl::PointXYZ max_point;
  min_point.x = min_point.y = min_point.z = std::numeric_limits<float>::max();
  max_point.x = max_point.y = max_point.z = std::numeric_limits<float>::lowest();

  for(const auto& point : cuboid.points){
    min_point.x = std::min(min_point.x, point.x);
    min_point.y = std::min(min_point.y, point.y);
    min_point.z = std::min(min_point.z, point.z);
    max_point.x = std::max(max_point.x, point.x);
    max_point.y = std::max(max_point.y, point.y);
    max_point.z = std::max(max_point.z, point.z);
  }
  return {min_point, max_point};
}

base_trajectory::Trajectory Local_Planner::rolloutMPPI(
  const std::vector<double>& vx_sequence,
  const std::vector<double>& wz_sequence,
  double dt)
{
  base_trajectory::Trajectory traj;
  if(vx_sequence.empty() || wz_sequence.empty()){
    return traj;
  }

  traj.xv_ = vx_sequence.front();
  traj.yv_ = 0.0;
  traj.thetav_ = wz_sequence.front();
  traj.time_delta_ = dt;
  traj.cost_ = 0.0;

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = global_frame_;
  pose.header.stamp = clock_->now();
  pose.pose.position.x = trans_gbl2b_.transform.translation.x;
  pose.pose.position.y = trans_gbl2b_.transform.translation.y;
  pose.pose.position.z = trans_gbl2b_.transform.translation.z;

  tf2::Quaternion q(
    trans_gbl2b_.transform.rotation.x,
    trans_gbl2b_.transform.rotation.y,
    trans_gbl2b_.transform.rotation.z,
    trans_gbl2b_.transform.rotation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  for(std::size_t step = 0; step < std::min(vx_sequence.size(), wz_sequence.size()); ++step){
    yaw += wz_sequence[step] * dt;
    pose.pose.position.x += std::cos(yaw) * vx_sequence[step] * dt;
    pose.pose.position.y += std::sin(yaw) * vx_sequence[step] * dt;
    tf2::Quaternion step_quat;
    step_quat.setRPY(0.0, 0.0, yaw);
    pose.pose.orientation = tf2::toMsg(step_quat);
    const auto cuboid = buildCuboidForPose(pose);
    traj.addPoint(pose, cuboid, getCuboidMinMax(cuboid));
  }

  return traj;
}

double Local_Planner::scoreMPPI(
  const base_trajectory::Trajectory& traj,
  const std::vector<double>& vx_sequence,
  const std::vector<double>& wz_sequence,
  const pcl::KdTreeFLANN<pcl::PointXYZI>& prune_plan_kdtree,
  const pcl::KdTreeFLANN<pcl::PointXYZI>& obstacle_kdtree,
  bool has_obstacles,
  const geometry_msgs::msg::PoseStamped& target_pose,
  double target_yaw) const
{
  if(traj.getPointsSize() == 0){
    return std::numeric_limits<double>::infinity();
  }

  const double inscribed_radius = perception_3d_ros_->getGlobalUtils()->getInscribedRadius();
  const double obstacle_lethal_radius = inscribed_radius + mppi_obstacle_margin_;
  double path_cost = 0.0;
  double obstacle_cost = 0.0;
  double smooth_cost = 0.0;
  double effort_cost = 0.0;
  double forward_progress = 0.0;
  double previous_vx = robot_state_.twist.twist.linear.x;
  double previous_wz = robot_state_.twist.twist.angular.z;

  for(unsigned int index = 0; index < traj.getPointsSize(); ++index){
    const auto pcl_point = traj.getPCLPoint(index);
    std::vector<int> nearest_indices(1);
    std::vector<float> nearest_distance_sq(1);
    if(prune_plan_kdtree.nearestKSearch(pcl_point, 1, nearest_indices, nearest_distance_sq) > 0){
      path_cost += std::sqrt(nearest_distance_sq[0]);
    } else {
      path_cost += 5.0;
    }

    if(has_obstacles){
      std::vector<int> obstacle_indices(1);
      std::vector<float> obstacle_distance_sq(1);
      if(obstacle_kdtree.nearestKSearch(pcl_point, 1, obstacle_indices, obstacle_distance_sq) > 0){
        const double obstacle_distance = std::sqrt(obstacle_distance_sq[0]);
        if(obstacle_distance <= obstacle_lethal_radius){
          return std::numeric_limits<double>::infinity();
        }
        obstacle_cost += 1.0 / std::max(0.05, obstacle_distance - inscribed_radius);
      }
    }

    const double vx = vx_sequence[index];
    const double wz = wz_sequence[index];
    smooth_cost += std::fabs(vx - previous_vx) + 0.5 * std::fabs(wz - previous_wz);
    effort_cost += std::fabs(vx) + 0.2 * std::fabs(wz);
    forward_progress += vx;
    previous_vx = vx;
    previous_wz = wz;
  }

  path_cost /= traj.getPointsSize();
  obstacle_cost /= std::max(1u, traj.getPointsSize());
  smooth_cost /= std::max(1u, traj.getPointsSize());
  effort_cost /= std::max(1u, traj.getPointsSize());
  forward_progress /= std::max(1u, traj.getPointsSize());

  const auto terminal_pose = traj.getPoint(traj.getPointsSize() - 1);
  const double dx = terminal_pose.pose.position.x - target_pose.pose.position.x;
  const double dy = terminal_pose.pose.position.y - target_pose.pose.position.y;
  const double goal_cost = std::hypot(dx, dy);

  tf2::Quaternion terminal_quat;
  tf2::convert(terminal_pose.pose.orientation, terminal_quat);
  double terminal_roll, terminal_pitch, terminal_yaw;
  tf2::Matrix3x3(terminal_quat).getRPY(terminal_roll, terminal_pitch, terminal_yaw);
  const double heading_cost = std::fabs(angles::shortest_angular_distance(terminal_yaw, target_yaw));

  return
    mppi_weight_path_ * path_cost +
    mppi_weight_goal_ * goal_cost +
    mppi_weight_heading_ * heading_cost +
    mppi_weight_obstacle_ * obstacle_cost +
    mppi_weight_smooth_ * smooth_cost +
    mppi_weight_effort_ * effort_cost -
    mppi_forward_reward_ * forward_progress;
}

dddmr_sys_core::PlannerState Local_Planner::computeVelocityCommandMPPI(
  base_trajectory::Trajectory& best_traj)
{
  if(!got_odom_){
    RCLCPP_ERROR(this->get_logger().get_child(name_), "Odom is not received.");
    return dddmr_sys_core::TF_FAIL;
  }

  if(!perception_3d_ros_->getStackedPerception()->isSensorOK()){
    RCLCPP_ERROR(this->get_logger().get_child(name_), "Perception 3D is not ok.");
    return dddmr_sys_core::PERCEPTION_MALFUNCTION;
  }

  control_loop_time_ = clock_->now();

  std::unique_lock<perception_3d::StackedPerception::mutex_t> pct_lock(
    *(perception_3d_ros_->getStackedPerception()->getMutex()));

  perception_3d_ros_->getStackedPerception()->aggregateObservations();
  prunePlan(forward_prune_, backward_prune_);

  sensor_msgs::msg::PointCloud2 ros2_aggregate_onservation;
  pcl::toROSMsg(
    *(perception_3d_ros_->getSharedDataPtr()->aggregate_observation_),
    ros2_aggregate_onservation);
  pub_aggregate_observation_->publish(ros2_aggregate_onservation);

  if((clock_->now()-trans_gbl2b_.header.stamp).seconds() > 2.0){
    RCLCPP_ERROR(
      this->get_logger().get_child(name_),
      "TF out of date in local planner, the local planner wont go further.");
    return dddmr_sys_core::TF_FAIL;
  }

  perception_3d_ros_->getSharedDataPtr()->pcl_prune_plan_ = pcl_prune_plan_;

  if((clock_->now()-last_valid_prune_plan_).seconds() >= prune_plane_timeout_){
    RCLCPP_FATAL(
      this->get_logger().get_child(name_),
      "Deviate global plan too much, computeVelocityCommandMPPI() returns false.");
    return dddmr_sys_core::PRUNE_PLAN_FAIL;
  }

  if(prune_plan_.poses.size() < 2 || pcl_prune_plan_.points.empty()){
    RCLCPP_WARN_THROTTLE(
      this->get_logger().get_child(name_),
      *clock_,
      5000,
      "Prune plan is too short for MPPI.");
    return dddmr_sys_core::ALL_TRAJECTORIES_FAIL;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr prune_plan_cloud(
    new pcl::PointCloud<pcl::PointXYZI>(pcl_prune_plan_));
  pcl::KdTreeFLANN<pcl::PointXYZI> prune_plan_kdtree;
  prune_plan_kdtree.setInputCloud(prune_plan_cloud);

  const auto obstacle_cloud = perception_3d_ros_->getSharedDataPtr()->aggregate_observation_;
  const bool has_obstacles = obstacle_cloud && !obstacle_cloud->points.empty();
  pcl::KdTreeFLANN<pcl::PointXYZI> obstacle_kdtree;
  if(has_obstacles){
    obstacle_kdtree.setInputCloud(obstacle_cloud);
  }

  geometry_msgs::msg::PoseStamped target_pose;
  double target_yaw = 0.0;
  if(!getLookaheadTarget(mppi_lookahead_distance_, target_pose, target_yaw)){
    RCLCPP_WARN_THROTTLE(
      this->get_logger().get_child(name_),
      *clock_,
      5000,
      "Unable to resolve MPPI lookahead target from prune plan.");
    return dddmr_sys_core::ALL_TRAJECTORIES_FAIL;
  }

  const double allowed_max_linear_speed =
    perception_3d_ros_->getSharedDataPtr()->current_allowed_max_linear_speed_;
  const double max_linear_speed =
    allowed_max_linear_speed > 0.0 ?
    std::min(mppi_max_vel_x_, allowed_max_linear_speed) :
    mppi_max_vel_x_;

  if(max_linear_speed <= 1e-3 || mppi_horizon_steps_ <= 0 || mppi_batch_size_ <= 0){
    RCLCPP_ERROR_THROTTLE(
      this->get_logger().get_child(name_),
      *clock_,
      5000,
      "Invalid MPPI configuration. max_linear_speed=%.3f horizon=%d batch=%d",
      max_linear_speed,
      mppi_horizon_steps_,
      mppi_batch_size_);
    return dddmr_sys_core::CONFIGURATION_ERROR;
  }

  const double current_linear_speed =
    std::fabs(robot_state_.twist.twist.linear.x) > 1e-3 ?
    robot_state_.twist.twist.linear.x :
    trajectory_generators_ros_->getSharedDataPtr()->ref_twist_for_trajectory_generation_.twist.linear.x;
  const double current_angular_speed =
    std::fabs(robot_state_.twist.twist.angular.z) > 1e-3 ?
    robot_state_.twist.twist.angular.z :
    trajectory_generators_ros_->getSharedDataPtr()->ref_twist_for_trajectory_generation_.twist.angular.z;

  initializeMPPINominalControl(current_linear_speed, current_angular_speed);
  shiftMPPINominalControl();

  std::vector<double> nominal_vx = mppi_nominal_vx_;
  std::vector<double> nominal_wz = mppi_nominal_wz_;

  geometry_msgs::msg::PoseArray accepted_pose_arr;
  geometry_msgs::msg::PoseArray sampled_pose_arr;
  double best_cost = std::numeric_limits<double>::infinity();
  bool found_valid_traj = false;
  std::vector<double> best_vx_sequence;
  std::vector<double> best_wz_sequence;
  rejected_trajectories_.clear();

  for(int iteration = 0; iteration < mppi_iterations_; ++iteration){
    std::normal_distribution<double> vx_noise_dist(0.0, mppi_noise_vx_);
    std::normal_distribution<double> wz_noise_dist(0.0, mppi_noise_wz_);

    std::vector<std::vector<double>> sampled_noise_vx(
      mppi_batch_size_,
      std::vector<double>(mppi_horizon_steps_, 0.0));
    std::vector<std::vector<double>> sampled_noise_wz(
      mppi_batch_size_,
      std::vector<double>(mppi_horizon_steps_, 0.0));
    std::vector<std::vector<double>> sampled_vx(
      mppi_batch_size_,
      std::vector<double>(mppi_horizon_steps_, 0.0));
    std::vector<std::vector<double>> sampled_wz(
      mppi_batch_size_,
      std::vector<double>(mppi_horizon_steps_, 0.0));
    std::vector<double> sample_costs(
      mppi_batch_size_,
      std::numeric_limits<double>::infinity());
    std::vector<base_trajectory::Trajectory> sampled_trajs(mppi_batch_size_);
    double iteration_best_cost = std::numeric_limits<double>::infinity();

    for(int sample_index = 0; sample_index < mppi_batch_size_; ++sample_index){
      double previous_vx = current_linear_speed;
      double previous_wz = current_angular_speed;
      for(int step = 0; step < mppi_horizon_steps_; ++step){
        sampled_noise_vx[sample_index][step] = vx_noise_dist(mppi_rng_);
        sampled_noise_wz[sample_index][step] = wz_noise_dist(mppi_rng_);

        const double desired_vx = nominal_vx[step] + sampled_noise_vx[sample_index][step];
        const double desired_wz = nominal_wz[step] + sampled_noise_wz[sample_index][step];
        const double min_linear_speed =
          (step == 0 && std::fabs(previous_vx) < 1e-3) || std::fabs(desired_vx) < mppi_min_vel_x_ ?
          0.0 : mppi_min_vel_x_;

        sampled_vx[sample_index][step] = clampVelocityByAcceleration(
          desired_vx,
          previous_vx,
          mppi_acc_lim_x_,
          mppi_dt_,
          min_linear_speed,
          max_linear_speed);
        sampled_wz[sample_index][step] = clampVelocityByAcceleration(
          desired_wz,
          previous_wz,
          mppi_acc_lim_theta_,
          mppi_dt_,
          -mppi_max_vel_theta_,
          mppi_max_vel_theta_);

        previous_vx = sampled_vx[sample_index][step];
        previous_wz = sampled_wz[sample_index][step];
      }

      sampled_trajs[sample_index] = rolloutMPPI(
        sampled_vx[sample_index],
        sampled_wz[sample_index],
        mppi_dt_);
      sample_costs[sample_index] = scoreMPPI(
        sampled_trajs[sample_index],
        sampled_vx[sample_index],
        sampled_wz[sample_index],
        prune_plan_kdtree,
        obstacle_kdtree,
        has_obstacles,
        target_pose,
        target_yaw);

      sampled_trajs[sample_index].cost_ = std::isfinite(sample_costs[sample_index]) ?
        sample_costs[sample_index] : -1.0;
      sampled_trajs[sample_index].rejected_by_ =
        std::isfinite(sample_costs[sample_index]) ? "accepted" : "mppi_constraints";
      rejected_trajectories_[sampled_trajs[sample_index].rejected_by_].push_back(
        sampled_trajs[sample_index]);
      pcl::PointCloud<pcl::PointXYZ> sampled_cuboids_pcl;
      trajectory2posearray_cuboids(
        sampled_trajs[sample_index],
        sampled_pose_arr,
        sampled_cuboids_pcl);

      if(std::isfinite(sample_costs[sample_index])){
        pcl::PointCloud<pcl::PointXYZ> accepted_cuboids_pcl;
        trajectory2posearray_cuboids(
          sampled_trajs[sample_index],
          accepted_pose_arr,
          accepted_cuboids_pcl);
        iteration_best_cost = std::min(iteration_best_cost, sample_costs[sample_index]);
        if(sample_costs[sample_index] < best_cost){
          best_cost = sample_costs[sample_index];
          best_traj = sampled_trajs[sample_index];
          best_vx_sequence = sampled_vx[sample_index];
          best_wz_sequence = sampled_wz[sample_index];
          found_valid_traj = true;
        }
      }
    }

    if(!std::isfinite(iteration_best_cost)){
      continue;
    }

    std::vector<double> weighted_noise_vx(mppi_horizon_steps_, 0.0);
    std::vector<double> weighted_noise_wz(mppi_horizon_steps_, 0.0);
    double weight_sum = 0.0;
    const double safe_lambda = std::max(1e-4, mppi_lambda_);

    for(int sample_index = 0; sample_index < mppi_batch_size_; ++sample_index){
      if(!std::isfinite(sample_costs[sample_index])){
        continue;
      }
      const double weight = std::exp(-(sample_costs[sample_index] - iteration_best_cost) / safe_lambda);
      weight_sum += weight;
      for(int step = 0; step < mppi_horizon_steps_; ++step){
        weighted_noise_vx[step] += weight * sampled_noise_vx[sample_index][step];
        weighted_noise_wz[step] += weight * sampled_noise_wz[sample_index][step];
      }
    }

    if(weight_sum <= 1e-9){
      continue;
    }

    double previous_vx = current_linear_speed;
    double previous_wz = current_angular_speed;
    for(int step = 0; step < mppi_horizon_steps_; ++step){
      const double desired_vx = nominal_vx[step] + weighted_noise_vx[step] / weight_sum;
      const double desired_wz = nominal_wz[step] + weighted_noise_wz[step] / weight_sum;
      const double min_linear_speed =
        (step == 0 && std::fabs(previous_vx) < 1e-3) || std::fabs(desired_vx) < mppi_min_vel_x_ ?
        0.0 : mppi_min_vel_x_;

      nominal_vx[step] = clampVelocityByAcceleration(
        desired_vx,
        previous_vx,
        mppi_acc_lim_x_,
        mppi_dt_,
        min_linear_speed,
        max_linear_speed);
      nominal_wz[step] = clampVelocityByAcceleration(
        desired_wz,
        previous_wz,
        mppi_acc_lim_theta_,
        mppi_dt_,
        -mppi_max_vel_theta_,
        mppi_max_vel_theta_);
      previous_vx = nominal_vx[step];
      previous_wz = nominal_wz[step];
    }
  }

  if(found_valid_traj){
    mppi_nominal_vx_ = best_vx_sequence;
    mppi_nominal_wz_ = best_wz_sequence;
    best_traj.xv_ = best_vx_sequence.front();
    best_traj.yv_ = 0.0;
    best_traj.thetav_ = best_wz_sequence.front();
    best_traj.time_delta_ = mppi_dt_;
    best_traj.cost_ = best_cost;
  }

  accepted_pose_arr.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  accepted_pose_arr.header.stamp = clock_->now();
  pub_accepted_trajectory_pose_array_->publish(accepted_pose_arr);

  sampled_pose_arr.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  sampled_pose_arr.header.stamp = clock_->now();
  pub_trajectory_pose_array_->publish(sampled_pose_arr);

  geometry_msgs::msg::PoseArray best_pose_arr;
  pcl::PointCloud<pcl::PointXYZ> cuboids_pcl;
  trajectory2posearray_cuboids(best_traj, best_pose_arr, cuboids_pcl);
  best_pose_arr.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  best_pose_arr.header.stamp = clock_->now();
  pub_best_trajectory_pose_->publish(best_pose_arr);

  auto t_diff = clock_->now() - control_loop_time_;
  RCLCPP_DEBUG(this->get_logger().get_child(name_), "Full MPPI control cycle time: %.9f", t_diff.seconds());

  if(t_diff.seconds() > 1./controller_frequency_){
    RCLCPP_WARN(
      this->get_logger().get_child(name_),
      "Local planner MPPI control time exceed expect time: %.2f but is %.2f",
      1./controller_frequency_,
      t_diff.seconds());
  }

  std::vector<perception_3d::PerceptionOpinion> opinions =
    perception_3d_ros_->getStackedPerception()->getOpinions();
  for(auto opinion_it = opinions.begin(); opinion_it != opinions.end(); ++opinion_it){
    if((*opinion_it) == perception_3d::PATH_BLOCKED_WAIT){
      RCLCPP_WARN_THROTTLE(
        this->get_logger().get_child(name_),
        *clock_,
        5000,
        "Found the prune plan is blocked, go to wait state.");
      return dddmr_sys_core::PATH_BLOCKED_WAIT;
    }
    else if((*opinion_it) == perception_3d::PATH_BLOCKED_REPLANNING){
      RCLCPP_WARN_THROTTLE(
        this->get_logger().get_child(name_),
        *clock_,
        5000,
        "Found the prune plan is blocked, go to replanning.");
      return dddmr_sys_core::PATH_BLOCKED_REPLANNING;
    }
  }

  if(!found_valid_traj){
    const std::string rejection_report =
      formatRejectedTrajectoryReport(rejected_trajectories_, mppi_batch_size_ * mppi_iterations_);
    RCLCPP_WARN_THROTTLE(
      this->get_logger().get_child(name_),
      *clock_,
      5000,
      "All MPPI rollouts are rejected. report={%s} prune_plan_points=%zu observation_points=%zu",
      rejection_report.c_str(),
      prune_plan_.poses.size(),
      perception_3d_ros_->getSharedDataPtr()->aggregate_observation_->points.size());
    return dddmr_sys_core::ALL_TRAJECTORIES_FAIL;
  }

  auto& ref_twist = trajectory_generators_ros_->getSharedDataPtr()->ref_twist_for_trajectory_generation_;
  ref_twist.header.stamp = clock_->now();
  ref_twist.header.frame_id = perception_3d_ros_->getGlobalUtils()->getRobotFrame();
  ref_twist.twist.linear.x = best_traj.xv_;
  ref_twist.twist.linear.y = 0.0;
  ref_twist.twist.angular.z = best_traj.thetav_;
  return dddmr_sys_core::TRAJECTORY_FOUND;
}

void Local_Planner::setPlan(const std::vector<geometry_msgs::msg::PoseStamped>& orig_global_plan) {

  if(orig_global_plan.size()<3){
    RCLCPP_ERROR(this->get_logger().get_child(name_), "Size of global plan is smaller than 3.");
    return;
  }

  global_plan_.clear();
  global_plan_ = orig_global_plan;

  pcl_global_plan_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  for(auto gbl_it = global_plan_.begin(); gbl_it!=global_plan_.end();gbl_it++){
    pcl::PointXYZ pt;
    pt.x = (*gbl_it).pose.position.x;
    pt.y = (*gbl_it).pose.position.y;
    pt.z = (*gbl_it).pose.position.z;
    pcl_global_plan_->push_back(pt);
  }

  kdtree_global_plan_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
  kdtree_global_plan_->setInputCloud (pcl_global_plan_);
  RCLCPP_INFO_THROTTLE(this->get_logger().get_child(name_), *clock_, 10000, "Recieve new global plan.");
  //RCLCPP_INFO(this->get_logger().get_child(name_), "Recieve new global plan: %.2f, %.2f", 
  //    global_plan_.back().pose.position.x, global_plan_.back().pose.position.y);
}

double Local_Planner::getDistanceBTWPoseStamp(const geometry_msgs::msg::PoseStamped& a, const geometry_msgs::msg::PoseStamped& b){

  double dx = a.pose.position.x-b.pose.position.x;
  double dy = a.pose.position.y-b.pose.position.y;
  double dz = a.pose.position.z-b.pose.position.z;
  return sqrt(dx*dx + dy*dy + dz*dz);
}

void Local_Planner::updateGlobalPose(){
  try
  {
    trans_gbl2b_ = tf2Buffer_->lookupTransform(
        global_frame_, robot_frame_, tf2::TimePointZero);
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_DEBUG(this->get_logger().get_child(name_), "%s: %s", name_.c_str(),e.what());
  }
  robot_cuboid_.markers.clear();
  marker_edge_.header.stamp = trans_gbl2b_.header.stamp;
  robot_cuboid_.markers.push_back(marker_edge_);
  pub_robot_cuboid_->publish(robot_cuboid_);
}

geometry_msgs::msg::TransformStamped Local_Planner::getGlobalPose(){
  return trans_gbl2b_;
}

void Local_Planner::prunePlan(double forward_distance, double backward_distance){

  if(pcl_global_plan_->points.size()<3)
    return;

  prune_plan_.poses.clear();
  pcl_prune_plan_.clear();

  std::vector<int> pointIdxNKNSearch(1);
  std::vector<float> pointNKNSquaredDistance(1);
  pcl::PointXYZ robot_pose;
  robot_pose.x = trans_gbl2b_.transform.translation.x;
  robot_pose.y = trans_gbl2b_.transform.translation.y;
  robot_pose.z = trans_gbl2b_.transform.translation.z;

  if ( kdtree_global_plan_->nearestKSearch (robot_pose, 1, pointIdxNKNSearch, pointNKNSquaredDistance) <= 0 ){
    RCLCPP_DEBUG(this->get_logger().get_child(name_), "Ready to fix some exception here.");
    return;
  }


  if(sqrt(pointNKNSquaredDistance[0]) > prune_plan_max_deviation_){
    RCLCPP_WARN_THROTTLE(
      this->get_logger().get_child(name_),
      *clock_,
      3000,
      "Robot deviated %.2f m from global plan, exceeding prune_plan_max_deviation %.2f m.",
      sqrt(pointNKNSquaredDistance[0]),
      prune_plan_max_deviation_);
    //@ consider to clear prune_plan in model_shared_data?
    return;
  }

  //@ backward check
  geometry_msgs::msg::PoseStamped last_pose = global_plan_[pointIdxNKNSearch[0]];
  for(int i=pointIdxNKNSearch[0]; i>=0; i--){
    prune_plan_.poses.push_back(global_plan_[i]);
    pcl::PointXYZI pt;
    pt.x = global_plan_[i].pose.position.x; pt.y = global_plan_[i].pose.position.y; pt.z = global_plan_[i].pose.position.z;
    pt.intensity = -1; //@ we tag backward plan as negative for path_blocked_strategy(plugin) to distinguish the backward pose
    pcl_prune_plan_.points.push_back(pt);
    if(i<pointIdxNKNSearch[0]){
      backward_distance -= getDistanceBTWPoseStamp(last_pose, global_plan_[i]);
    }
    last_pose = global_plan_[i];
    if(backward_distance<0)
      break;
  }
  
  std::reverse(prune_plan_.poses.begin(),prune_plan_.poses.end()); 

  //@ forward check
  for(int i=pointIdxNKNSearch[0];i<global_plan_.size();i++){
    prune_plan_.poses.push_back(global_plan_[i]);
    pcl::PointXYZI pt;
    pt.x = global_plan_[i].pose.position.x; pt.y = global_plan_[i].pose.position.y; pt.z = global_plan_[i].pose.position.z;
    if(i == 0){
      pt.intensity = 0;
    }
    else{
      pt.intensity = 1;
    }
    pcl_prune_plan_.points.push_back(pt);

    if(i>pointIdxNKNSearch[0]){
      forward_distance -= getDistanceBTWPoseStamp(last_pose, global_plan_[i]);
    }
    last_pose = global_plan_[i];
    if(forward_distance<0)
      break;
  }
  
  prune_plan_.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  prune_plan_.header.stamp = clock_->now();
  pub_prune_plan_->publish(prune_plan_);
  last_valid_prune_plan_ = clock_->now();
  //RCLCPP_DEBUG(this->get_logger().get_child(name_), "%lu",prune_plan_.poses.size());
}

void Local_Planner::getBestTrajectory(std::string traj_gen_name, base_trajectory::Trajectory& best_traj){

  //@ in case we have collision
  best_traj.cost_ = -1;

  double minimum_cost = 9999999;
  geometry_msgs::msg::PoseArray accepted_pose_arr;
  pcl::PointCloud<pcl::PointXYZ> cuboids_pcl;
  
  rejected_trajectories_.clear();

  for(auto traj_it=trajectories_->begin();traj_it!=trajectories_->end();traj_it++){

    mpc_critics_ros_->scoreTrajectory(traj_gen_name, (*traj_it));
    
    if((*traj_it).cost_>=0 && (*traj_it).cost_<=minimum_cost){
      best_traj = (*traj_it);
      minimum_cost = (*traj_it).cost_;
    }

    if((*traj_it).cost_>=0){
      trajectory2posearray_cuboids((*traj_it), accepted_pose_arr, cuboids_pcl);
    }

    rejected_trajectories_[(*traj_it).rejected_by_].push_back(*traj_it);
    
  }
  
  //for(auto report_it=rejected_trajectories_.begin(); report_it!=rejected_trajectories_.end(); report_it++){
  //  RCLCPP_INFO(this->get_logger().get_child(name_), "Report: %s with rate: %.2f", (*report_it).first.c_str(), (float)(*report_it).second.size()/(float)trajectories_->size());
  //}

  accepted_pose_arr.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  accepted_pose_arr.header.stamp = clock_->now();
  pub_accepted_trajectory_pose_array_->publish(accepted_pose_arr);

  geometry_msgs::msg::PoseArray best_pose_arr;
  trajectory2posearray_cuboids(best_traj, best_pose_arr, cuboids_pcl);
  best_pose_arr.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  best_pose_arr.header.stamp = clock_->now();
  pub_best_trajectory_pose_->publish(best_pose_arr);

}

namespace
{

std::string formatRejectedTrajectoryReport(
  const std::map<std::string, std::vector<base_trajectory::Trajectory>>& rejected_trajectories,
  std::size_t total_trajectories)
{
  std::ostringstream oss;
  bool first = true;
  for(const auto& [name, trajectories] : rejected_trajectories){
    if(!first){
      oss << ", ";
    }
    first = false;
    oss << name << ":" << trajectories.size();
  }

  if(total_trajectories == 0){
    oss << " | no_samples_generated";
  }

  return oss.str();
}

}  // namespace

dddmr_sys_core::PlannerState Local_Planner::computeVelocityCommand(std::string traj_gen_name, base_trajectory::Trajectory& best_traj){
  
  if(!got_odom_){
    RCLCPP_ERROR(this->get_logger().get_child(name_), "Odom is not received.");
    return dddmr_sys_core::TF_FAIL;
  }

  if(!perception_3d_ros_->getStackedPerception()->isSensorOK()){
    RCLCPP_ERROR(this->get_logger().get_child(name_), "Perception 3D is not ok.");
    return dddmr_sys_core::PERCEPTION_MALFUNCTION;
  }
  
  if(!trajectory_generators_ros_->theoryExists(traj_gen_name)){
    RCLCPP_ERROR(this->get_logger().get_child(name_), "Specified trajectory generator: %s is not declare in yaml nor not consistent", traj_gen_name.c_str());
    return dddmr_sys_core::CONFIGURATION_ERROR;
  }

  //for timing that gives real time even in simulation
  control_loop_time_ = clock_->now();

  std::unique_lock<perception_3d::StackedPerception::mutex_t> pct_lock(*(perception_3d_ros_->getStackedPerception()->getMutex()));
  
  //@ update current observation for scoring
  //@ we need to visualized this for debug/justification
  perception_3d_ros_->getStackedPerception()->aggregateObservations();

  //@ forward_prune_/backward_prune_: should adapt to vehicle speed.
  //@ prune plan are used by trajectory_generators/perception
  //@ prune plan has to come after mutex lock, because global_plan_ros_sub_ reset global plan kd tree
  prunePlan(forward_prune_, backward_prune_);

  sensor_msgs::msg::PointCloud2 ros2_aggregate_onservation;
  pcl::toROSMsg(*(perception_3d_ros_->getSharedDataPtr()->aggregate_observation_), ros2_aggregate_onservation);
  pub_aggregate_observation_->publish(ros2_aggregate_onservation);
  if((clock_->now()-trans_gbl2b_.header.stamp).seconds() > 2.0){
    RCLCPP_ERROR(this->get_logger().get_child(name_), "TF out of date in local planner, the local planner wont go further.");
    return dddmr_sys_core::TF_FAIL;
  }

  //@ TODO: Compute cuboid of each pose and send to determineIsPathBlock
  perception_3d_ros_->getSharedDataPtr()->pcl_prune_plan_ = pcl_prune_plan_;
  //perception_3d_ros_->getStackedPerception()->determineIsPathBlock(pcl_prune_plan_);

  if((clock_->now()-last_valid_prune_plan_).seconds()>=prune_plane_timeout_){
    RCLCPP_FATAL(this->get_logger().get_child(name_), "Deviate global plan too much, computeVelocityCommand() returns false.");
    return dddmr_sys_core::PRUNE_PLAN_FAIL;
  }

  //Do not create a function to set the parameters unless a nice structure is found
  //Below assignment of variables is useful when migrate to ROS2
  trajectory_generators_ros_->getSharedDataPtr()->robot_pose_ = trans_gbl2b_;
  trajectory_generators_ros_->getSharedDataPtr()->robot_state_ = robot_state_;
  trajectory_generators_ros_->getSharedDataPtr()->ackermann_drive_state_ = ackermann_drive_state_;
  trajectory_generators_ros_->getSharedDataPtr()->prune_plan_ = prune_plan_;
  //@ change max speed from perception shared data framework
  trajectory_generators_ros_->getSharedDataPtr()->current_allowed_max_linear_speed_ 
                  = perception_3d_ros_->getSharedDataPtr()->current_allowed_max_linear_speed_;

  trajectory_generators_ros_->initializeTheories_wi_Shared_data();

  geometry_msgs::msg::PoseArray pose_arr;
  pcl::PointCloud<pcl::PointXYZ> cuboids_pcl;

  trajectories_ = std::make_shared<std::vector<base_trajectory::Trajectory>>();

  #ifdef HAVE_SYS_TIME_H
  struct timeval start, end;
  double start_t, end_t, t_diff;
  gettimeofday(&start, NULL);
  #endif

  //@ We queue all trajectories in trajectories_, then score them one by one in getBestTrajectory()
  while(trajectory_generators_ros_->hasMoreTrajectories(traj_gen_name)){
    base_trajectory::Trajectory a_traj;
    if(trajectory_generators_ros_->nextTrajectory(traj_gen_name, a_traj)){
      //@ collected all trajectories here, for later scoring
      trajectories_->push_back(a_traj);
      trajectory2posearray_cuboids(a_traj, pose_arr, cuboids_pcl);
    }

  }

  #ifdef HAVE_SYS_TIME_H
  gettimeofday(&end, NULL);
  start_t = start.tv_sec + double(start.tv_usec) / 1e6;
  end_t = end.tv_sec + double(end.tv_usec) / 1e6;
  t_diff = end_t - start_t;
  RCLCPP_WARN(this->get_logger(), "Map update time: %.9f", t_diff);
  #endif

  pose_arr.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  pose_arr.header.stamp = clock_->now();
  pub_trajectory_pose_array_->publish(pose_arr);

  
  //cuboids_pcl.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  //pub_cuboids_.publish(cuboids_pcl);
  

  //@Update data for critics
  std::unique_lock<mpc_critics::StackedScoringModel::model_mutex_t> critics_lock(*(mpc_critics_ros_->getStackedScoringModelPtr()->getMutex()));
  //@ unless we come up with a better strcuture
  //@ keep below for easy migration for ROS2
  mpc_critics_ros_->getSharedDataPtr()->robot_pose_ = trans_gbl2b_;
  mpc_critics_ros_->getSharedDataPtr()->robot_state_ = robot_state_;
  mpc_critics_ros_->getSharedDataPtr()->ackermann_drive_state_ = ackermann_drive_state_;
  mpc_critics_ros_->getSharedDataPtr()->pcl_perception_ = perception_3d_ros_->getSharedDataPtr()->aggregate_observation_;
  mpc_critics_ros_->getSharedDataPtr()->prune_plan_ = prune_plan_;
  //@ Below function transform prune_plane from nav::msg to pcl type
  //@ Below function generate kd-tree using aggregate observation
  mpc_critics_ros_->updateSharedData();
  getBestTrajectory(traj_gen_name, best_traj);

  auto t_diff = clock_->now() - control_loop_time_;
  RCLCPP_DEBUG(this->get_logger().get_child(name_), "Full control cycle time: %.9f", t_diff.seconds());

  if(t_diff.seconds() > 1./controller_frequency_){
    RCLCPP_WARN(this->get_logger().get_child(name_), "Local planner control time exceed expect time: %.2f but is %.2f", 1./controller_frequency_, t_diff.seconds());
  }
  
  const bool is_rotate_generator = traj_gen_name.find("rotate") != std::string::npos;
  //@Loop opinions
  std::vector<perception_3d::PerceptionOpinion> opinions = perception_3d_ros_->getStackedPerception()->getOpinions();
  for(auto opinion_it=opinions.begin(); opinion_it!=opinions.end();opinion_it++){
    if((*opinion_it)==perception_3d::PATH_BLOCKED_WAIT){
      if(is_rotate_generator){
        RCLCPP_WARN_THROTTLE(
          this->get_logger().get_child(name_),
          *clock_,
          5000,
          "Ignoring PATH_BLOCKED_WAIT while evaluating rotate trajectory %s.",
          traj_gen_name.c_str());
        continue;
      }
      RCLCPP_WARN_THROTTLE(this->get_logger().get_child(name_), *clock_, 5000, "Found the prune plan is blocked, go to wait state.");
      return dddmr_sys_core::PATH_BLOCKED_WAIT;
    }
    else if((*opinion_it)==perception_3d::PATH_BLOCKED_REPLANNING){
      if(is_rotate_generator){
        RCLCPP_WARN_THROTTLE(
          this->get_logger().get_child(name_),
          *clock_,
          5000,
          "Ignoring PATH_BLOCKED_REPLANNING while evaluating rotate trajectory %s.",
          traj_gen_name.c_str());
        continue;
      }
      RCLCPP_WARN_THROTTLE(this->get_logger().get_child(name_), *clock_, 5000, "Found the prune plan is blocked, go to replanning.");
      return dddmr_sys_core::PATH_BLOCKED_REPLANNING;      
    }
  }


  if(best_traj.cost_<0){
    const std::string rejection_report =
      formatRejectedTrajectoryReport(rejected_trajectories_, trajectories_->size());
    RCLCPP_WARN_THROTTLE(
      this->get_logger().get_child(name_),
      *clock_,
      5000,
      "All trajectories are rejected by critics. generator=%s report={%s} prune_plan_points=%zu observation_points=%zu",
      traj_gen_name.c_str(),
      rejection_report.c_str(),
      prune_plan_.poses.size(),
      perception_3d_ros_->getSharedDataPtr()->aggregate_observation_->points.size());
    return dddmr_sys_core::ALL_TRAJECTORIES_FAIL;
  }
  else{
    auto& ref_twist = trajectory_generators_ros_->getSharedDataPtr()->ref_twist_for_trajectory_generation_;
    ref_twist.header.stamp = clock_->now();
    ref_twist.header.frame_id = perception_3d_ros_->getGlobalUtils()->getRobotFrame();
    ref_twist.twist.linear.x = best_traj.xv_;
    ref_twist.twist.linear.y = best_traj.yv_;
    ref_twist.twist.angular.z = best_traj.thetav_;
    return dddmr_sys_core::TRAJECTORY_FOUND;
  }
}

void Local_Planner::trajectory2posearray_cuboids(const base_trajectory::Trajectory& a_traj, 
                                      geometry_msgs::msg::PoseArray& pose_arr,
                                      pcl::PointCloud<pcl::PointXYZ>& cuboids_pcl){

  for(unsigned int i=0;i<a_traj.getPointsSize();i++){
      auto p = a_traj.getPoint(i);
      pose_arr.poses.push_back(p.pose);
      //@ For cuboids debug
      //cuboids_pcl += a_traj.getCuboid(i);       
  }

}

/*
void Local_Planner::cbMCL_ground_normal(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  
  ground_with_normals_.reset(new pcl::PointCloud<pcl::PointNormal>);
  pcl::fromROSMsg(*msg, *ground_with_normals_);
  if(perception_3d_ros_->getGlobalUtils()->getGblFrame().compare(msg->header.frame_id) != 0)
    ROS_ERROR("%s: the global frame is not consistent with topics and perception setting.", name_.c_str());
  global_frame_ = msg->header.frame_id;
  normal2quaternion();
}

void Local_Planner::normal2quaternion(){

  visualization_msgs::MarkerArray markerArray;
  for(size_t i=0;i<ground_with_normals_->points.size();i++){

    tf2::Vector3 axis_vector(ground_with_normals_->points[i].normal_x, ground_with_normals_->points[i].normal_y, ground_with_normals_->points[i].normal_z);

    tf2::Vector3 up_vector(1.0, 0.0, 0.0);
    tf2::Vector3 right_vector = axis_vector.cross(up_vector);
    right_vector.normalized();
    tf2::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
    q.normalize();

    //@Create arrow
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = ground_with_normals_->header.frame_id;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = i;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::ARROW;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = ground_with_normals_->points[i].x;
    marker.pose.position.y = ground_with_normals_->points[i].y;
    marker.pose.position.z = ground_with_normals_->points[i].z;
    marker.pose.orientation.x = q.getX();
    marker.pose.orientation.y = q.getY();
    marker.pose.orientation.z = q.getZ();
    marker.pose.orientation.w = q.getW();

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.3; //scale.x is the arrow length,
    marker.scale.y = 0.05; //scale.y is the arrow width 
    marker.scale.z = 0.1; //scale.z is the arrow height. 

    double angle = atan2(ground_with_normals_->points[i].normal_z, 
                  sqrt(ground_with_normals_->points[i].normal_x*ground_with_normals_->points[i].normal_x+ ground_with_normals_->points[i].normal_y*ground_with_normals_->points[i].normal_y) ) * 180 / 3.1415926535;

    if(fabs(angle)<=10){
      marker.color.r = 1.0f;
      marker.color.g = 0.5f;
      marker.color.b = 0.0f;      
    }
    else{
      marker.color.r = 0.0f;
      marker.color.g = 0.8f;
      marker.color.b = 0.2f; 
    }

    marker.color.a = 0.6f;   
    markerArray.markers.push_back(marker); 
  }
  pub_pc_normal_.publish(markerArray);
}
*/

}// end of name space
