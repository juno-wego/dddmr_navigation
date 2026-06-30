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
#include <mpc_critics/pure_pursuit_model.h>

#include <cmath>

PLUGINLIB_EXPORT_CLASS(mpc_critics::PurePursuitModel, mpc_critics::ScoringModel)

namespace mpc_critics
{

namespace
{

double planarDistance(
  const geometry_msgs::msg::PoseStamped& a,
  const geometry_msgs::msg::PoseStamped& b)
{
  const double dx = a.pose.position.x - b.pose.position.x;
  const double dy = a.pose.position.y - b.pose.position.y;
  return std::hypot(dx, dy);
}

double getYaw(const geometry_msgs::msg::Quaternion& q_msg)
{
  tf2::Quaternion q;
  tf2::convert(q_msg, q);
  double roll;
  double pitch;
  double yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

double shortestAngularDistance(double from, double to)
{
  return std::atan2(std::sin(to - from), std::cos(to - from));
}

}  // namespace

PurePursuitModel::PurePursuitModel(){
  return;
  
}

void PurePursuitModel::onInitialize(){

  node_->declare_parameter(name_ + ".weight", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".weight", weight_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "weight: %.2f", weight_);

  node_->declare_parameter(name_ + ".translation_weight", rclcpp::ParameterValue(0.5));
  node_->get_parameter(name_ + ".translation_weight", translation_weight_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "translation_weight: %.2f", translation_weight_);

  node_->declare_parameter(name_ + ".orientation_weight", rclcpp::ParameterValue(0.5));
  node_->get_parameter(name_ + ".orientation_weight", orientation_weight_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "orientation_weight: %.2f", orientation_weight_);

  node_->declare_parameter(name_ + ".lookahead_distance", rclcpp::ParameterValue(0.8));
  node_->get_parameter(name_ + ".lookahead_distance", lookahead_distance_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "lookahead_distance: %.2f", lookahead_distance_);

}


double PurePursuitModel::scoreTrajectory(base_trajectory::Trajectory &traj){

  if(shared_data_->prune_plan_.poses.size() < 2 || traj.getPointsSize()<2){
    return -4.0;  
  }

  geometry_msgs::msg::PoseStamped last_traj_pose = traj.getPoint(traj.getPointsSize()-1);
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.pose.position.x = shared_data_->robot_pose_.transform.translation.x;
  robot_pose.pose.position.y = shared_data_->robot_pose_.transform.translation.y;
  robot_pose.pose.position.z = shared_data_->robot_pose_.transform.translation.z;
  robot_pose.pose.orientation = shared_data_->robot_pose_.transform.rotation;

  const auto& prune_plan = shared_data_->prune_plan_.poses;
  std::size_t nearest_index = 0;
  double nearest_distance = std::numeric_limits<double>::max();
  for(std::size_t i = 0; i < prune_plan.size(); ++i){
    const double distance = planarDistance(robot_pose, prune_plan[i]);
    if(distance < nearest_distance){
      nearest_distance = distance;
      nearest_index = i;
    }
  }

  std::size_t target_index = nearest_index;
  double accumulated_distance = 0.0;
  while(target_index + 1 < prune_plan.size() && accumulated_distance < lookahead_distance_){
    accumulated_distance += planarDistance(prune_plan[target_index], prune_plan[target_index + 1]);
    ++target_index;
  }

  const geometry_msgs::msg::PoseStamped& target_pose = prune_plan[target_index];
  const double translation_error = planarDistance(last_traj_pose, target_pose);

  double target_yaw = getYaw(target_pose.pose.orientation);
  if(target_index + 1 < prune_plan.size()){
    const auto& next_pose = prune_plan[target_index + 1];
    const double dx = next_pose.pose.position.x - target_pose.pose.position.x;
    const double dy = next_pose.pose.position.y - target_pose.pose.position.y;
    if(std::hypot(dx, dy) > 1e-4){
      target_yaw = std::atan2(dy, dx);
    }
  }
  const double traj_yaw = getYaw(last_traj_pose.pose.orientation);
  const double orientation_error = std::fabs(shortestAngularDistance(traj_yaw, target_yaw));

  return translation_weight_ * translation_error +
         orientation_weight_ * orientation_error;
}

}//end of name space
