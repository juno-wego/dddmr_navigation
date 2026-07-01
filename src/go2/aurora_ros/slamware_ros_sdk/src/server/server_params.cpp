
#include "server_params.h"

#include <cmath>

namespace slamware_ros_sdk {

    //////////////////////////////////////////////////////////////////////////

    const float C_FLT_PI = ((float)M_PI);
    const float C_FLT_2PI = (C_FLT_PI * 2);

    //////////////////////////////////////////////////////////////////////////

    ServerParams::ServerParams()
    : Node("server_params")
    {
        resetToDefault();
    }

    void ServerParams::resetToDefault()
    {
        this->declare_parameter<std::string>("ip_address", "192.168.11.1");
        this->declare_parameter<int>("reconn_wait_ms", 1000 * 3);

        this->declare_parameter<bool>("angle_compensate", true);
        this->declare_parameter<bool>("ladar_data_clockwise", true);

        this->declare_parameter<std::string>("robot_frame", "base_link");
        this->declare_parameter<std::string>("laser_frame", "laser");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("imu_frame", "imu_link");
        this->declare_parameter<std::string>("camera_left", "camera_left");
        this->declare_parameter<std::string>("camera_right", "camera_right");

        this->declare_parameter<float>("odometry_pub_period", 0.01f);
        this->declare_parameter<float>("robot_pose_pub_period", 0.01f);
        this->declare_parameter<float>("scan_pub_period", 0.1f);
        this->declare_parameter<float>("map_update_period", 0.3f);
        this->declare_parameter<float>("map_pub_period", 0.3f);

        this->declare_parameter<float>("map_sync_once_get_max_wh", 100.f);
        this->declare_parameter<float>("map_update_near_robot_half_wh", 8.0f);

        this->declare_parameter<float>("imu_raw_data_period", 0.05f);
        this->declare_parameter<float>("system_status_pub_period", 0.1f);
        this->declare_parameter<float>("stereo_image_pub_period", 0.1f);
        this->declare_parameter<float>("point_cloud_pub_period", 0.2f);
        this->declare_parameter<float>("enhanced_imaging_pub_period", 0.5f);
        this->declare_parameter<float>("robot_basic_state_pub_period", 0.5f);

        this->declare_parameter<std::string>("scan_topic", "/slamware_ros_sdk_server_node/scan");
        this->declare_parameter<std::string>("robot_pose_topic", "/slamware_ros_sdk_server_node/robot_pose");
        this->declare_parameter<std::string>("odom_topic", "/slamware_ros_sdk_server_node/odom");
        this->declare_parameter<std::string>("map_topic", "/slamware_ros_sdk_server_node/map");
        this->declare_parameter<std::string>("map_info_topic", "/slamware_ros_sdk_server_node/map_metadata");
        this->declare_parameter<std::string>("system_status_topic_name", "/slamware_ros_sdk_server_node/system_status");
        this->declare_parameter<std::string>("relocalization_status_topic_name", "/slamware_ros_sdk_server_node/relocalization_status");
        this->declare_parameter<std::string>("left_image_raw_topic_name", "/slamware_ros_sdk_server_node/left_image_raw");
        this->declare_parameter<std::string>("right_image_raw_topic_name", "/slamware_ros_sdk_server_node/right_image_raw");
        this->declare_parameter<std::string>("point_cloud_topic_name", "/slamware_ros_sdk_server_node/point_cloud");
        this->declare_parameter<std::string>("stereo_keypoints_topic_name", "/slamware_ros_sdk_server_node/stereo_keypoints");
        
        // Enhanced imaging topics
        this->declare_parameter<std::string>("depth_image_raw_topic_name", "/slamware_ros_sdk_server_node/depth_image_raw");
        this->declare_parameter<std::string>("depth_image_colorized_topic_name", "/slamware_ros_sdk_server_node/depth_image_colorized");
        this->declare_parameter<std::string>("depth_point_cloud_topic_name", "/slamware_ros_sdk_server_node/depth_point_cloud");
        this->declare_parameter<std::string>("semantic_segmentation_topic_name", "/slamware_ros_sdk_server_node/semantic_segmentation");

        this->declare_parameter<std::string>("imu_raw_data_topic", "/slamware_ros_sdk_server_node/imu_raw_data");
    }

    void ServerParams::setBy(const std::shared_ptr<rclcpp::Node> nhRos)
    {
        auto syncString = [this, nhRos](const std::string& name) {
            auto value = this->getParameter<std::string>(name);
            if (nhRos->has_parameter(name)) {
                nhRos->get_parameter(name, value);
            } else {
                value = nhRos->declare_parameter<std::string>(name, value);
            }
            this->set_parameter(rclcpp::Parameter(name, value));
        };
        auto syncBool = [this, nhRos](const std::string& name) {
            auto value = this->getParameter<bool>(name);
            if (nhRos->has_parameter(name)) {
                nhRos->get_parameter(name, value);
            } else {
                value = nhRos->declare_parameter<bool>(name, value);
            }
            this->set_parameter(rclcpp::Parameter(name, value));
        };
        auto syncInt = [this, nhRos](const std::string& name) {
            auto value = this->getParameter<int>(name);
            if (nhRos->has_parameter(name)) {
                nhRos->get_parameter(name, value);
            } else {
                value = nhRos->declare_parameter<int>(name, value);
            }
            this->set_parameter(rclcpp::Parameter(name, value));
        };
        auto syncFloat = [this, nhRos](const std::string& name) {
            auto value = this->getParameter<float>(name);
            if (nhRos->has_parameter(name)) {
                nhRos->get_parameter(name, value);
            } else {
                value = nhRos->declare_parameter<float>(name, value);
            }
            this->set_parameter(rclcpp::Parameter(name, value));
        };

        syncString("ip_address");
        syncInt("reconn_wait_ms");
        syncBool("angle_compensate");
        syncBool("ladar_data_clockwise");

        syncString("robot_frame");
        syncString("laser_frame");
        syncString("map_frame");
        syncString("odom_frame");
        syncString("imu_frame");
        syncString("camera_left");
        syncString("camera_right");

        syncFloat("odometry_pub_period");
        syncFloat("robot_pose_pub_period");
        syncFloat("scan_pub_period");
        syncFloat("map_update_period");
        syncFloat("map_pub_period");
        syncFloat("map_sync_once_get_max_wh");
        syncFloat("map_update_near_robot_half_wh");
        syncFloat("system_status_pub_period");
        syncFloat("stereo_image_pub_period");
        syncFloat("point_cloud_pub_period");
        syncFloat("enhanced_imaging_pub_period");
        syncFloat("robot_basic_state_pub_period");
        syncFloat("imu_raw_data_period");

        syncString("scan_topic");
        syncString("odom_topic");
        syncString("robot_pose_topic");
        syncString("map_topic");
        syncString("map_info_topic");
        syncString("system_status_topic_name");
        syncString("relocalization_status_topic_name");
        syncString("left_image_raw_topic_name");
        syncString("right_image_raw_topic_name");
        syncString("point_cloud_topic_name");
        syncString("stereo_keypoints_topic_name");
        syncString("depth_image_raw_topic_name");
        syncString("depth_image_colorized_topic_name");
        syncString("depth_point_cloud_topic_name");
        syncString("semantic_segmentation_topic_name");
        syncString("imu_raw_data_topic");
    }

    //////////////////////////////////////////////////////////////////////////
    
}
