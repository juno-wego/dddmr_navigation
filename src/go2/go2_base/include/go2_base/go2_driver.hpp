#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rmw/qos_profiles.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "go2_msgs/srv/motion_cmd.hpp"
#include "go2_msgs/msg/battery_status.hpp"
#include "go2_msgs/msg/motor_status_array.hpp"
#include "go2_msgs/msg/motor_status.hpp"

#include <unitree/robot/go2/sport/sport_client.hpp> // for basics movement
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp> // for mode change
#include <unitree/idl/go2/SportModeState_.hpp> // for state sub for high state
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp> // for subscribe dds go2
#include <unitree/robot/go2/obstacles_avoid/obstacles_avoid_client.hpp> // for obstacle avoidance
#include <unitree/idl/ros2/PointCloud2_.hpp>
#include <unitree/idl/ros2/String_.hpp>

#include <mutex>
#include <chrono>
#include <cmath>
#include <atomic>

#define MAX_SPEED_VX 3.8f
#define MIN_SPEED_VX -2.5f
#define MAX_SPEED_VY 1.0f
#define MIN_SPEED_VY -1.0f
#define MAX_SPEED_VYAW 4.0f
#define MIN_SPEED_VYAW -4.0f

#define CLAMP_SPEED_VX(value) \
    ((value) < MIN_SPEED_VX ? MIN_SPEED_VX : ((value) > MAX_SPEED_VX ? MAX_SPEED_VX : (value)))
#define CLAMP_SPEED_VY(value) \
    ((value) < MIN_SPEED_VY ? MIN_SPEED_VY : ((value) > MAX_SPEED_VY ? MAX_SPEED_VY : (value)))
#define CLAMP_SPEED_VYAW(value) \
    ((value) < MIN_SPEED_VYAW ? MIN_SPEED_VYAW : ((value) > MAX_SPEED_VYAW ? MAX_SPEED_VYAW : (value)))

# define ZERO_IF_SMALL(x) (((x) > -0.015 && (x) < 0.015) ? 0.0 : (x))
# define ZERO_IF_SMALL_RAD(x) (((x) > -0.025 && (x) < 0.025) ? 0.0 : (x))

// High-level status topic, where rt indicates real-time and lf indicates low frequency
#define TOPIC_HIGHSTATE "rt/sportmodestate"
#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_CLOUD "rt/utlidar/cloud"
#define TOPIC_FAULT "rt/errorlist"

using go2_msgs::srv::MotionCmd;
using std::placeholders::_1;
using std::placeholders::_2;

class Go2Driver : public rclcpp::Node
{
public:
    Go2Driver()
    : Node("go2_driver")
    {
        lidar_frame_id_ = this->declare_parameter<std::string>("lidar_frame_id", "utlidar");
        const double lidar_transform_x =
            this->declare_parameter<double>("lidar_transform_x", 0.0);
        const double lidar_transform_y =
            this->declare_parameter<double>("lidar_transform_y", 0.0);
        const double lidar_transform_z =
            this->declare_parameter<double>("lidar_transform_z", 0.0);
        const double lidar_transform_roll =
            this->declare_parameter<double>("lidar_transform_roll", 0.0);
        const double lidar_transform_pitch =
            this->declare_parameter<double>("lidar_transform_pitch", 0.0);
        const double lidar_transform_yaw =
            this->declare_parameter<double>("lidar_transform_yaw", 0.0);
        tf2::Quaternion lidar_transform_quat;
        lidar_transform_quat.setRPY(
            lidar_transform_roll,
            lidar_transform_pitch,
            lidar_transform_yaw);
        lidar_output_transform_.setOrigin(
            tf2::Vector3(
                lidar_transform_x,
                lidar_transform_y,
                lidar_transform_z));
        lidar_output_transform_.setRotation(lidar_transform_quat);
        transform_lidar_points_ =
            std::abs(lidar_transform_x) > 1e-9 ||
            std::abs(lidar_transform_y) > 1e-9 ||
            std::abs(lidar_transform_z) > 1e-9 ||
            std::abs(lidar_transform_roll) > 1e-9 ||
            std::abs(lidar_transform_pitch) > 1e-9 ||
            std::abs(lidar_transform_yaw) > 1e-9;

        // init the Go2 the sports client
        this->sport_client_.SetTimeout(10.0f);
        this->sport_client_.Init();

        // model change init
        this->msc_.SetTimeout(10.0f); 
        this->msc_.Init();

        // obsjtacle avoidance
        this->sc_.SetTimeout(10.0f);
        this->sc_.Init();

        // set for odom data
        this->odom_msg_.header.frame_id = "odom";
        this->odom_msg_.child_frame_id = "base_footprint";

        // tf set
        this->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // qos set for the publisher
        rclcpp::SensorDataQoS _sensor_qos;

        // create publisher and callback group of timer for high state
        this->imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("go2/imu", _sensor_qos);
        this->odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        this->robot_sm_pub_ = this->create_publisher<std_msgs::msg::Int32>("go2_mode", 10);
        this->comm_ok_pub_ = this->create_publisher<std_msgs::msg::Bool>("go2/comm_ok", rclcpp::QoS(1).transient_local());
        {
          std_msgs::msg::Bool m;
          m.data = false;
          comm_ok_pub_->publish(m);
        }

        // create publisher and callback group of timer for low state
        this->bat_pub_ = this->create_publisher<go2_msgs::msg::BatteryStatus>("go2_battery_state", 10);
        this->joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",10);
        this->motor_state_pub_ = this->create_publisher<go2_msgs::msg::MotorStatusArray>("go2_motor_state", 10);

        // the publisher for fault code
        this->faults_raw_pub_ = this->create_publisher<std_msgs::msg::String>("go2/faults_raw", rclcpp::QoS(1).transient_local());

        // create publisher for pointcloud
        this->lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "go2/lidar_points",
            rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile());

                // set publisher for odom and imu
        this->state_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // callback group setting for command
        this->cmd_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions cmd_sub_options;
        cmd_sub_options.callback_group = cmd_callback_group_;

        // Set for service of motion command
        this->go2_motion_srv_ = this->create_service<MotionCmd>(
            "go2_motion_cmd",
            std::bind(&Go2Driver::motionCmd, this, _1, _2), 
            rmw_qos_profile_services_default, 
            cmd_callback_group_);
        
        // the subscriber from go2 high state data
        this->high_state_suber_ = std::make_shared<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(TOPIC_HIGHSTATE);
        this->high_state_suber_->InitChannel(std::bind(&Go2Driver::highStateHandler, this, std::placeholders::_1));
        
        // the subscriber from go2 low state data
        this->low_state_suber_ = std::make_shared<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>>(TOPIC_LOWSTATE);
        this->low_state_suber_->InitChannel(std::bind(&Go2Driver::lowStateHandler, this, std::placeholders::_1));

        // the subscriber for go2 lidar
        this->point_cloud_suber_ = std::make_shared<unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_>>(TOPIC_CLOUD);
        this->point_cloud_suber_->InitChannel(std::bind(&Go2Driver::pointCloudHandler, this, std::placeholders::_1));

        // the subscriber for fault code
        this->fault_suber_ = std::make_shared<unitree::robot::ChannelSubscriber<std_msgs::msg::dds_::String_>>(TOPIC_FAULT);
        this->fault_suber_->InitChannel(std::bind(&Go2Driver::faultListHandler, this, std::placeholders::_1));


        this->high_state_timer_ptr_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&Go2Driver::highStateTimer, this),
            state_callback_group_
        );

        this->health_timer_ptr_ = this->create_wall_timer(
                std::chrono::milliseconds(200),
                std::bind(&Go2Driver::healthTimer, this),
                state_callback_group_
        ); 

        this->low_state_timer_ptr_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Go2Driver::lowStateTimer, this),
            state_callback_group_
        );

        // cmd_vel subscription
        this->cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 
            10, 
            std::bind(&Go2Driver::cmdCallback, this, _1),
            cmd_sub_options);
        
        this->ori_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "go2/ori_cmd",
            10,
            std::bind(&Go2Driver::oriCmdCallback, this, _1),
            cmd_sub_options
        );
        
    }

private:
    // Go2 variables for client
    unitree::robot::go2::SportClient sport_client_;
    unitree::robot::b2::MotionSwitcherClient msc_;
    unitree::robot::go2::ObstaclesAvoidClient sc_;

    // Create a Subscriber for high state
    std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>> high_state_suber_;
    unitree_go::msg::dds_::SportModeState_ latest_high_state_;
    std::mutex high_state_mutex_;
    std::atomic<bool> high_ready_{false};

    // Create at Subscriber for low state
    std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>> low_state_suber_;
    unitree_go::msg::dds_::LowState_ latest_low_state_;
    std::mutex low_state_mutex_;
    std::atomic<bool> low_ready_{false};

    // Create Subscriber for point cloud data
    std::shared_ptr<unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_>> point_cloud_suber_;

    // Create subscriber for fault code
    std::shared_ptr<unitree::robot::ChannelSubscriber<std_msgs::msg::dds_::String_>> fault_suber_;
    
    // set callback group
    rclcpp::CallbackGroup::SharedPtr cmd_callback_group_;
    rclcpp::CallbackGroup::SharedPtr state_callback_group_;

    // timer callback
    rclcpp::TimerBase::SharedPtr high_state_timer_ptr_;
    rclcpp::TimerBase::SharedPtr low_state_timer_ptr_;
    rclcpp::TimerBase::SharedPtr health_timer_ptr_;

    // publisher
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
    rclcpp::Publisher<go2_msgs::msg::BatteryStatus>::SharedPtr bat_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr robot_sm_pub_; // the state machine for go2
    rclcpp::Publisher<go2_msgs::msg::MotorStatusArray>::SharedPtr motor_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr comm_ok_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr faults_raw_pub_;
    std::string lidar_frame_id_;
    tf2::Transform lidar_output_transform_;
    bool transform_lidar_points_{false};

    // the variable for the communication check
    std::atomic<int64_t> last_high_rx_ns_{0};
    double high_timeout_sec_ =0.5;

    // Odometry
    nav_msgs::msg::Odometry odom_msg_;
    double theta_ = 0.0;
    rclcpp::Time last_stamp_;
    bool start_flag_ = true;
    
    // TF for odometry
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // cmd_vel subscription
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr ori_sub_;

    // service thingings for go2 command
    rclcpp::Service<MotionCmd>::SharedPtr go2_motion_srv_;

    float cmd_vx_, cmd_vy_, cmd_vyaw_; // velocity variable
    int cmd_error_code_;

    /*********************************************callback from ros************************************************************/
    void cmdCallback(const geometry_msgs::msg::Twist & msg)
    {
        cmd_vx_ = CLAMP_SPEED_VX(msg.linear.x);
        cmd_vy_ = CLAMP_SPEED_VY(msg.linear.y);
        cmd_vyaw_ = CLAMP_SPEED_VYAW(msg.angular.z);
        cmd_error_code_ = this->sport_client_.Move(cmd_vx_, cmd_vy_, cmd_vyaw_);
    }

    void oriCmdCallback(const geometry_msgs::msg::Vector3 & msg)
    {
        this->sport_client_.Euler(msg.x, msg.y, msg.z);
    }

    void motionCmd(const std::shared_ptr<MotionCmd::Request> req, 
                    std::shared_ptr<MotionCmd::Response> res)
    {
        if(req->task==MotionCmd::Request::DAMP){
            res->respond=this->sport_client_.Damp();
        }else if(req->task==MotionCmd::Request::BALANCE_STAND){
            res->respond=this->sport_client_.BalanceStand();
        }else if(req->task==MotionCmd::Request::STOP_MOVE){
            res->respond=this->sport_client_.StopMove();
        }else if(req->task==MotionCmd::Request::STAND_UP){
            res->respond=this->sport_client_.StandUp();
        }else if(req->task==MotionCmd::Request::STAND_DOWN){
            res->respond=this->sport_client_.StandDown();
        }else if(req->task==MotionCmd::Request::RECOVERY_STAND){
            res->respond=this->sport_client_.RecoveryStand();
        }else if(req->task==MotionCmd::Request::SIT){
            res->respond=this->sport_client_.Sit();
        }else if(req->task==MotionCmd::Request::RISE_SIT){
            res->respond=this->sport_client_.RiseSit();
        }else if(req->task==MotionCmd::Request::HELLO){
            res->respond=this->sport_client_.Hello();
        }else if(req->task==MotionCmd::Request::DISABLEAUTORECOVERY){
            // res->respond=this->sport_client_.AutoRecoverSet(false); // when the robot have payload should be disabled
            res->respond=0;
        }else if(req->task==MotionCmd::Request::ABLEAUTORECOVERY){
            // res->respond=this->sport_client_.AutoRecoverSet(true);
            res->respond=0;
        }else if(req->task==MotionCmd::Request::SETNORMALMODE){
            res->respond=this->msc_.SelectMode("normal"); 
        }else if(req->task==MotionCmd::Request::SETAIMODE){
            res->respond=this->msc_.SelectMode("ai");  // AI walking mode
        }else if(req->task==MotionCmd::Request::SETSPORTSMODE){
            res->respond=this->msc_.SelectMode("advanced"); // sports
        }else if(req->task==MotionCmd::Request::SETAVOIDANCE){
            res->respond=this->sc_.SwitchSet(true); // set avoidance
        }else if(req->task==MotionCmd::Request::UNSETAVOIDANCE){
            res->respond=this->sc_.SwitchSet(false); // unset avoidance
        }else if(req->task==MotionCmd::Request::SETSTRIKEPOSE){
            res->respond=this->sport_client_.Pose(true);
        }else if(req->task==MotionCmd::Request::UNSETSTRIKEPOSE){
            res->respond=this->sport_client_.Pose(false);
        }
        else{
            res->respond=this->sport_client_.Damp();
        }
    }

    /*********************************************Subscriber handler from dds************************************************************/
    void highStateHandler(const void* message)
    {
        {
            std::lock_guard<std::mutex> lock(high_state_mutex_); // to avoid race
            this->latest_high_state_ = *(unitree_go::msg::dds_::SportModeState_*)message;
        }
        high_ready_.store(true, std::memory_order_release);
        last_high_rx_ns_.store(this->get_clock()->now().nanoseconds(), std::memory_order_relaxed);
    }

    void lowStateHandler(const void* message)
    {
        {
            std::lock_guard<std::mutex> lock(low_state_mutex_); // to avoid race
            this->latest_low_state_ = *(unitree_go::msg::dds_::LowState_*)message;
        }
        low_ready_.store(true, std::memory_order_release);
    }

    void pointCloudHandler(const void* message)
    {
        const auto* dds_msg = static_cast<const sensor_msgs::msg::dds_::PointCloud2_*>(message);
        sensor_msgs::msg::PointCloud2 ros_msg;
        ros_msg.header.frame_id = lidar_frame_id_;
        ros_msg.header.stamp = this->get_clock()->now();

        ros_msg.height = dds_msg->height();
        ros_msg.width = dds_msg->width();
        ros_msg.is_bigendian = static_cast<bool>(dds_msg->is_bigendian());
        ros_msg.point_step = static_cast<uint32_t>(dds_msg->point_step());
        ros_msg.row_step = static_cast<uint32_t>(dds_msg->row_step());
        ros_msg.is_dense = static_cast<bool>(dds_msg->is_dense());

        ros_msg.data.assign(dds_msg->data().begin(), dds_msg->data().end());
        
        ros_msg.fields.clear();
        for (const auto& dds_field : dds_msg->fields())
        {
            sensor_msgs::msg::PointField ros_field;
            ros_field.name = dds_field.name();
            ros_field.offset = static_cast<uint32_t>(dds_field.offset());
            ros_field.datatype =  static_cast<decltype(ros_field.datatype)>(dds_field.datatype());
            ros_field.count = static_cast<uint32_t>(dds_field.count());
            ros_msg.fields.push_back(ros_field);
        }

        if (transform_lidar_points_) {
            sensor_msgs::PointCloud2Iterator<float> iter_x(ros_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(ros_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(ros_msg, "z");
            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
                tf2::Vector3 point(*iter_x, *iter_y, *iter_z);
                point = lidar_output_transform_ * point;
                *iter_x = static_cast<float>(point.x());
                *iter_y = static_cast<float>(point.y());
                *iter_z = static_cast<float>(point.z());
            }
        }

        this->lidar_pub_->publish(ros_msg);
    }

    void faultListHandler(const void* message)
    {
        const auto* dds_msg = static_cast<const std_msgs::msg::dds_::String_*>(message);
        const std::string payload = dds_msg->data();  // JSON: {"errors":[[ts, src, code], ...]}

        std_msgs::msg::String raw;
        raw.data = payload;
        faults_raw_pub_->publish(raw);
    }

    /*********************************************The Timer publisher************************************************************/
    void highStateTimer()
    {
        if(!high_ready_.load(std::memory_order_acquire)) return;

        unitree_go::msg::dds_::SportModeState_ hs;
        {
            std::lock_guard<std::mutex> lk(high_state_mutex_);
            hs = latest_high_state_;
        }
        sensor_msgs::msg::Imu _imu_msg;
        
        rclcpp::Time now = this->get_clock()->now();
        double dt;
        if(this->start_flag_){
            dt = 0.0;
            this->start_flag_ = false;
        }else{
            dt = (now - this->last_stamp_).seconds();
        }
        last_stamp_ = now;
        
        // imu calc
        _imu_msg.header.frame_id = "imu";
        _imu_msg.header.stamp=this->get_clock()->now();

        _imu_msg.orientation.w = hs.imu_state().quaternion()[0];
        _imu_msg.orientation.x = hs.imu_state().quaternion()[1];
        _imu_msg.orientation.y = hs.imu_state().quaternion()[2];
        _imu_msg.orientation.z = hs.imu_state().quaternion()[3];

        _imu_msg.orientation_covariance[0] = 1e-3;
        _imu_msg.orientation_covariance[4] = 1e-3;
        _imu_msg.orientation_covariance[8] = 1e-3;

        _imu_msg.angular_velocity.x = hs.imu_state().gyroscope()[0];
        _imu_msg.angular_velocity.y = hs.imu_state().gyroscope()[1];
        _imu_msg.angular_velocity.z = hs.imu_state().gyroscope()[2];

        _imu_msg.linear_acceleration.x = hs.imu_state().accelerometer()[0];
        _imu_msg.linear_acceleration.y = hs.imu_state().accelerometer()[1];
        _imu_msg.linear_acceleration.z = hs.imu_state().accelerometer()[2];

        _imu_msg.angular_velocity_covariance[0] = 5.0e-5;
        _imu_msg.angular_velocity_covariance[4] = 7.5e-5;
        _imu_msg.angular_velocity_covariance[8] = 6.0e-5;

        _imu_msg.linear_acceleration_covariance[0] = 0.002;
        _imu_msg.linear_acceleration_covariance[4] = 0.002;
        _imu_msg.linear_acceleration_covariance[8] = 0.002;
        
        // odom calc
        this->odom_msg_.header.stamp = this->get_clock()->now();

        auto truncate_2digits = [](double val) {
            return std::trunc(val * 100.0) / 100.0;
        };

        this->odom_msg_.twist.twist.linear.x = ZERO_IF_SMALL(truncate_2digits(hs.velocity()[0]));
        this->odom_msg_.twist.twist.linear.y = ZERO_IF_SMALL(truncate_2digits(hs.velocity()[1]));
        this->odom_msg_.twist.twist.linear.z = ZERO_IF_SMALL(truncate_2digits(hs.velocity()[2]));

        this->odom_msg_.twist.covariance[0] = 2.5e-4;
        this->odom_msg_.twist.covariance[7] = 2.5e-4;
        this->odom_msg_.twist.covariance[13] = 2.5e-4;

        this->odom_msg_.twist.twist.angular.z = ZERO_IF_SMALL_RAD(hs.yaw_speed());

        this->odom_msg_.twist.covariance[35] = 1e-4;
        
        this->theta_ += this->odom_msg_.twist.twist.angular.z * dt;

        while(this->theta_ <= -3.141592){this->theta_ += 6.283184;}
        while(this->theta_ >= 3.141592){this->theta_ -= 6.283184;}

        this->odom_msg_.pose.pose.position.x += cos(this->theta_) * odom_msg_.twist.twist.linear.x * dt -
                                               sin(this->theta_) * odom_msg_.twist.twist.linear.y * dt;
        this->odom_msg_.pose.pose.position.y += sin(this->theta_) * odom_msg_.twist.twist.linear.x * dt +
                                               cos(this->theta_) * odom_msg_.twist.twist.linear.y * dt;
        this->odom_msg_.pose.pose.position.z = hs.position()[2];

        tf2::Quaternion _q_tf;
        _q_tf.setRPY(0, 0, this->theta_);
        geometry_msgs::msg::Quaternion _q_msg = tf2::toMsg(_q_tf);
        this->odom_msg_.pose.pose.orientation = _q_msg;

        // clac tf just for some debugging
        geometry_msgs::msg::TransformStamped _t;
        _t.header.stamp = this->get_clock()->now();
        _t.header.frame_id = "odom";
        _t.child_frame_id = "base_footprint";

        _t.transform.translation.x = this->odom_msg_.pose.pose.position.x;
        _t.transform.translation.y = this->odom_msg_.pose.pose.position.y;
        _t.transform.translation.z = this->odom_msg_.pose.pose.position.z;

        _t.transform.rotation.x = _q_tf.x();
        _t.transform.rotation.y = _q_tf.y();
        _t.transform.rotation.z = _q_tf.z();
        _t.transform.rotation.w = _q_tf.w();

        this->tf_broadcaster_->sendTransform(_t);

        std_msgs::msg::Int32 state_code;
        state_code.data = hs.error_code();

        this->robot_sm_pub_->publish(state_code);
        this->odom_pub_->publish(this->odom_msg_);
        this->imu_pub_->publish(_imu_msg);
    }

    void healthTimer()
    {
        const int64_t now_ns = this->get_clock()->now().nanoseconds();
        const int64_t hi_ns  = last_high_rx_ns_.load(std::memory_order_relaxed);

        const bool got_hi = (hi_ns > 0);
        const double age_hi = got_hi ? (now_ns - hi_ns) * 1e-9 : 1e9; // 한 번도 못 받았으면 아주 큰 값

        std_msgs::msg::Bool msg;
        msg.data = got_hi && (age_hi <= high_timeout_sec_);
        comm_ok_pub_->publish(msg);
    }

    void lowStateTimer()
    {
        if(!low_ready_.load(std::memory_order_acquire)) return;

        unitree_go::msg::dds_::LowState_ ls;
        {
            std::lock_guard<std::mutex> lk(low_state_mutex_);
            ls = latest_low_state_;
        }

        go2_msgs::msg::BatteryStatus _battery_state;
        sensor_msgs::msg::JointState _joint_state;
        go2_msgs::msg::MotorStatus _motor_state;
        go2_msgs::msg::MotorStatusArray _motors_state;

        _battery_state.soc = ls.bms_state().soc();
        _battery_state.current = ls.bms_state().current();
        _battery_state.cycle = ls.bms_state().cycle();
        _battery_state.battery_temp_1 = ls.bms_state().bq_ntc()[0];
        _battery_state.battery_temp_2 = ls.bms_state().bq_ntc()[1];

        _joint_state.header.stamp = this->get_clock()->now();
        _joint_state.name.push_back("FL_hip_joint");
        _joint_state.position.push_back(ls.motor_state()[3].q());
        _motor_state.joint_name = "FL_hip_joint";
        _motor_state.q = ls.motor_state()[3].q();
        _motor_state.dq = ls.motor_state()[3].dq();
        _motor_state.ddq = ls.motor_state()[3].ddq();
        _motor_state.temperature = ls.motor_state()[3].temperature();
        _motors_state.motor_states.push_back(_motor_state);

        _joint_state.name.push_back("FL_thigh_joint");
        _joint_state.position.push_back(ls.motor_state()[4].q());
        _motor_state.joint_name = "FL_thigh_joint";
        _motor_state.q = ls.motor_state()[4].q();
        _motor_state.dq = ls.motor_state()[4].dq();
        _motor_state.ddq = ls.motor_state()[4].ddq();
        _motor_state.temperature = ls.motor_state()[4].temperature();
        _motors_state.motor_states.push_back(_motor_state);

        _joint_state.name.push_back("FL_calf_joint");
        _joint_state.position.push_back(ls.motor_state()[5].q());
        _motor_state.joint_name = "FL_calf_joint";
        _motor_state.q = ls.motor_state()[5].q();
        _motor_state.dq = ls.motor_state()[5].dq();
        _motor_state.ddq = ls.motor_state()[5].ddq();
        _motor_state.temperature = ls.motor_state()[5].temperature();
        _motors_state.motor_states.push_back(_motor_state);

        _joint_state.name.push_back("FR_hip_joint");
        _joint_state.position.push_back(ls.motor_state()[0].q());
        _motor_state.joint_name = "FR_hip_joint";
        _motor_state.q = ls.motor_state()[0].q();
        _motor_state.dq = ls.motor_state()[0].dq();
        _motor_state.ddq = ls.motor_state()[0].ddq();
        _motor_state.temperature = ls.motor_state()[0].temperature();
        _motors_state.motor_states.push_back(_motor_state);

        _joint_state.name.push_back("FR_thigh_joint");
        _joint_state.position.push_back(ls.motor_state()[1].q());
        _motor_state.joint_name = "FR_thigh_joint";
        _motor_state.q = ls.motor_state()[1].q();
        _motor_state.dq = ls.motor_state()[1].dq();
        _motor_state.ddq = ls.motor_state()[1].ddq();
        _motor_state.temperature = ls.motor_state()[1].temperature();
        _motors_state.motor_states.push_back(_motor_state);

        _joint_state.name.push_back("FR_calf_joint");
        _joint_state.position.push_back(ls.motor_state()[2].q());
        _motor_state.joint_name = "FR_calf_joint";
        _motor_state.q = ls.motor_state()[2].q();
        _motor_state.dq = ls.motor_state()[2].dq();
        _motor_state.ddq = ls.motor_state()[2].ddq();
        _motor_state.temperature = ls.motor_state()[2].temperature();
        _motors_state.motor_states.push_back(_motor_state);

        _joint_state.name.push_back("RL_hip_joint");
        _joint_state.position.push_back(ls.motor_state()[9].q());
        _motor_state.joint_name = "RL_hip_joint";
        _motor_state.q = ls.motor_state()[9].q();
        _motor_state.dq = ls.motor_state()[9].dq();
        _motor_state.ddq = ls.motor_state()[9].ddq();
        _motor_state.temperature = ls.motor_state()[9].temperature();
        _motors_state.motor_states.push_back(_motor_state);

        _joint_state.name.push_back("RL_thigh_joint");
        _joint_state.position.push_back(ls.motor_state()[10].q());
        _motor_state.joint_name = "RL_thigh_joint";
        _motor_state.q = ls.motor_state()[10].q();
        _motor_state.dq = ls.motor_state()[10].dq();
        _motor_state.ddq = ls.motor_state()[10].ddq();
        _motor_state.temperature = ls.motor_state()[10].temperature();
        _motors_state.motor_states.push_back(_motor_state);

        _joint_state.name.push_back("RL_calf_joint");
        _joint_state.position.push_back(ls.motor_state()[11].q());
        _motor_state.joint_name = "RL_calf_joint";
        _motor_state.q = ls.motor_state()[11].q();
        _motor_state.dq = ls.motor_state()[11].dq();
        _motor_state.ddq = ls.motor_state()[11].ddq();
        _motor_state.temperature = ls.motor_state()[11].temperature();
        _motors_state.motor_states.push_back(_motor_state);

        _joint_state.name.push_back("RR_hip_joint");
        _joint_state.position.push_back(ls.motor_state()[6].q());
        _motor_state.joint_name = "RR_hip_joint";
        _motor_state.q = ls.motor_state()[6].q();
        _motor_state.dq = ls.motor_state()[6].dq();
        _motor_state.ddq = ls.motor_state()[6].ddq();
        _motor_state.temperature = ls.motor_state()[6].temperature();
        _motors_state.motor_states.push_back(_motor_state);

        _joint_state.name.push_back("RR_thigh_joint");
        _joint_state.position.push_back(ls.motor_state()[7].q());
        _motor_state.joint_name = "RR_thigh_joint";
        _motor_state.q = ls.motor_state()[7].q();
        _motor_state.dq = ls.motor_state()[7].dq();
        _motor_state.ddq = ls.motor_state()[7].ddq();
        _motor_state.temperature = ls.motor_state()[7].temperature();
        _motors_state.motor_states.push_back(_motor_state);

        _joint_state.name.push_back("RR_calf_joint");
        _joint_state.position.push_back(ls.motor_state()[8].q());
        _motor_state.joint_name = "RR_calf_joint";
        _motor_state.q = ls.motor_state()[8].q();
        _motor_state.dq = ls.motor_state()[8].dq();
        _motor_state.ddq = ls.motor_state()[8].ddq();
        _motor_state.temperature = ls.motor_state()[8].temperature();
        _motors_state.motor_states.push_back(_motor_state);

        _joint_state.velocity.resize(_joint_state.name.size(), 0.0);
        _joint_state.effort.resize(_joint_state.name.size(), 0.0);

        this->motor_state_pub_->publish(_motors_state);
        this->bat_pub_->publish(_battery_state);
        this->joint_pub_->publish(_joint_state);
    }
};
