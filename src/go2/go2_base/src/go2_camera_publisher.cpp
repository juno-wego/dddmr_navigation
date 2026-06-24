#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/header.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv4/opencv2/opencv.hpp"
#include "camera_info_manager/camera_info_manager.hpp"

class ScopedCapture {
public:
  explicit ScopedCapture(cv::VideoCapture& cap) : cap_(cap) {}
  ~ScopedCapture() { if (cap_.isOpened()) cap_.release(); }
private:
  cv::VideoCapture& cap_;
};

class Go2CameraPulibhser : public rclcpp::Node
{
public:
    Go2CameraPulibhser()
    : Node("go2_camera_publisher")
    {
        pipeline_ = declare_parameter<std::string>(
        "pipeline",
        "udpsrc address=230.1.1.1 port=1720 multicast-iface=enp88s0 ! \
        application/x-rtp, media=video, encoding-name=H264 ! \
        rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! \
        video/x-raw,width=1280,height=720,format=BGR ! appsink drop=1");
    
        frame_id_ = declare_parameter<std::string>("frame_id", "front_camera");
        publish_compressed_ = declare_parameter<bool>("publish_compressed", true);
        fps_ = declare_parameter<double>("fps", 30.0);
        topic_base_ = declare_parameter<std::string>("topic_base", "/go2/camera");
        
        camera_name_ = declare_parameter<std::string>("camera_name", "/go2/camera");
        camera_info_url_ = declare_parameter<std::string>(
            "camera_info_url",
            "file:///home/wego/camera/go2_camera_info.yaml"
        );

        pub_raw_ = this->create_publisher<sensor_msgs::msg::Image>(
        topic_base_ + "/image_raw", 10);
        if (publish_compressed_) {
            pub_comp_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            topic_base_ + "/image_raw/compressed", 10);
        }
        pub_info_ = create_publisher<sensor_msgs::msg::CameraInfo>(topic_base_ + "/camera_info", 10);

        cinfo_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, camera_name_, camera_info_url_);
        if (!cinfo_->isCalibrated()) {
            RCLCPP_WARN(get_logger(), "CameraInfo not calibrated or YAML not found: %s", camera_info_url_.c_str());
        } else {
            RCLCPP_INFO(get_logger(), "Loaded camera info: name=%s, url=%s", camera_name_.c_str(), camera_info_url_.c_str());
        }


        cap_.open(pipeline_, cv::CAP_GSTREAMER);
        if (!cap_.isOpened()) {
          RCLCPP_FATAL(get_logger(), "Failed to open GStreamer pipeline.\nPipeline: %s", pipeline_.c_str());
          throw std::runtime_error("VideoCapture open failed");
        }
        RCLCPP_INFO(get_logger(), "Opened Go2 camera stream via GStreamer.");

        // 타이머 루프 (fps에 맞춰 cap.read)
        using namespace std::chrono_literals;
        auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(1.0, fps_)));
        timer_ = this->create_wall_timer(period, std::bind(&Go2CameraPulibhser::grabAndPublish, this));
    
    }


private:
    void grabAndPublish() {
        cv::Mat frame;
        if (!cap_.read(frame) || frame.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 3000,
                               "Empty frame or read() failed.");
            return;
        }
      
        // ROS Image (BGR8)
        auto stamp = this->now();
        std_msgs::msg::Header header;
        header.stamp = stamp;
        header.frame_id = frame_id_;
      
        auto img_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        pub_raw_->publish(*img_msg);

        auto cam_info = cinfo_->getCameraInfo();
        cam_info.header = header;
        pub_info_->publish(cam_info);
      
        // 옵션: CompressedImage (JPEG)
        if (publish_compressed_ && pub_comp_) {
            std::vector<uint8_t> buf;
            std::vector<int> params = { cv::IMWRITE_JPEG_QUALITY, 90 };
            cv::imencode(".jpg", frame, buf, params);

            sensor_msgs::msg::CompressedImage cmsg;
            cmsg.header = header;
            cmsg.format = "jpeg";
            cmsg.data = std::move(buf);
            pub_comp_->publish(cmsg);
        }
    }

    std::string pipeline_;
    std::string frame_id_;
    std::string topic_base_;
    bool publish_compressed_{true};
    double fps_{30.0};

    std::string camera_name_;
    std::string camera_info_url_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_raw_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_comp_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_;
    rclcpp::TimerBase::SharedPtr timer_;

    // video
    cv::VideoCapture cap_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<Go2CameraPulibhser>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        fprintf(stderr, "Fatal: %s\n", e.what());
    }
    rclcpp::shutdown();

    return 0;
}