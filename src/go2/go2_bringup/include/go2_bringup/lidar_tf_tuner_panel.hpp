#pragma once

#include <array>
#include <memory>
#include <string>

#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace go2_bringup
{

class LidarTFTunerPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit LidarTFTunerPanel(QWidget * parent = nullptr);

  void onInitialize() override;

private Q_SLOTS:
  void publishTransforms();
  void resetUtlidar();
  void resetLivox();

private:
  struct SensorDefaults
  {
    std::string title;
    std::string parent_frame;
    std::string child_frame;
    std::array<double, 3> xyz;
    std::array<double, 3> rpy_rad;
  };

  struct SensorControls
  {
    SensorDefaults defaults;
    std::array<QDoubleSpinBox *, 3> xyz;
    std::array<QSlider *, 3> rpy_sliders;
    std::array<QLabel *, 3> rpy_values;
    QLabel * ros_args;
  };

  QGroupBox * makeSensorGroup(SensorControls & controls, const SensorDefaults & defaults);
  void resetSensor(SensorControls & controls);
  void updateSensorLabels(SensorControls & controls);
  void publishSensorTransform(const SensorControls & controls);
  double sliderToRadians(const QSlider * slider) const;

  SensorControls utlidar_;
  SensorControls livox_;

  rclcpp::Node::SharedPtr raw_node_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  QTimer * publish_timer_;
};

}  // namespace go2_bringup
