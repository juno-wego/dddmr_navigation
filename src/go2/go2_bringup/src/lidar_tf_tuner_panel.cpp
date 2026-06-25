#include "go2_bringup/lidar_tf_tuner_panel.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

#include <QGridLayout>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace go2_bringup
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

std::string formatDouble(const double value, const int precision)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}
}  // namespace

LidarTFTunerPanel::LidarTFTunerPanel(QWidget * parent)
: rviz_common::Panel(parent),
  publish_timer_(new QTimer(this))
{
  auto * layout = new QVBoxLayout();
  layout->setContentsMargins(4, 4, 4, 4);

  QLabel * note = new QLabel(
    "Publishes live base_link TF overrides. Copy final values back to URDF/launch.");
  note->setWordWrap(true);
  layout->addWidget(note);

  layout->addWidget(makeSensorGroup(
    utlidar_,
    SensorDefaults{
      "Go2 UT Lidar",
      "base_link",
      "utlidar",
      {0.3, 0.0, -0.03},
      {0.22, -3.00, 2.05}}));

  layout->addWidget(makeSensorGroup(
    livox_,
    SensorDefaults{
      "Livox MID360",
      "base_link",
      "livox_frame",
      {0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0}}));

  layout->addStretch(1);
  setLayout(layout);

  connect(publish_timer_, &QTimer::timeout, this, &LidarTFTunerPanel::publishTransforms);
}

void LidarTFTunerPanel::onInitialize()
{
  raw_node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(raw_node_);
  publish_timer_->start(100);
  publishTransforms();
}

QGroupBox * LidarTFTunerPanel::makeSensorGroup(
  SensorControls & controls,
  const SensorDefaults & defaults)
{
  controls.defaults = defaults;

  auto * group = new QGroupBox(QString::fromStdString(defaults.title));
  auto * grid = new QGridLayout();

  grid->addWidget(new QLabel("frame"), 0, 0);
  grid->addWidget(new QLabel(QString::fromStdString(
    defaults.parent_frame + " -> " + defaults.child_frame)), 0, 1, 1, 3);

  const std::array<const char *, 3> xyz_names = {"x", "y", "z"};
  for (int i = 0; i < 3; ++i) {
    auto * label = new QLabel(xyz_names[i]);
    auto * spin = new QDoubleSpinBox();
    spin->setRange(-5.0, 5.0);
    spin->setDecimals(3);
    spin->setSingleStep(0.01);
    spin->setSuffix(" m");
    controls.xyz[i] = spin;
    grid->addWidget(label, 1, i);
    grid->addWidget(spin, 2, i);
    connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this, &controls]() {
      updateSensorLabels(controls);
      publishTransforms();
    });
  }

  const std::array<const char *, 3> rpy_names = {"roll", "pitch", "yaw"};
  for (int i = 0; i < 3; ++i) {
    auto * name = new QLabel(rpy_names[i]);
    auto * value = new QLabel();
    auto * slider = new QSlider(Qt::Horizontal);
    slider->setRange(-1800, 1800);
    slider->setSingleStep(1);
    slider->setPageStep(10);
    slider->setTickInterval(300);
    slider->setTickPosition(QSlider::TicksBelow);
    controls.rpy_sliders[i] = slider;
    controls.rpy_values[i] = value;

    const int row = 3 + i;
    grid->addWidget(name, row, 0);
    grid->addWidget(slider, row, 1, 1, 2);
    grid->addWidget(value, row, 3);

    connect(slider, &QSlider::valueChanged, this, [this, &controls]() {
      updateSensorLabels(controls);
      publishTransforms();
    });
  }

  auto * reset = new QPushButton("Reset");
  if (defaults.child_frame == "utlidar") {
    connect(reset, &QPushButton::clicked, this, &LidarTFTunerPanel::resetUtlidar);
  } else {
    connect(reset, &QPushButton::clicked, this, &LidarTFTunerPanel::resetLivox);
  }

  controls.ros_args = new QLabel();
  controls.ros_args->setTextInteractionFlags(Qt::TextSelectableByMouse);
  controls.ros_args->setWordWrap(true);

  grid->addWidget(reset, 6, 0);
  grid->addWidget(controls.ros_args, 6, 1, 1, 3);

  group->setLayout(grid);
  resetSensor(controls);
  return group;
}

void LidarTFTunerPanel::resetUtlidar()
{
  resetSensor(utlidar_);
  publishTransforms();
}

void LidarTFTunerPanel::resetLivox()
{
  resetSensor(livox_);
  publishTransforms();
}

void LidarTFTunerPanel::resetSensor(SensorControls & controls)
{
  for (int i = 0; i < 3; ++i) {
    controls.xyz[i]->setValue(controls.defaults.xyz[i]);
    controls.rpy_sliders[i]->setValue(
      static_cast<int>(std::round(controls.defaults.rpy_rad[i] * kRadToDeg * 10.0)));
  }
  updateSensorLabels(controls);
}

void LidarTFTunerPanel::updateSensorLabels(SensorControls & controls)
{
  std::array<double, 3> rpy{};
  for (int i = 0; i < 3; ++i) {
    rpy[i] = sliderToRadians(controls.rpy_sliders[i]);
    const double deg = controls.rpy_sliders[i]->value() / 10.0;
    controls.rpy_values[i]->setText(QString::fromStdString(
      formatDouble(deg, 1) + " deg / " + formatDouble(rpy[i], 4) + " rad"));
  }

  controls.ros_args->setText(QString::fromStdString(
    "args: " +
    formatDouble(controls.xyz[0]->value(), 3) + " " +
    formatDouble(controls.xyz[1]->value(), 3) + " " +
    formatDouble(controls.xyz[2]->value(), 3) + " " +
    formatDouble(rpy[0], 6) + " " +
    formatDouble(rpy[1], 6) + " " +
    formatDouble(rpy[2], 6) + " " +
    controls.defaults.parent_frame + " " +
    controls.defaults.child_frame));
}

double LidarTFTunerPanel::sliderToRadians(const QSlider * slider) const
{
  return (slider->value() / 10.0) * kDegToRad;
}

void LidarTFTunerPanel::publishTransforms()
{
  if (!tf_broadcaster_) {
    return;
  }
  publishSensorTransform(utlidar_);
  publishSensorTransform(livox_);
}

void LidarTFTunerPanel::publishSensorTransform(const SensorControls & controls)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = raw_node_->now();
  transform.header.frame_id = controls.defaults.parent_frame;
  transform.child_frame_id = controls.defaults.child_frame;
  transform.transform.translation.x = controls.xyz[0]->value();
  transform.transform.translation.y = controls.xyz[1]->value();
  transform.transform.translation.z = controls.xyz[2]->value();

  tf2::Quaternion quaternion;
  quaternion.setRPY(
    sliderToRadians(controls.rpy_sliders[0]),
    sliderToRadians(controls.rpy_sliders[1]),
    sliderToRadians(controls.rpy_sliders[2]));
  quaternion.normalize();

  transform.transform.rotation.x = quaternion.x();
  transform.transform.rotation.y = quaternion.y();
  transform.transform.rotation.z = quaternion.z();
  transform.transform.rotation.w = quaternion.w();

  tf_broadcaster_->sendTransform(transform);
}

}  // namespace go2_bringup

PLUGINLIB_EXPORT_CLASS(go2_bringup::LidarTFTunerPanel, rviz_common::Panel)
