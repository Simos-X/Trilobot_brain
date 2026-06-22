#ifndef DIFFDRIVE_ARDUINO__DIFFBOT_SYSTEM_HPP_
#define DIFFDRIVE_ARDUINO__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "diffdrive_arduino/visibility_control.h"
#include "diffdrive_arduino/arduino_comms.hpp"
#include "diffdrive_arduino/wheel.hpp"

#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <mutex>

namespace diffdrive_arduino
{
  class DiffDriveArduinoHardware : public hardware_interface::SystemInterface
  {
    struct Config
    {
      std::string left_wheel_name = "";
      std::string right_wheel_name = "";
      std::vector<std::string> servo_names;
      std::vector<int> servo_channels;
      float loop_rate = 0.0;
      std::string device = "";
      int baud_rate = 0;
      int timeout_ms = 0;
      int enc_counts_per_rev = 0;
      float pid_p = 0;
      float pid_d = 0;
      float pid_i = 0;
      float pid_o = 0;
    };

    struct Servo
    {
      std::string name = "";
      double cmd = 0.0;
      double pos = 0.0;
      double vel = 0.0;
      int channel = 0;
    };

    public:
      RCLCPP_SHARED_PTR_DEFINITIONS(DiffDriveArduinoHardware);

      DIFFDRIVE_ARDUINO_PUBLIC
      hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

      DIFFDRIVE_ARDUINO_PUBLIC
      std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

      DIFFDRIVE_ARDUINO_PUBLIC
      std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

      DIFFDRIVE_ARDUINO_PUBLIC
      hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

      DIFFDRIVE_ARDUINO_PUBLIC
      hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State & previous_state) override;

      DIFFDRIVE_ARDUINO_PUBLIC
      hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

      DIFFDRIVE_ARDUINO_PUBLIC
      hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

      DIFFDRIVE_ARDUINO_PUBLIC
      hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

      DIFFDRIVE_ARDUINO_PUBLIC
      hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

      void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
      void chassis_imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

    private:
      ArduinoComms comms_;
      Config cfg_;
      Wheel wheel_l_;
      Wheel wheel_r_;
      std::vector<Servo> servos_;
      std::vector<double> servo_offsets_deg_;
      std::shared_ptr<rclcpp::Node> node_;
      rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
      sensor_msgs::msg::Imu::SharedPtr last_imu_msg_;
      std::mutex imu_mutex_;
      rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr chassis_imu_pub_;
      rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr chassis_imu_mag_pub_;
      rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr chassis_imu_sub_;
      sensor_msgs::msg::Imu::SharedPtr last_chassis_imu_msg_;
      std::mutex chassis_imu_mutex_;
      void publish_chassis_imu();
      void publish_chassis_imu_mag();
      tf2::Quaternion q_offset_;
      size_t head_yaw_idx_ = static_cast<size_t>(-1);
      size_t head_pitch_idx_ = static_cast<size_t>(-1);
      std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
      std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  };

}  

#endif  // DIFFDRIVE_ARDUINO__DIFFBOT_SYSTEM_HPP_