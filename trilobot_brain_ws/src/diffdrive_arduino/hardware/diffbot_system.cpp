// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "diffdrive_arduino/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <sstream>
#include <algorithm>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <mutex>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <thread>

namespace diffdrive_arduino
{

  hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_init(const hardware_interface::HardwareInfo & info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
    cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
    cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
    cfg_.device = info_.hardware_parameters["device"];
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
    cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

    // Parse servo names 
    std::string servo_names_str = info_.hardware_parameters["servo_names"];
    std::stringstream ss_names(servo_names_str);
    std::string name;
    while (std::getline(ss_names, name, ',')) {
      cfg_.servo_names.push_back(name);
    }

    // Parse servo channels 
    std::string servo_channels_str = info_.hardware_parameters["servo_channels"];
    std::stringstream ss_channels(servo_channels_str);
    std::string channel_str;
    while (std::getline(ss_channels, channel_str, ',')) {
      cfg_.servo_channels.push_back(std::stoi(channel_str));
    }

    if (cfg_.servo_names.size() != cfg_.servo_channels.size()) {
      RCLCPP_FATAL(rclcpp::get_logger("DiffDriveArduinoHardware"), "Number of servo names and channels do not match.");
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (info_.hardware_parameters.count("pid_p") > 0)
    {
      cfg_.pid_p = std::stof(info_.hardware_parameters["pid_p"]);
      cfg_.pid_d = std::stof(info_.hardware_parameters["pid_d"]);
      cfg_.pid_i = std::stof(info_.hardware_parameters["pid_i"]);
      cfg_.pid_o = std::stof(info_.hardware_parameters["pid_o"]);
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "PID values not supplied, using defaults.");
    }

    wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
    wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

    servos_.resize(cfg_.servo_names.size());
    for (size_t i = 0; i < servos_.size(); ++i) {
      servos_[i].name = cfg_.servo_names[i];
      servos_[i].channel = cfg_.servo_channels[i];
      if (servos_[i].name == "head_yaw_joint") {
        head_yaw_idx_ = i;
      } else if (servos_[i].name == "head_pitch_joint") {
        head_pitch_idx_ = i;
      }
    }

    // Define per-joint offsets in degrees so initial positions maps to 0 rad
    servo_offsets_deg_ = {80.0, 35.0, 125.0, 120.0};

    // IMU mounting offset for OAK-D-S2: roll=π, pitch=π/2, yaw=0
    double offset_yaw = 0.0;
    double offset_pitch = M_PI / 2.0;
    double offset_roll = M_PI;

    double cy = cos(offset_yaw * 0.5);
    double sy = sin(offset_yaw * 0.5);
    double cp = cos(offset_pitch * 0.5);
    double sp = sin(offset_pitch * 0.5);
    double cr = cos(offset_roll * 0.5);
    double sr = sin(offset_roll * 0.5);

    q_offset_.setValue(
      sr * cp * cy - cr * sp * sy,
      cr * sp * cy + sr * cp * sy,
      cr * cp * sy - sr * sp * cy,
      cr * cp * cy + sr * sp * sy
    );

    node_ = std::make_shared<rclcpp::Node>("diffdrive_arduino_hw");

    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("DiffDriveArduinoHardware"),
          "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
          joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION)
      {

      } else {
        RCLCPP_FATAL(
          rclcpp::get_logger("DiffDriveArduinoHardware"),
          "Joint '%s' has invalid command interface '%s'. Expected velocity or position.",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("DiffDriveArduinoHardware"),
          "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("DiffDriveArduinoHardware"),
          "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("DiffDriveArduinoHardware"),
          "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> DiffDriveArduinoHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

    for (auto &servo : servos_) {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        servo.name, hardware_interface::HW_IF_POSITION, &servo.pos));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        servo.name, hardware_interface::HW_IF_VELOCITY, &servo.vel)); 
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> DiffDriveArduinoHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

    for (auto &servo : servos_) {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        servo.name, hardware_interface::HW_IF_POSITION, &servo.cmd));
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Configuring ...please wait...");

    if (comms_.connected())
    {
      comms_.disconnect();
    }

    comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);

    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10, std::bind(&DiffDriveArduinoHardware::imu_callback, this, std::placeholders::_1));

    chassis_imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("/chassis_imu/raw", 10);
    chassis_imu_mag_pub_ = node_->create_publisher<sensor_msgs::msg::MagneticField>("/chassis_imu/mag", 10);
    chassis_imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>("/chassis_imu/data", 10, std::bind(&DiffDriveArduinoHardware::chassis_imu_callback, this, std::placeholders::_1));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully configured!");

    comms_.set_servo_angles(80.0f, 35.0f, 125.0f, 120.0f);
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Cleaning up ...please wait...");

    if (comms_.connected())
    {
      comms_.disconnect();
    }

    imu_sub_.reset();

    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully cleaned up!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Activating ...please wait...");

    if (!comms_.connected())
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (cfg_.pid_p > 0)
    {
      comms_.set_pid_values(cfg_.pid_p,cfg_.pid_d,cfg_.pid_i,cfg_.pid_o);
    }

    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Deactivating ...please wait...");

    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  void DiffDriveArduinoHardware::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    last_imu_msg_ = msg;
  }

  void DiffDriveArduinoHardware::chassis_imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(chassis_imu_mutex_);
    last_chassis_imu_msg_ = msg;
  }

  void DiffDriveArduinoHardware::publish_chassis_imu()
  {
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    float mx = 0.0f, my = 0.0f, mz = 0.0f;
    float calib_ax = 0.0f, calib_ay = 0.0f, calib_az = 0.0f;
    float calib_gx = 0.0f, calib_gy = 0.0f, calib_gz = 0.0f;
    float accel_offsets[3]={-0.606,0.208,-0.411},gyro_offsets[3]={-0.058,0.004,0.001};

    comms_.read_chassis_imu(ax, ay, az, gx, gy, gz, mx, my, mz);

    calib_ax = ax - accel_offsets[0];
    calib_ay = ay - accel_offsets[1];
    calib_az = az - accel_offsets[2];
    calib_gx = gx - gyro_offsets[0];
    calib_gy = gy - gyro_offsets[1];
    calib_gz = gz - gyro_offsets[2];
    // Build IMU message (assumes the MCU already provides SI units: m/s^2 and rad/s)
    auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
    imu_msg->header.stamp = node_->now();
    imu_msg->header.frame_id = "mag_chassis";

    // RCLCPP_INFO(rclcpp::get_logger("imu data: "), "%f %f %f %f %f %f",calib_ax, calib_ay, calib_az, calib_gx, calib_gy, calib_gz);

    imu_msg->linear_acceleration.x = static_cast<double>(calib_ax);
    imu_msg->linear_acceleration.y = static_cast<double>(calib_ay);
    imu_msg->linear_acceleration.z = static_cast<double>(-calib_az);

    imu_msg->angular_velocity.x = static_cast<double>(calib_gx);
    imu_msg->angular_velocity.y = static_cast<double>(calib_gy);
    imu_msg->angular_velocity.z = static_cast<double>(-calib_gz);

    imu_msg->orientation_covariance[0] = -1.0;

    if (chassis_imu_pub_) {
      chassis_imu_pub_->publish(*imu_msg);
    } else {
      RCLCPP_WARN(rclcpp::get_logger("DiffDriveArduinoHardware"), "Chassis IMU publisher not initialized");
    }
  }

  void DiffDriveArduinoHardware::publish_chassis_imu_mag() {
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    float mx = 0.0f, my = 0.0f, mz = 0.0f;
    float mag_offsets[3] = {3.098e-05f, 1.504e-05f, 1.026e-6f}; 
    float phi = 1.512f;
    float rat = 1.008f;
    comms_.read_chassis_imu(ax, ay, az, gx, gy, gz, mx, my, mz);

    float temp = mx;
    mx = my;
    my = -temp;
    mz = -mz;

    // Apply magnetometer calibration 
    float centered_x = mx - mag_offsets[0];
    float centered_y = my - mag_offsets[1];
    float cos_phi = cos(phi);
    float sin_phi = sin(phi);
    float aligned_x = cos_phi * centered_x + sin_phi * centered_y;
    float aligned_y = -sin_phi * centered_x + cos_phi * centered_y;
    float scaled_aligned_y = aligned_y * rat;
    float calib_x = cos_phi * aligned_x - sin_phi * scaled_aligned_y;
    float calib_y = sin_phi * aligned_x + cos_phi * scaled_aligned_y;
    float calib_z = mz - mag_offsets[2];

    // RCLCPP_INFO(rclcpp::get_logger("mag data: "), "%f %f %f",calib_x, calib_y, calib_z);

    auto mag_msg = std::make_shared<sensor_msgs::msg::MagneticField>();
    mag_msg->header.stamp = node_->now();
    mag_msg->header.frame_id = "imu_chassis"; // Match the frame from publish_chassis_imu()
    mag_msg->magnetic_field.x = static_cast<double>(calib_x) / 1e6;
    mag_msg->magnetic_field.y = static_cast<double>(calib_y) / 1e6;
    mag_msg->magnetic_field.z = static_cast<double>(0) / 1e6;

    if (chassis_imu_mag_pub_) {
      chassis_imu_mag_pub_->publish(*mag_msg);
    } else {
      RCLCPP_WARN(rclcpp::get_logger("DiffDriveArduinoHardware"), "Chassis IMU mag publisher not initialized");
    }
  }

hardware_interface::return_type DiffDriveArduinoHardware::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
  {
    rclcpp::spin_some(node_);

    if (!comms_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }
    
    // Sleep briefly to allow serial comms to settle
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    
    // Read Wheel Encoders
    comms_.read_encoder_values(wheel_l_.enc, wheel_r_.enc);

    double delta_seconds = period.seconds();

    // Calculate Left Wheel State
    double pos_prev = wheel_l_.pos;
    wheel_l_.pos = wheel_l_.calc_enc_angle();
    wheel_l_.vel = (wheel_l_.pos - pos_prev) / delta_seconds;

    // Calculate Right Wheel State
    pos_prev = wheel_r_.pos;
    wheel_r_.pos = wheel_r_.calc_enc_angle();
    wheel_r_.vel = (wheel_r_.pos - pos_prev) / delta_seconds;

    // Get Fake Servo Feedback
    std::string response = comms_.send_msg("s\r");
    if (!response.empty()) {
      std::stringstream ss(response.substr(2)); // Skip "s "
      float degs[4];
      ss >> degs[0] >> degs[1] >> degs[2] >> degs[3];
      for (size_t i = 0; i < servos_.size(); ++i) {
        servos_[i].pos = (degs[i] - servo_offsets_deg_[i]) * (M_PI / 180.0); // Subtract offset and convert to radians
        servos_[i].vel = 0.0;
      }
    } else {
      RCLCPP_WARN(rclcpp::get_logger("DiffDriveArduinoHardware"), "No servo feedback received.");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    publish_chassis_imu();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    publish_chassis_imu_mag();

    //Camera IMU for head joint orientation feedback
    if (last_imu_msg_ && last_chassis_imu_msg_ && head_yaw_idx_ != static_cast<size_t>(-1) && head_pitch_idx_ != static_cast<size_t>(-1)) {
      std::lock_guard<std::mutex> head_lock(imu_mutex_);
      std::lock_guard<std::mutex> chassis_lock(chassis_imu_mutex_);

      // Clamp imu based head joint angles around fake servo feedback 
      double yaw_tolerance_rad = 0.02;   
      double pitch_tolerance_rad = 0.35; 

      // Save the Fake Servo Positions as Reference
      double ref_yaw_pos = servos_[head_yaw_idx_].pos;
      double ref_pitch_pos = servos_[head_pitch_idx_].pos;

      //Calculate pure IMU Orientation
      tf2::Quaternion q_head, q_chassis;
      tf2::fromMsg(last_imu_msg_->orientation, q_head);
      tf2::fromMsg(last_chassis_imu_msg_->orientation, q_chassis);
      
      tf2::Quaternion q_head_corrected = q_head * q_offset_.inverse();
      tf2::Quaternion q_relative = q_chassis.inverse() * q_head_corrected;
      
      tf2::Matrix3x3 m(q_relative);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      //Calculate Odom Offset Correction
      double offset = 0.0;
      try {
        if (tf_buffer_->canTransform("odom", "base_link", tf2::TimePointZero)) {
          geometry_msgs::msg::TransformStamped base_transform = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
          tf2::Quaternion q_base;
          tf2::fromMsg(base_transform.transform.rotation, q_base);
          tf2::Matrix3x3 m_base(q_base);
          double roll_base, pitch_base, yaw_base;
          m_base.getRPY(roll_base, pitch_base, yaw_base);

          tf2::Matrix3x3 m_chassis_mat(q_chassis);
          double roll_chassis, pitch_chassis, yaw_chassis;
          m_chassis_mat.getRPY(roll_chassis, pitch_chassis, yaw_chassis);

          double delta = yaw_base - yaw_chassis;
          offset = std::atan2(std::sin(delta), std::cos(delta));
        }
      } catch (const tf2::TransformException & ex) {
      }

      // Determine the Raw IMU Targets
      double imu_target_yaw = -(yaw + offset);
      double imu_target_pitch = -pitch;

      // Helper Lambda to clamp IMU target to within tolerance of Reference
      auto clamp_angle = [](double imu_val, double ref_val, double tolerance) -> double {
        double diff = imu_val - ref_val;
        // Normalize difference to -PI to +PI to handle wrapping
        diff = std::atan2(std::sin(diff), std::cos(diff));

        if (diff > tolerance) {
          return ref_val + tolerance;
        } else if (diff < -tolerance) {
          return ref_val - tolerance;
        }
        return imu_val;
      };

      // Apply Clamping and Set Final Positions
      servos_[head_yaw_idx_].pos = clamp_angle(imu_target_yaw, ref_yaw_pos, yaw_tolerance_rad);
      servos_[head_pitch_idx_].pos = clamp_angle(imu_target_pitch, ref_pitch_pos, pitch_tolerance_rad);

    } else {
      // If no IMU data use the fake servo feedback 
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), 
        *node_->get_clock(), 
        2000, 
        "IMU fusion inactive (missing data or indices): Using raw servo feedback."
      );
    }
    
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type DiffDriveArduinoHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    if (!comms_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }

    int motor_l_counts_per_loop = wheel_l_.cmd / wheel_l_.rads_per_count / cfg_.loop_rate;
    int motor_r_counts_per_loop = wheel_r_.cmd / wheel_r_.rads_per_count / cfg_.loop_rate;

    comms_.set_motor_values(motor_l_counts_per_loop, motor_r_counts_per_loop);

    // Send servo commands
    if (!servos_.empty()) {
      std::vector<float> degs(servos_.size());
      for (size_t i = 0; i < servos_.size(); ++i) {
        float rad = servos_[i].cmd;
        degs[i] = (rad * (180.0 / M_PI)) + servo_offsets_deg_[i]; // Add offset to command
        // Clamp the angles
        degs[i] = std::max(0.0f, std::min(180.0f, degs[i]));
      }
      comms_.set_servo_angles(degs[0], degs[1], degs[2], degs[3]);
    }

    return hardware_interface::return_type::OK;
  }

} 

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  diffdrive_arduino::DiffDriveArduinoHardware, hardware_interface::SystemInterface)