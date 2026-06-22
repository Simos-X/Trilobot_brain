#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP

#include <sstream>
#include <libserial/SerialPort.h>
#include <iostream>
#include <algorithm> 
#include <chrono>

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms
{
public:
  ArduinoComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }

  std::string send_msg(const std::string &msg_to_send, bool print_output = false)
  {
    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(msg_to_send);
    std::string response = "";
    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout&)
    {
      std::cerr << "The ReadByte() call has timed out." << std::endl ;
    }
    if (print_output)
    {
      std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
    }
    return response;
  }

  void send_empty_msg()
  {
    std::string response = send_msg("\r");
  }

  // Setter for encoder read frequency
  void set_encoder_read_frequency(int hz) {
    if (hz > 0) {
      encoder_interval_ = std::chrono::milliseconds(1000 / hz);
    } else {
      std::cerr << "Invalid frequency." << std::endl;
    }
  }

  void read_encoder_values(int &val_1, int &val_2)
  {
    auto now = std::chrono::steady_clock::now();
    if (now - last_encoder_time_ >= encoder_interval_) {
      std::string response = send_msg("e\r");
      std::string delimiter = " ";
      size_t del_pos = response.find(delimiter);
      std::string token_1 = response.substr(0, del_pos);
      std::string token_2 = response.substr(del_pos + delimiter.length());
      last_encoder_val1_ = std::atoi(token_1.c_str());
      last_encoder_val2_ = std::atoi(token_2.c_str());
      last_encoder_time_ = now;
    }
    val_1 = last_encoder_val1_;
    val_2 = last_encoder_val2_;
  }

  // Setter for motor set frequency
  void set_motor_set_frequency(int hz) {
    if (hz > 0) {
      motor_interval_ = std::chrono::milliseconds(1000 / hz);
    } else {
      std::cerr << "Invalid frequency." << std::endl;
    }
  }

  void set_motor_values(int val_1, int val_2)
  {
    auto now = std::chrono::steady_clock::now();
    if (now - last_motor_time_ >= motor_interval_) {
      std::stringstream ss;
      ss << "m " << val_1 << " " << val_2 << "\r";
      send_msg(ss.str());
      last_motor_time_ = now;
    }
  }

  // Setter for PID set frequency
  void set_pid_set_frequency(int hz) {
    if (hz > 0) {
      pid_interval_ = std::chrono::milliseconds(1000 / hz);
    } else {
      std::cerr << "Invalid frequency." << std::endl;
    }
  }

  void set_pid_values(float k_p, float k_d, float k_i, float k_o)
  {
    auto now = std::chrono::steady_clock::now();
    if (now - last_pid_time_ >= pid_interval_) {
      std::stringstream ss;
      ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
      send_msg(ss.str());
      last_pid_time_ = now;
    }
  }

  // Setter for servo set frequency
  void set_servo_set_frequency(int hz) {
    if (hz > 0) {
      servo_set_interval_ = std::chrono::milliseconds(1000 / hz);
    } else {
      std::cerr << "Invalid frequency." << std::endl;
    }
  }

  void set_servo_angles(float deg1, float deg2, float deg3, float deg4)
  {
    auto now = std::chrono::steady_clock::now();
    if (now - last_servo_set_time_ >= servo_set_interval_) {
      std::stringstream ss;
      ss << "j " << deg1 << " " << deg2 << " " << deg3 << " " << deg4 << "\r";
      send_msg(ss.str());
      last_servo_set_time_ = now;
    }
  }

  // Setter for servo read frequency
  void set_servo_read_frequency(int hz) {
    if (hz > 0) {
      servo_read_interval_ = std::chrono::milliseconds(1000 / hz);
    } else {
      std::cerr << "Invalid frequency." << std::endl;
    }
  }

  void read_servo_angles(float &deg1, float &deg2, float &deg3, float &deg4)
  {
    auto now = std::chrono::steady_clock::now();
    if (now - last_servo_read_time_ >= servo_read_interval_) {
      std::string response = send_msg("s\r");
      if (!response.empty() && response[0] == 's') {
        std::stringstream ss(response.substr(2)); // Skip "s "
        ss >> last_servo_deg1_ >> last_servo_deg2_ >> last_servo_deg3_ >> last_servo_deg4_;
      } else {
        last_servo_deg1_ = 80.0f;
        last_servo_deg2_ = 35.0f;
        last_servo_deg3_ = 125.0f;
        last_servo_deg4_ = 130.0f;
      }
      last_servo_read_time_ = now;
    }
    deg1 = last_servo_deg1_;
    deg2 = last_servo_deg2_;
    deg3 = last_servo_deg3_;
    deg4 = last_servo_deg4_;
  }

  // Setter for IMU read frequency
  void set_imu_read_frequency(int hz) {
    if (hz > 0) {
      imu_read_interval_ = std::chrono::milliseconds(1000 / hz);
    } else {
      std::cerr << "Invalid frequency: " << hz << ". Using default." << std::endl;
    }
  }

  void read_chassis_imu(float &ax, float &ay, float &az,
  float &gx, float &gy, float &gz,
  float &mx, float &my, float &mz)
  {
    auto now = std::chrono::steady_clock::now();
    if (now - last_imu_read_time_ >= imu_read_interval_) {
      std::string response = send_msg("i\r");
      std::string delimiter = " ";
      size_t p1 = response.find(delimiter);
      size_t p2 = response.find(delimiter, p1 + 1);
      size_t p3 = response.find(delimiter, p2 + 1);
      size_t p4 = response.find(delimiter, p3 + 1);
      size_t p5 = response.find(delimiter, p4 + 1);
      size_t p6 = response.find(delimiter, p5 + 1);
      size_t p7 = response.find(delimiter, p6 + 1);
      size_t p8 = response.find(delimiter, p7 + 1);
      std::string token1 = response.substr(0, p1);
      std::string token2 = response.substr(p1 + 1, p2 - (p1 + 1));
      std::string token3 = response.substr(p2 + 1, p3 - (p2 + 1));
      std::string token4 = response.substr(p3 + 1, p4 - (p3 + 1));
      std::string token5 = response.substr(p4 + 1, p5 - (p4 + 1));
      std::string token6 = response.substr(p5 + 1, p6 - (p5 + 1));
      std::string token7 = response.substr(p6 + 1, p7 - (p6 + 1));
      std::string token8 = response.substr(p7 + 1, p8 - (p7 + 1));
      std::string token9 = response.substr(p8 + 1);
      last_ax_ = std::atof(token1.c_str());
      last_ay_ = std::atof(token2.c_str());
      last_az_ = std::atof(token3.c_str());
      last_gx_ = std::atof(token4.c_str());
      last_gy_ = std::atof(token5.c_str());
      last_gz_ = std::atof(token6.c_str());
      last_mx_ = std::atof(token7.c_str());
      last_my_ = std::atof(token8.c_str());
      last_mz_ = std::atof(token9.c_str());
      last_imu_read_time_ = now;
    }
    // Always return the last read values
    ax = last_ax_;
    ay = last_ay_;
    az = last_az_;
    gx = last_gx_;
    gy = last_gy_;
    gz = last_gz_;
    mx = last_mx_;
    my = last_my_;
    mz = last_mz_;
  }

private:
  LibSerial::SerialPort serial_conn_;
  int timeout_ms_;

  std::chrono::milliseconds encoder_interval_{100}; 
  std::chrono::steady_clock::time_point last_encoder_time_ = std::chrono::steady_clock::now() - std::chrono::hours(1);
  int last_encoder_val1_ = 0, last_encoder_val2_ = 0;

  std::chrono::milliseconds motor_interval_{100};
  std::chrono::steady_clock::time_point last_motor_time_ = std::chrono::steady_clock::now() - std::chrono::hours(1);

  std::chrono::milliseconds pid_interval_{1000}; 
  std::chrono::steady_clock::time_point last_pid_time_ = std::chrono::steady_clock::now() - std::chrono::hours(1);

  std::chrono::milliseconds servo_set_interval_{100}; 
  std::chrono::steady_clock::time_point last_servo_set_time_ = std::chrono::steady_clock::now() - std::chrono::hours(1);

  std::chrono::milliseconds servo_read_interval_{100}; 
  std::chrono::steady_clock::time_point last_servo_read_time_ = std::chrono::steady_clock::now() - std::chrono::hours(1);
  float last_servo_deg1_ = 80.0f, last_servo_deg2_ = 35.0f, last_servo_deg3_ = 125.0f, last_servo_deg4_ = 130.0f;

  std::chrono::milliseconds imu_read_interval_{100}; 
  std::chrono::steady_clock::time_point last_imu_read_time_ = std::chrono::steady_clock::now() - std::chrono::hours(1); // Initial old time
  float last_ax_ = 0.0f, last_ay_ = 0.0f, last_az_ = 0.0f;
  float last_gx_ = 0.0f, last_gy_ = 0.0f, last_gz_ = 0.0f;
  float last_mx_ = 0.0f, last_my_ = 0.0f, last_mz_ = 0.0f;
};
#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP