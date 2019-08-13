/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS 2 Node
 *
 *  Copyright 2017 - 2020 EAI TEAM
 *  http://www.eaibot.com
 *
 */

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include <math.h>

#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "CYdLidar.h"
#include "timer.h"
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>

#define ROS2Verision "1.3.6"


using namespace ydlidar;

std::vector<float> split(const std::string &s, char delim) {
  std::vector<float> elems;
  std::stringstream ss(s);
  std::string number;

  while (std::getline(ss, number, delim)) {
    elems.push_back(atoi(number.c_str()));
  }

  return elems;
}



bool fileExists(const std::string filename) {
  return 0 == _access(filename.c_str(), 0x00);  // 0x00 = Check for existence only!
}


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("ydlidar_node");
  std::string port = "/dev/ttyUSB0";
  int baudrate = 115200;
  std::string frame_id = "laser_frame";
  bool angle_fixed = true;
  bool intensities = false;
  bool low_exposure = false;
  bool reversion = false;
  bool resolution_fixed = true;
  bool auto_reconnect = true;
  double angle_max = 180;
  double angle_min = -180;
  int samp_rate = 9;
  std::string list = "";
  double max_range = 16.0;
  double min_range = 0.06;
  double frequency = 7.0;

  node->declare_parameter("port");
  node->get_parameter("port", port);

  node->declare_parameter("frame_id");
  node->get_parameter("frame_id", frame_id);

  node->declare_parameter("ignore_array");
  node->get_parameter("ignore_array", list);

  node->declare_parameter("baudrate");
  node->get_parameter("baudrate", baudrate);

  node->declare_parameter("samp_rate");
  node->get_parameter("samp_rate", samp_rate);

  node->declare_parameter("resolution_fixed");
  node->get_parameter("resolution_fixed", resolution_fixed);

  node->declare_parameter("low_exposure");
  node->get_parameter("low_exposure", low_exposure);

  node->declare_parameter("auto_reconnect");
  node->get_parameter("auto_reconnect", auto_reconnect);

  node->declare_parameter("reversion");
  node->get_parameter("reversion", reversion);

  node->declare_parameter("intensities");
  node->get_parameter("intensities", intensities);

  node->declare_parameter("angle_max");
  node->get_parameter("angle_max", angle_max);

  node->declare_parameter("angle_min");
  node->get_parameter("angle_min", angle_min);

  node->declare_parameter("max_range");
  node->get_parameter("max_range", max_range);

  node->declare_parameter("min_range");
  node->get_parameter("min_range", min_range);

  node->declare_parameter("frequency");
  node->get_parameter("frequency", frequency);




  std::vector<float> ignore_array = split(list, ',');

  if (ignore_array.size() % 2) {
    RCLCPP_ERROR(node->get_logger(), "ignore array is odd need be even");
  }

  for (uint16_t i = 0 ; i < ignore_array.size(); i++) {
    if (ignore_array[i] < -180 && ignore_array[i] > 180) {
      RCLCPP_ERROR(node->get_logger(), "ignore array should be between -180 and 180");
    }
  }



  CYdLidar laser;

  if (frequency < 5) {
    frequency = 7.0;
  }

  if (frequency > 12) {
    frequency = 12;
  }

  if (angle_max < angle_min) {
    double temp = angle_max;
    angle_max = angle_min;
    angle_min = temp;
  }

  laser.setSerialPort(port);
  laser.setSerialBaudrate(baudrate);
  laser.setIntensities(intensities);
  laser.setMaxRange(max_range);
  laser.setMinRange(min_range);
  laser.setMaxAngle(angle_max);
  laser.setMinAngle(angle_min);
  laser.setReversion(reversion);
  laser.setFixedResolution(resolution_fixed);
  laser.setAutoReconnect(auto_reconnect);
  laser.setExposure(low_exposure);
  laser.setScanFrequency(frequency);
  laser.setSampleRate(samp_rate);
  laser.setReversion(reversion);
  laser.setIgnoreArray(ignore_array);


  printf("[YDLIDAR INFO] Current ROS Driver Version: %s\n", ((std::string)ROS2Verision).c_str());
  laser.initialize();

  // Set the QoS. ROS 2 will provide QoS profiles based on the following use cases:
  // Default QoS settings for publishers and subscriptions (rmw_qos_profile_default).
  // Sensor data (rmw_qos_profile_sensor_data).
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  // set the depth to the QoS profile
  custom_qos_profile.depth = 7;




  auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", custom_qos_profile);



  rclcpp::WallRate loop_rate(20);

  while (rclcpp::ok()) {

    bool hardError;
    LaserScan scan;//

    if (laser.doProcessSimple(scan, hardError)) {

      auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

      scan_msg->header.stamp.sec = RCL_NS_TO_S(scan.system_time_stamp);
      scan_msg->header.stamp.nanosec =  scan.system_time_stamp - RCL_S_TO_NS(scan_msg->header.stamp.sec);
      scan_msg->header.frame_id = frame_id;
      scan_msg->angle_min = scan.config.min_angle;
      scan_msg->angle_max = scan.config.max_angle;
      scan_msg->angle_increment = scan.config.ang_increment;
      scan_msg->scan_time = scan.config.scan_time;
      scan_msg->time_increment = scan.config.time_increment;
      scan_msg->range_min = scan.config.min_range;
      scan_msg->range_max = scan.config.max_range;

      scan_msg->ranges = scan.ranges;
      scan_msg->intensities =  scan.intensities;
      laser_pub->publish(scan_msg);


    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to get scan");
    }



    rclcpp::spin_some(node);
    loop_rate.sleep();
  }


  printf("[YDLIDAR INFO] Now YDLIDAR is stopping .......\n");
  laser.turnOff();
  laser.disconnecting();
  rclcpp::shutdown();

  return 0;
}
