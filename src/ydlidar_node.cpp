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

#define ROS2Verision "2.0.9"


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
  std::string frame_id = "laser_frame";
  bool reversion = false;
  bool resolution_fixed = true;
  bool auto_reconnect = true;
  double angle_max = 180;
  double angle_min = -180;
  int samp_rate = 9;
  std::string list = "";
  double max_range = 16.0;
  double min_range = 0.1;
  double frequency = 10.0;


  node->get_parameter("port", port);

  node->get_parameter("frame_id", frame_id);

  node->get_parameter("ignore_array", list);

  node->get_parameter("samp_rate", samp_rate);

  node->get_parameter("resolution_fixed", resolution_fixed);

  node->get_parameter("auto_reconnect", auto_reconnect);

  node->get_parameter("reversion", reversion);

  node->get_parameter("angle_max", angle_max);

  node->get_parameter("angle_min", angle_min);

  node->get_parameter("max_range", max_range);

  node->get_parameter("min_range", min_range);

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
  laser.setMaxRange(max_range);
  laser.setMinRange(min_range);
  laser.setAutoReconnect(auto_reconnect);
  laser.setScanFrequency(frequency);
  laser.setSampleRate(samp_rate);
  laser.setIgnoreArray(ignore_array);


  printf("[YDLIDAR INFO] Current ROS Driver Version: %s\n", ((std::string)ROS2Verision).c_str());
  bool ret = laser.initialize();
  if (ret) {
    ret = laser.turnOn();
  }

  // Set the QoS. ROS 2 will provide QoS profiles based on the following use cases:
  // Default QoS settings for publishers and subscriptions (rmw_qos_profile_default).
  // Sensor data (rmw_qos_profile_sensor_data).
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  // set the depth to the QoS profile
  custom_qos_profile.depth = 7;




  auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", custom_qos_profile);



  rclcpp::WallRate loop_rate(20);

  while (ret && rclcpp::ok()) {

    bool hardError;
    LaserScan scan;//

    if (laser.doProcessSimple(scan, hardError)) {

      auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

      scan_msg->header.stamp.sec = RCL_NS_TO_S(scan.system_time_stamp);
      scan_msg->header.stamp.nanosec =  scan.system_time_stamp - RCL_S_TO_NS(scan_msg->header.stamp.sec);
      scan_msg->header.frame_id = frame_id;
      scan_msg->angle_min = angles::from_degrees(angle_min);
      scan_msg->angle_max = angles::from_degrees(angle_max);
//      scan_msg->angle_increment = scan.config.ang_increment;
      scan_msg->scan_time = scan.config.scan_time;
      scan_msg->time_increment = scan.config.time_increment;
      scan_msg->range_min = scan.config.min_range;
      scan_msg->range_max = scan.config.max_range;
      int fixed_size = scan.data.size();
      if(resolution_fixed) {
          fixed_size = laser.getFixedSize();
      }
      if(scan_msg->angle_max - scan_msg->angle_min == 2*M_PI) {
          scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / (fixed_size);
      } else {
          scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / (fixed_size - 1);
      }
      int index = 0;
      scan_msg->ranges.resize(fixed_size, std::numeric_limits<float>::infinity());
      scan_msg->intensities.resize(fixed_size, 0);

      for(size_t i = 0; i < scan.data.size(); i++) {
          LaserPoint point = scan.data[i];
          float angle = angles::from_degrees(point.angle);
  	  if(reversion) {
		angle += M_PI;
          }
          angle = 2*M_PI - angle;
          angle = angles::normalize_angle(angle);
          index = (angle -scan_msg->angle_min ) / scan_msg->angle_increment + 0.5;
          if(index >=0 && index < fixed_size) {
             if(point.distance < scan_msg->range_min) {
                 scan_msg->ranges[index] = std::numeric_limits<float>::infinity();
                 scan_msg->intensities[index] = 0;
              } else {
                 scan_msg->ranges[index] = point.distance;
                 scan_msg->intensities[index] = point.intensity;
             }
          }

      } 

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
