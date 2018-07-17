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
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "CYdLidar.h"
#include "timer.h"
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>

#include <ini_parser.hpp>

#if !defined(_MSC_VER)
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#endif

#if !defined(_MSC_VER)
#	define _access access
#endif

#define ROS2Verision "1.3.5"


using namespace ydlidar;

static rclcpp::Logger g_logger = rclcpp::get_logger("ydlidar_node");

static std::string INI_NAME="/ros2/config.ini";

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
    return 0 == _access(filename.c_str(), 0x00 ); // 0x00 = Check for existence only!
}


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("ydlidar_node");
  g_logger = node->get_logger();

  std::string port = "/dev/ttyUSB0";
  int baudrate = 115200;
  std::string frame_id = "laser_frame";
  bool angle_fixed = true;
  bool intensities = false;
  bool low_exposure = false;
  bool reversion = false;
  bool resolution_fixed = true;
  bool heartbeat = false;
  bool auto_reconnect = true;
  bool debug = false;
  double angle_max = 180;
  double angle_min = -180;
  int samp_rate = 9;
  std::string list = "";
  double max_range = 16.0;
  double min_range = 0.06;
  double frequency = 7.0;
  double sensor_x = 0.0;
  double sensor_y = 0.0;
  double sensor_yaw = 0.0;
    

#if !defined(_MSC_VER)
 char *homedir=NULL;
  homedir = getenv("HOME");
  if (homedir == NULL) {
    homedir = getpwuid(getuid())->pw_dir;
  }
  INI_NAME = (std::string)homedir + "/.ros2/settings.ini";
#endif


  for (int i=0; i< argc; i++) {
	std::string str = argv[i];
        if (str.find(".ini") != string::npos) {
		INI_NAME = str;
		break;
	}
  }

  if (fileExists(INI_NAME)) {
  	ini_parser parser_instance(INI_NAME);
    const std::string section = "LASER";
	port = parser_instance.get_string("port","/dev/ttyUSB0", section);
	frame_id = parser_instance.get_string("frame_id","laser_frame", section);
	list = parser_instance.get_string("ignore_array","", section);
	baudrate = parser_instance.get_int("baudrate",115200, section);
    samp_rate = parser_instance.get_int("samp_rate",4, section);
	angle_fixed = parser_instance.get_bool("angle_fixed",true, section);
    resolution_fixed = parser_instance.get_bool("resolution_fixed", true,section);
    heartbeat = parser_instance.get_bool("heartbeat", false,section);
    low_exposure = parser_instance.get_bool("low_exposure", false,section);

    auto_reconnect = parser_instance.get_bool("auto_reconnect", true,section);
    reversion = parser_instance.get_bool("reversion", false,section);
    debug = parser_instance.get_bool("debug", false,section);
    intensities = parser_instance.get_bool("intensities", false,section);


	angle_max = parser_instance.get_double("angle_max",180, section);
	angle_min = parser_instance.get_double("angle_min",-180, section);
	max_range = parser_instance.get_double("max_range", 16.0, section);
	min_range = parser_instance.get_double("min_range",0.08, section);
    frequency = parser_instance.get_double("frequency",7.0, section);

    sensor_x = parser_instance.get_double("sensor_x",7.0, section);
    sensor_y = parser_instance.get_double("sensor_y",7.0, section);
    sensor_yaw = parser_instance.get_double("sensor_yaw",7.0, section);



	RCLCPP_INFO(g_logger, "port:%s", port.c_str());
	RCLCPP_INFO(g_logger, "baudrate:%i", baudrate);

  }


  node->get_parameter("port", port);

  node->get_parameter("frame_id", frame_id);

  node->get_parameter("ignore_array", list);

  node->get_parameter("baudrate", baudrate);

  node->get_parameter("samp_rate", samp_rate);

  node->get_parameter("angle_fixed", angle_fixed);

  node->get_parameter("resolution_fixed", resolution_fixed);

  node->get_parameter("heartbeat", heartbeat);

  node->get_parameter("low_exposure", low_exposure);

  node->get_parameter("auto_reconnect", auto_reconnect);

  node->get_parameter("reversion", reversion);

  node->get_parameter("debug", debug);

  node->get_parameter("intensities", intensities);


  node->get_parameter("angle_max", angle_max);

  node->get_parameter("angle_min", angle_min);

  node->get_parameter("max_range", max_range);

  node->get_parameter("min_range", min_range);

  node->get_parameter("frequency", frequency);

  node->get_parameter("sensor_x", sensor_x);

  node->get_parameter("sensor_y", sensor_y);

  node->get_parameter("sensor_yaw", sensor_yaw);




  std::vector<float> ignore_array = split(list ,',');
  if (ignore_array.size()%2) {
        RCLCPP_ERROR(g_logger,"ignore array is odd need be even");
    }

    for (uint16_t i =0 ; i < ignore_array.size();i++) {
        if (ignore_array[i] < -180 && ignore_array[i] > 180) {
            RCLCPP_ERROR(g_logger,"ignore array should be between -180 and 180");
        }
    }

  CYdLidar laser;

  if (frequency<5) {
      frequency = 7.0;
  }
  if (frequency>12) {
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
    laser.setHeartBeat(heartbeat);
    laser.setReversion(reversion);
    laser.setFixedResolution(resolution_fixed);
    laser.setAutoReconnect(auto_reconnect);
    laser.setEnableDebug(debug);
    laser.setExposure(low_exposure);
    laser.setScanFrequency(frequency);
    laser.setSampleRate(samp_rate);
    laser.setReversion(reversion);
    laser.setIgnoreArray(ignore_array);

    //雷达相对机器人安装位置
    pose_info laser_pose;
    laser_pose.x = sensor_x;
    laser_pose.y = sensor_y;
    laser_pose.phi = sensor_yaw;
    laser.setSensorPose(laser_pose);

    printf("[YDLIDAR INFO] Current ROS Driver Version: %s\n",((std::string)ROS2Verision).c_str());
    laser.initialize();

    // Set the QoS. ROS 2 will provide QoS profiles based on the following use cases:
    // Default QoS settings for publishers and subscriptions (rmw_qos_profile_default).
    // Services (rmw_qos_profile_services_default).
    // Sensor data (rmw_qos_profile_sensor_data).
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
    // set the depth to the QoS profile
    custom_qos_profile.depth = 7;

  auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", custom_qos_profile);
  auto sync_laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("sync_scan", custom_qos_profile);
  auto marker_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>("publish_line_markers", custom_qos_profile);




  rclcpp::WallRate loop_rate(30);

  while (rclcpp::ok()) {

      bool hardError;
        LaserScan scan;//原始激光数据
        LaserScan syncscan;//同步后激光数据
        PointCloud pc;//同步后激光点云数据
        std::vector<gline> lines;
        if(laser.doProcessSimple(scan, syncscan, pc, lines, hardError )){

            auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
            auto sync_scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

            scan_msg->header.stamp.sec = RCL_NS_TO_S( scan.system_time_stamp);
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

	        sync_scan_msg->header = scan_msg->header;
            sync_scan_msg->angle_min = syncscan.config.min_angle;
            sync_scan_msg->angle_max = syncscan.config.max_angle;
            sync_scan_msg->angle_increment = syncscan.config.ang_increment;
            sync_scan_msg->scan_time = syncscan.config.scan_time;
            sync_scan_msg->time_increment = syncscan.config.time_increment;
            sync_scan_msg->range_min = syncscan.config.min_range;
            sync_scan_msg->range_max = syncscan.config.max_range;
            
            sync_scan_msg->ranges = syncscan.ranges;
            sync_scan_msg->intensities =  syncscan.intensities;
	        sync_laser_pub->publish(sync_scan_msg);
            

            auto marker_msg = std::make_shared<visualization_msgs::msg::Marker>();
            marker_msg->ns = "laser_fit_lines";
	        marker_msg->id = 0;
	        marker_msg->type = visualization_msgs::msg::Marker::LINE_LIST;
	        marker_msg->scale.x = 0.03;
	        marker_msg->color.r = 0.0;
	        marker_msg->color.g = 1.0;
	        marker_msg->color.b = 0.0;
	        marker_msg->color.a = 1.0;
            for(std::vector<gline>::const_iterator it = lines.begin(); it != lines.end(); it++) {
                geometry_msgs::msg::Point p_start;
		        p_start.x = it->x1;
	            p_start.y = it->y1;
	            p_start.z = 0;
	            marker_msg->points.push_back(p_start);
	            geometry_msgs::msg::Point p_end;
	            p_end.x = it->x2;
	            p_end.y = it->y2;
	            p_end.z = 0;
	            marker_msg->points.push_back(p_end);
            }
            marker_msg->header = scan_msg->header;
            marker_publisher_->publish(marker_msg);
            

		} else {
             RCLCPP_ERROR(node->get_logger(), "Failed to get scan");
        }

        {//做imu和odometry数据输入
            odom_info odom;
            odom.x = 0;
            odom.y = 0;
            odom.phi = 0;
            odom.stamp = getTime();
            laser.setSyncOdometry(odom);


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
