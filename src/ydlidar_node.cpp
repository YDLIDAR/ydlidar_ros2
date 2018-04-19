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

#include "ydlidar_driver.h"
#include "common.h"
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

#define DELAY_SECONDS 5
#define DEG2RAD(x) ((x)*M_PI/180.)

#if !defined(_MSC_VER)
#	define _access access
#endif

using namespace ydlidar;

static rclcpp::Logger g_logger = rclcpp::get_logger("ydlidar_node");

static std::string INI_NAME="/ros2/config.ini";

static bool flag = true;
static int nodes_count = 720;
static float each_angle = 0.5;
void publish_scan(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub,  node_info *nodes,  size_t node_count,  rcutils_time_point_value_t start, double scan_time, float angle_min, float angle_max, std::string frame_id, std::vector<int> ignore_array, double min_range , double max_range)
{
    auto msg = std::make_shared<sensor_msgs::msg::LaserScan>();

    int counts = node_count*((angle_max-angle_min)/360.0f);
    int angle_start = 180+angle_min;
    int node_start = node_count*(angle_start/360.0f);

    msg->ranges.resize(counts);
    msg->intensities.resize(counts);

    float nodes_array[node_count];
    float quality_array[node_count];
    float range = 0.0;
    float intensity = 0.0;
    int index = 0;
    for (size_t i = 0; i < node_count; i++) {
	range = (float)nodes[i].distance_q2/4.0f/1000;
	intensity = (float)(nodes[i].sync_quality >> 2);

        if(i<node_count/2){
	    index = node_count/2-1-i;	    
        }else{
	    index =node_count-1-(i-node_count/2);
        }

        if(ignore_array.size() != 0){
	    float angle = (float)((nodes[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
            if(angle>180){
                angle=360-angle;
            }else{
                angle=-angle;
            }
	    for(uint16_t j = 0; j < ignore_array.size();j = j+2){
                if((ignore_array[j] < angle) && (angle <= ignore_array[j+1])){
		   range = 0.0;
		   break;
		}
	    }
	}

	if(range > max_range||range < min_range){
	    range = 0.0;
        }

	int pos = index - node_start ;
        if(0<= pos && pos < counts){
	    msg->ranges[pos] =  range;
	    msg->intensities[pos] = intensity;
	}

    }


    msg->header.stamp.sec = RCL_NS_TO_S(start);
    msg->header.stamp.nanosec = start - RCL_S_TO_NS(msg->header.stamp.sec);
    msg->header.frame_id = frame_id;
    float radian_min = DEG2RAD(angle_min);
    float radian_max = DEG2RAD(angle_max);
    msg->angle_min = radian_min;
    msg->angle_max = radian_max;
    msg->angle_increment = (msg->angle_max - msg->angle_min) / (double)counts;
    msg->scan_time = scan_time;
    msg->time_increment = scan_time / (double)counts;
    msg->range_min = min_range;
    msg->range_max = max_range;

    pub->publish(msg);
}

std::vector<int> split(const std::string &s, char delim) {
    std::vector<int> elems;
    std::stringstream ss(s);
    std::string number;
    while(std::getline(ss, number, delim)) {
        elems.push_back(atoi(number.c_str()));
    }
    return elems;
}

bool getDeviceInfo(std::string port)
{
    if (!YDlidarDriver::singleton()){
        return false;
    }

    device_info devinfo;
    if (YDlidarDriver::singleton()->getDeviceInfo(devinfo) !=RESULT_OK){
        RCLCPP_ERROR(g_logger,"YDLIDAR get DeviceInfo Error\n" );
        return false;
    }
    int _samp_rate=4;
    std::string model;
    float freq = 7.0f;
    switch(devinfo.model){
            case 1:
                model="F4";
                _samp_rate=4;
                freq = 7.0;
                break;
            case 4:
                model="S4";
                _samp_rate=4;
                freq = 7.0;
                break;
            case 5:
                model="G4";
                _samp_rate=9;
                nodes_count = 1440;
                each_angle = 0.25;
                freq = 7.0;
                break;
            case 6:
                model="X4";
                _samp_rate=5;
                freq = 7.0;
                break;
            default:
                model = "Unknown";
    }

    uint16_t maxv = (uint16_t)(devinfo.firmware_version>>8);
    uint16_t midv = (uint16_t)(devinfo.firmware_version&0xff)/10;
    uint16_t minv = (uint16_t)(devinfo.firmware_version&0xff)%10;
    if(midv==0){
        midv = minv;
        minv = 0;
    }

    printf("[YDLIDAR INFO] Connection established in %s:\n"
            "Firmware version: %u.%u.%u\n"
            "Hardware version: %u\n"
            "Model: %s\n"
            "Serial: ",
            port.c_str(),
            maxv,
            midv,
            minv,
            (uint16_t)devinfo.hardware_version,
            model.c_str());

    for (int i=0;i<16;i++){
        printf("%01X",devinfo.serialnum[i]&0xff);
    }
    printf("\n");

    printf("[YDLIDAR INFO] Current Sampling Rate : %dK\n" , _samp_rate);
    printf("[YDLIDAR INFO] Current Scan Frequency : %fHz\n" , freq);

    return true;

}


bool getDeviceHealth()
{
    if (!YDlidarDriver::singleton()){
        return false;
    }

    result_t op_result;
    device_health healthinfo;

    op_result = YDlidarDriver::singleton()->getHealth(healthinfo);
    if (op_result == RESULT_OK) { 
        printf("[YDLIDAR INFO] YDLIDAR running correctly! The health status: %s\n", healthinfo.status==0?"well":"bad");
        
        if (healthinfo.status == 2) {
            RCLCPP_ERROR(g_logger,"YDLIDAR internal error detected. Please reboot the device to retry.");
            return false;
        } else {
            return true;
        }

    } else {
        RCLCPP_ERROR(g_logger,"cannot retrieve YDLIDAR health code: %x", op_result);
        return false;
    }

}

bool fileExists(const std::string filename){
    return 0 == _access(filename.c_str(), 0x00 ); // 0x00 = Check for existence only!
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("ydlidar_node");
  g_logger = node->get_logger();

  std::string port = "/dev/ttyUSB0";
  std::string frame_id = "laser_frame";
  std::string list ="";
  int baudrate = 128000;
  bool angle_fixed = true;
  bool intensities_ = false;
  double angle_max = 180;
  double angle_min = -180;
  double max_range = 16.0;
  double min_range = 0.08;

#if !defined(_MSC_VER)
 char *homedir=NULL;
  homedir = getenv("HOME");
  if(homedir == NULL){
    homedir = getpwuid(getuid())->pw_dir;
  }
  INI_NAME = (std::string)homedir + "/.ros2/settings.ini";
#endif


  for(int i=0; i< argc; i++){
	std::string str = argv[i];
	if(str.find(".ini") != string::npos){
		INI_NAME = str;
		break;
	}
  }

  if(fileExists(INI_NAME)){
  	ini_parser parser_instance(INI_NAME);
        const std::string section = "LASER";
	port = parser_instance.get_string("port","/dev/ttyUSB0", section);
	frame_id = parser_instance.get_string("frame_id","laser_frame", section);
	list = parser_instance.get_string("ignore_array","", section);
	baudrate = parser_instance.get_int("baudrate",115200, section);
	angle_fixed = parser_instance.get_bool("angle_fixed",true, section);
	intensities_ = parser_instance.get_bool("intensities", false,section);
	angle_max = parser_instance.get_double("angle_max",180, section);
	angle_min = parser_instance.get_double("angle_min",-180, section);
	max_range = parser_instance.get_double("max_range", 16.0, section);
	min_range = parser_instance.get_double("min_range",0.08, section);

	RCLCPP_INFO(g_logger, "port:%s", port.c_str());
	RCLCPP_INFO(g_logger, "baudrate:%i", baudrate);

  }


  node->get_parameter("port", port);

  node->get_parameter("frame_id", frame_id);

  node->get_parameter("ignore_array", list);

  node->get_parameter("baudrate", baudrate);

  node->get_parameter("angle_fixed", angle_fixed);

  node->get_parameter("intensities", intensities_);

  node->get_parameter("angle_max", angle_max);

  node->get_parameter("angle_min", angle_min);

  node->get_parameter("max_range", max_range);

  node->get_parameter("min_range", min_range);


  std::vector<int> ignore_array = split(list ,',');
  if(ignore_array.size()%2){
        RCLCPP_ERROR(g_logger,"ignore array is odd need be even");
    }

    for(uint16_t i =0 ; i < ignore_array.size();i++){
        if(ignore_array[i] < -180 && ignore_array[i] > 180){
            RCLCPP_ERROR(g_logger,"ignore array should be between -180 and 180");
        }
    }

  YDlidarDriver::initDriver(); 
  if (!YDlidarDriver::singleton()) {
	RCLCPP_ERROR(g_logger, "YDLIDAR Create Driver fail, exit\n");
      return -2;
  }

  printf("[YDLIDAR INFO] Current SDK Version: %s\n",YDlidarDriver::singleton()->getSDKVersion().c_str());

    // Set the QoS. ROS 2 will provide QoS profiles based on the following use cases:
    // Default QoS settings for publishers and subscriptions (rmw_qos_profile_default).
    // Services (rmw_qos_profile_services_default).
    // Sensor data (rmw_qos_profile_sensor_data).
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
    // set the depth to the QoS profile
    custom_qos_profile.depth = 7;

  auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", custom_qos_profile);



  result_t op_result = YDlidarDriver::singleton()->connect(port.c_str(), (uint32_t)baudrate);
    if (op_result != RESULT_OK) {
        int seconds=0;
        while(seconds <= DELAY_SECONDS&&flag){
            delay(2000);
            seconds = seconds + 2;
            YDlidarDriver::singleton()->disconnect();
            op_result = YDlidarDriver::singleton()->connect(port.c_str(), (uint32_t)baudrate);
            printf("[YDLIDAR INFO] Try to connect the port %s again  after %d s .\n", port.c_str() , seconds);
            if(op_result==RESULT_OK){
                break;
            }
        }
        
        if(seconds > DELAY_SECONDS){
            RCLCPP_ERROR(g_logger,"YDLIDAR Cannot bind to the specified serial port %s" , port.c_str());
	    YDlidarDriver::singleton()->disconnect();
	    YDlidarDriver::done();
            return -1;
        }
    }

    printf("[YDLIDAR INFO] Connected to YDLIDAR on port %s at %d \n" , port.c_str(), baudrate);
    if(!getDeviceHealth()||!getDeviceInfo(port)){
        YDlidarDriver::singleton()->disconnect();
        YDlidarDriver::done();
        return -1;
    }
    YDlidarDriver::singleton()->setIntensities(intensities_);

   result_t ans=YDlidarDriver::singleton()->startScan();
    if(ans != RESULT_OK){
        ans = YDlidarDriver::singleton()->startScan();
        if(ans != RESULT_OK){
            RCLCPP_ERROR(g_logger,"start YDLIDAR is failed! Exit!! ......");
            YDlidarDriver::singleton()->disconnect();
            YDlidarDriver::done();
            return 0;
        }
    }

  printf("[YDLIDAR INFO] Now YDLIDAR is scanning ......\n");
  flag = false;

  rclcpp::WallRate loop_rate(30);
 rcutils_time_point_value_t start_scan_time,end_scan_time;

  node_info all_nodes[nodes_count];
  memset(all_nodes, 0, nodes_count*sizeof(node_info));

  while (rclcpp::ok()) {
        try{
            node_info nodes[nodes_count];
            size_t   count = _countof(nodes);

             if (rcutils_system_time_now(&start_scan_time) != RCUTILS_RET_OK) {
        	  RCLCPP_ERROR(node->get_logger(), "Failed to get system time")
	     }
            op_result = YDlidarDriver::singleton()->grabScanData(nodes, count);
            if (rcutils_system_time_now(&end_scan_time) != RCUTILS_RET_OK) {
        	  RCLCPP_ERROR(node->get_logger(), "Failed to get system time")
	     }
            double scan_duration = (end_scan_time - start_scan_time);
        
            if (op_result == RESULT_OK) {
                op_result = YDlidarDriver::singleton()->ascendScanData(nodes, count);
            
                if (op_result == RESULT_OK) {
                    if (angle_fixed) {
                        memset(all_nodes, 0, nodes_count*sizeof(node_info));
                    
                        for(size_t i = 0; i < count; i++) {
                            if (nodes[i].distance_q2 != 0) {
                                float angle = (float)((nodes[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
                                int inter =(int)( angle / each_angle );
                                float angle_pre = angle - inter * each_angle;
                                float angle_next = (inter+1) * each_angle - angle;
                                if(angle_pre < angle_next){
                                    if(inter < nodes_count){
                                        all_nodes[inter]=nodes[i];
                                    }
                                }else{
                                    if(inter < nodes_count-1){
                                        all_nodes[inter+1]=nodes[i];
                                    }
                                }
                            }
                        }
                        publish_scan(laser_pub, all_nodes, nodes_count, start_scan_time, scan_duration, angle_min, angle_max, frame_id, ignore_array, min_range , max_range);
                    } else {
                        int start_node = 0, end_node = 0;
                        int i = 0;
                        while (nodes[i++].distance_q2 == 0&&i<count);
                        start_node = i-1;
                        i = count -1;
                        while (nodes[i--].distance_q2 == 0&&i>=0);
                        end_node = i+1;

                        angle_min = (float)(nodes[start_node].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
                        angle_max = (float)(nodes[end_node].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;

                        publish_scan(laser_pub, &nodes[start_node], end_node-start_node +1,  start_scan_time, scan_duration, angle_min, angle_max, frame_id, ignore_array, min_range , max_range);
                   }
                }
            }
            rclcpp::spin_some(node);
    	    loop_rate.sleep();
	}catch(std::exception &e){//
            
            YDlidarDriver::singleton()->disconnect();
            printf("[YDLIDAR INFO] Now YDLIDAR is stopping .......\n");
            YDlidarDriver::done();
            return 0;
	}catch(...){//anthor exception
            RCLCPP_ERROR(g_logger,"Unhandled Exception:Unknown ");
            YDlidarDriver::singleton()->disconnect();
            printf("[YDLIDAR INFO] Now YDLIDAR is stopping .......\n");
            YDlidarDriver::done();
            return 0;
	}
  }

  YDlidarDriver::singleton()->disconnect();
  printf("[YDLIDAR INFO] Now YDLIDAR is stopping .......\n");
  YDlidarDriver::done();
  rclcpp::shutdown();

  return 0;
}
