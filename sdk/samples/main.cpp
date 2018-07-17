
#include "CYdLidar.h"
#include "timer.h"
#include <iostream>
#include <string>
#include <signal.h>
#include <memory>
//#include <unistd.h>
using namespace std;
using namespace ydlidar;
CYdLidar laser;
static bool running = false;

static void Stop(int signo)
{

    printf("Received exit signal\n");
    running = true;

}

int main(int argc, char * argv[])
{

	printf(" YDLIDAR C++ TEST\n");
    std::string port;
    std::string baudrate;
    std::string intensity;
    printf("Please enter the lidar port:");
    std::cin>>port;
    printf("Please enter the lidar baud rate:");
    std::cin>>baudrate;

    printf("Please enter the lidar intensity:");
    std::cin>>intensity;

    const int baud = atoi(baudrate.c_str());
    bool intensities = atoi(intensity.c_str()) ==0?false:true;

    signal(SIGINT, Stop);
    signal(SIGTERM, Stop);
    laser.setSerialPort(port);
    laser.setSerialBaudrate(baud);
    laser.setIntensities(intensities);
    laser.setMaxRange(16.0);
    laser.setMinRange(0.1);
    laser.setMaxAngle(180);
    laser.setMinAngle(-180);
    laser.setHeartBeat(false);
    laser.setReversion(false);
    laser.setFixedResolution(false);
    laser.setAutoReconnect(true);
    laser.setEnableDebug(false);

    //雷达相对机器人安装位置
    pose_info laser_pose;
    laser_pose.x = 0;
    laser_pose.y = 0;
    laser_pose.phi = 0;
    laser.setSensorPose(laser_pose);

    laser.initialize();


    while(!running){
		bool hardError;
        LaserScan scan;//原始激光数据
        LaserScan syncscan;//同步后激光数据
        PointCloud pc;//同步后激光点云数据
        std::vector<gline> lines;

        if(laser.doProcessSimple(scan, syncscan, pc, lines, hardError )){
            for(int i =0; i < scan.ranges.size(); i++ ){
                float angle = scan.config.min_angle + i*scan.config.ang_increment;
                float dis = scan.ranges[i];

            }
            fprintf(stdout,"min_angle: %f \n",scan.config.min_angle);
            fprintf(stdout,"max_angle: %f \n",scan.config.max_angle);

            fprintf(stdout,"Scan received: %u ranges\n",(unsigned int)scan.ranges.size());
            fprintf(stdout, "fit line size: %u \n", (unsigned int)lines.size());
            for(std::vector<gline>::const_iterator it = lines.begin(); it != lines.end(); it++) {
                fprintf(stdout, "line length: %f,   line angle: %f\n", (*it).distance, (*it).angle);
            }
            fflush(stdout);

		}

        {//做imu和odometry数据输入
            odom_info odom;
            odom.x = 0;
            odom.y = 0;
            odom.phi = 0;
            odom.stamp = getTime();
            laser.setSyncOdometry(odom);


        }

	}
  laser.turnOff();
  laser.disconnecting();

  return 0;


}
