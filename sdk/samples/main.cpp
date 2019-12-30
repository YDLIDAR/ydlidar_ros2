
#include "CYdLidar.h"
#include <iostream>
#include <string>
#include <memory>

using namespace std;
using namespace ydlidar;

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_driver.lib")
#endif
CYdLidar laser;


int main(int argc, char *argv[]) {
  printf("__   ______  _     ___ ____    _    ____  \n");
  printf("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\ \n");
  printf(" \\ V /| | | | |    | || | | |/ _ \\ | |_) | \n");
  printf("  | | | |_| | |___ | || |_| / ___ \\|  _ <  \n");
  printf("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\ \n");
  printf("\n");
  fflush(stdout);


  std::string port;
  ydlidar::init(argc, argv);

  std::map<std::string, std::string> ports = CYdLidar::lidarPortList();
  std::map<std::string, std::string>::iterator it;

  if (ports.size() == 1) {
    it = ports.begin();
    port = it->second;
  } else {
    int id = 0;

    for (it = ports.begin(); it != ports.end(); it++) {
      ydlidar::console.show("%d. %s\n", id, it->first.c_str());
      id++;
    }

    if (ports.empty()) {
      ydlidar::console.show("Not Lidar was detected. Please enter the lidar serial port:");
      std::cin >> port;
    } else {
      while (ydlidar::ok()) {
        ydlidar::console.show("Please select the lidar port:");
        std::string number;
        std::cin >> number;

        if ((size_t)atoi(number.c_str()) >= ports.size()) {
          continue;
        }

        it = ports.begin();
        id = atoi(number.c_str());

        while (id) {
          id--;
          it++;
        }

        port = it->second;
        break;
      }
    }
  }

  if (!ydlidar::ok()) {
    return 0;
  }

  int baudrate = 512000;

  float frequency = 10.0;
  std::string info = "Please Input the lidar Scan frequency: ";

  while (ydlidar::ok()) {
    ydlidar::console.show("%s", info.c_str());
    std::string number;
    std::cin >> number;
    frequency = atof(number.c_str());

    if (frequency >= 3 &&  frequency <= 15) {
      break;
    }

    info = "The lidar frequency is wrong, please re-enter: ";
  }

  if (!ydlidar::ok()) {
    return 0;
  }


  laser.setSerialPort(port);
  laser.setSerialBaudrate(baudrate);
  laser.setAutoReconnect(true);//hot plug
  laser.setMaxRange(64.0);
  laser.setMinRange(0.1);
  laser.setMaxAngle(180);
  laser.setMinAngle(-180);
  laser.setSampleRate(20);
  laser.setScanFrequency(frequency);
  laser.setReversion(true);
  laser.setFixedResolution(false);
  bool ret = laser.initialize();

  if (ret) {
    ret = laser.turnOn();
  }


  while (ret && ydlidar::ok()) {
    bool hardError;
    LaserScan scan;

    if (laser.doProcessSimple(scan, hardError)) {
      for (int i = 0; i < scan.ranges.size(); i++) {
        float angle = scan.config.min_angle + i * scan.config.ang_increment;
        float dis = scan.ranges[i];
//        printf("angle: %f, distance: %f\n", angle, dis);

      }

      ydlidar::console.message("Scan received[%llu]: %u ranges is [%f]Hz",
                               scan.self_time_stamp, (unsigned int)scan.ranges.size(),
                               1.0 / scan.config.scan_time);
    } else {
      ydlidar::console.warning("Failed to get Lidar Data");
    }

  }

  laser.turnOff();
  laser.disconnecting();
  return 0;


}
