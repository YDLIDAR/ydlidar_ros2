#include "CYdLidar.h"
#include "common.h"
#include <map>
#include "angles.h"



using namespace std;
using namespace ydlidar;
using namespace impl;


/*-------------------------------------------------------------
                        Constructor
-------------------------------------------------------------*/
CYdLidar::CYdLidar() : lidarPtr(nullptr) {
  m_SerialPort        = "";
  m_SerialBaudrate    = 512000;
  m_FixedResolution   = false;
  m_Reversion         = true;
  m_AutoReconnect     = true;
  m_MaxAngle          = 180.f;
  m_MinAngle          = -180.f;
  m_MaxRange          = 64.0;
  m_MinRange          = 0.08;
  m_SampleRate        = 20;
  m_ScanFrequency     = 10.0;
  isScanning          = false;
  node_counts         = 2800;
  m_FrequencyOffset   = 0.4;
  m_AbnormalCheckCount = 4;
  m_IgnoreArray.clear();
  nodes = new node_info[YDlidarDriver::MAX_SCAN_NODES];
  m_pointTime         = 1e9 / 20000;
  m_packageTime       = 0;        ///零位包传送时间
  last_node_time      = getTime();
  model = YDlidarDriver::YDLIDAR_TG30;
}

/*-------------------------------------------------------------
                    ~CYdLidar
-------------------------------------------------------------*/
CYdLidar::~CYdLidar() {
  disconnecting();

  if (nodes) {
    delete[] nodes;
    nodes = nullptr;
  }
}

void CYdLidar::disconnecting() {
  if (lidarPtr) {
    lidarPtr->disconnect();
    delete lidarPtr;
    lidarPtr = nullptr;
  }

  isScanning = false;
}

std::map<std::string, std::string>  CYdLidar::lidarPortList() {
  return ydlidar::YDlidarDriver::lidarPortList();
}


/*-------------------------------------------------------------
                        doProcessSimple
-------------------------------------------------------------*/
bool  CYdLidar::doProcessSimple(LaserScan &outscan, bool &hardwareError) {
  hardwareError           = false;

  // Bound?
  if (!checkHardware()) {
    delay(1000 / (2 * m_ScanFrequency));
    hardwareError = true;
    return false;
  }

  size_t   count = YDlidarDriver::MAX_SCAN_NODES;
  size_t all_nodes_counts = node_counts;

  //  wait Scan data:
  uint64_t tim_scan_start = getTime();
  uint64_t startTs = tim_scan_start;
  result_t op_result =  lidarPtr->grabScanData(nodes, count);
  uint64_t tim_scan_end = getTime();

  // Fill in scan data:
  if (IS_OK(op_result)) {

    if (!m_FixedResolution) {
      all_nodes_counts = count;
    } else {
      all_nodes_counts = node_counts;
    }

    if (m_MaxAngle < m_MinAngle) {
      float temp = m_MinAngle;
      m_MinAngle = m_MaxAngle;
      m_MaxAngle = temp;
    }

    uint64_t scan_time = m_pointTime * (count - 1);
//    tim_scan_end -= m_packageTime;
    tim_scan_end -= nodes[0].dstamp;
    tim_scan_end -= m_pointTime;
    tim_scan_start = tim_scan_end -  scan_time ;

    if (tim_scan_start < startTs) {
      tim_scan_start = startTs;
      tim_scan_end = tim_scan_start + scan_time;
    }

    if ((last_node_time + m_pointTime) >= tim_scan_start) {
      tim_scan_start = last_node_time + m_pointTime;
      tim_scan_end = tim_scan_start + scan_time;
    }

    last_node_time = tim_scan_end;


    int counts = all_nodes_counts * ((m_MaxAngle - m_MinAngle) / 360.0f);
    outscan.ranges.resize(counts);
    outscan.intensities.resize(counts);
    outscan.system_time_stamp = tim_scan_start;
    outscan.self_time_stamp = tim_scan_start;
    outscan.config.min_angle = angles::from_degrees(m_MinAngle);
    outscan.config.max_angle = angles::from_degrees(m_MaxAngle);
    outscan.config.ang_increment = (outscan.config.max_angle -
                                    outscan.config.min_angle) /
                                   (double)(counts - 1);
    outscan.config.scan_time = static_cast<float>(1.0 * scan_time / 1e9);
    outscan.config.time_increment = outscan.config.scan_time / (double)(counts - 1);
    outscan.config.min_range = m_MinRange;
    outscan.config.max_range = m_MaxRange;


    float range = 0.0;
    float angle = 0.0;
    float intensity = 0.0;
    int index = 0;
    unsigned int i = 0;

    for (; i < count; i++) {

      if (model == YDlidarDriver::YDLIDAR_G4) {
        range = static_cast<float>(nodes[i].distance_q2 / 4000.f);
      } else {
        range = static_cast<float>(nodes[i].distance_q2 / 2000.f);
      }

      intensity = static_cast<float>((nodes[i].sync_quality >>
                                      LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT));
      angle = static_cast<float>(((nodes[i].angle_q6_checkbit >>
                                   LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f));
      angle = angles::from_degrees(angle);
      angle = 2 * M_PI - angle;

      if (m_Reversion) {
        angle += M_PI;
      }

      angle = angles::normalize_angle(angle);

      if (m_IgnoreArray.size() != 0) {
        for (uint16_t j = 0; j < m_IgnoreArray.size(); j = j + 2) {
          if ((angles::from_degrees(m_IgnoreArray[j]) <= angle) &&
              (angle <= angles::from_degrees(m_IgnoreArray[j + 1]))) {
            range = 0.0;
            intensity = 0.0;
            break;
          }
        }
      }

      if (angle >= outscan.config.min_angle && angle <= outscan.config.max_angle) {
        index = (angle - outscan.config.min_angle) / outscan.config.ang_increment + 0.5;

        if (0 <= index && index < counts) {
          if (range > m_MaxRange || range < m_MinRange) {
            range = 0.0;
            intensity = 0.0;
          }

          outscan.ranges[index] =  range;
          outscan.intensities[index] = intensity;
        }
      }
    }

    return true;
  } else {
    if (op_result == RESULT_FAIL) {
      // Error? Retry connection
      //this->disconnect();
    }
  }

  return false;

}


/*-------------------------------------------------------------
                        turnOn
-------------------------------------------------------------*/
bool  CYdLidar::turnOn() {
  if (isScanning && lidarPtr->isscanning()) {
    return true;
  }

  // start scan...
  result_t op_result = lidarPtr->startScan();

  if (!IS_OK(op_result)) {
    op_result = lidarPtr->startScan();

    if (!IS_OK(op_result)) {
      lidarPtr->stop();
      ydlidar::console.error("[CYdLidar] Failed to start scan mode: %x", op_result);
      isScanning = false;
      return false;
    }
  }

  m_pointTime = lidarPtr->getPointTime();

  if (checkLidarAbnormal()) {
    lidarPtr->stop();
    ydlidar::console.error("[CYdLidar] Failed to turn on the Lidar, because the lidar is blocked or the lidar hardware is faulty.");
    isScanning = false;
    return false;
  }

  isScanning = true;
  m_packageTime = lidarPtr->getPackageTime();
  lidarPtr->setAutoReconnect(m_AutoReconnect);
  ydlidar::console.message("[YDLIDAR INFO] Now YDLIDAR is scanning ......");
  fflush(stdout);
  return true;
}

/*-------------------------------------------------------------
                        turnOff
-------------------------------------------------------------*/
bool  CYdLidar::turnOff() {
  if (lidarPtr) {
    lidarPtr->stop();
  }

  if (isScanning) {
    ydlidar::console.message("[YDLIDAR INFO] Now YDLIDAR Scanning has stopped ......");

  }

  isScanning = false;
  return true;
}

bool CYdLidar::checkLidarAbnormal() {
  int check_abnormal_count = 0;

  if (m_AbnormalCheckCount < 2) {
    m_AbnormalCheckCount = 2;
  }

  result_t op_result = RESULT_FAIL;
  size_t   count = YDlidarDriver::MAX_SCAN_NODES;


  while (check_abnormal_count < m_AbnormalCheckCount) {
    //Ensure that the voltage is insufficient or the motor resistance is high, causing an abnormality.
    if (check_abnormal_count > 0) {
      delay(check_abnormal_count * 1000);
    }

    count = YDlidarDriver::MAX_SCAN_NODES;
    op_result =  lidarPtr->grabScanData(nodes, count);

    if (IS_OK(op_result)) {
      return false;
    }

    check_abnormal_count++;
  }

  return !IS_OK(op_result);
}

/** Returns true if the device is connected & operative */
bool CYdLidar::getDeviceHealth() const {
  if (!lidarPtr) {
    return false;
  }

  lidarPtr->stop();
  result_t op_result;
  device_health healthinfo;
  ydlidar::console.message("[YDLIDAR]:SDK Version: %s",
                           YDlidarDriver::getSDKVersion().c_str());
  op_result = lidarPtr->getHealth(healthinfo);

  if (IS_OK(op_result)) {
    if (healthinfo.status == 0) {
      ydlidar::console.message("YDLidar running correctly ! The health status is good");
    } else {
      ydlidar::console.error("YDLidar running correctly ! The health status is bad");
    }

    if (healthinfo.status == 2) {
      ydlidar::console.error("Error, Yd Lidar internal error detected. Please reboot the device to retry.");
      return false;
    } else {
      return true;
    }

  } else {
    ydlidar::console.error("Error, cannot retrieve Yd Lidar health code: %x",
                           op_result);
    return false;
  }

}

bool CYdLidar::getDeviceInfo() {

  if (!lidarPtr) {
    return false;
  }

  device_info devinfo;
  result_t ans = lidarPtr->getDeviceInfo(devinfo);

  if (!IS_OK(ans)) {
    ydlidar::console.error("get DeviceInfo Error");
    return false;
  }

  if (devinfo.model != YDlidarDriver::YDLIDAR_G4 &&
      devinfo.model != YDlidarDriver::YDLIDAR_G6 &&
      devinfo.model != YDlidarDriver::YDLIDAR_TG15 &&
      devinfo.model != YDlidarDriver::YDLIDAR_TG30 &&
      devinfo.model != YDlidarDriver::YDLIDAR_TG50) {
    ydlidar::console.error("[YDLIDAR INFO] Current SDK does not support current lidar models[%d]",
                           devinfo.model);
    return false;
  }

  std::string modelName = "G6";
  model = devinfo.model;

  switch (devinfo.model) {
    case YDlidarDriver::YDLIDAR_G4:
      modelName = "G4";
      break;

    case YDlidarDriver::YDLIDAR_G6:
      modelName = "G6";
      break;

    case YDlidarDriver::YDLIDAR_TG15:
      modelName = "TG15";
      break;

    case YDlidarDriver::YDLIDAR_TG30:
      modelName = "TG30";
      break;

    case YDlidarDriver::YDLIDAR_TG50:
      modelName = "TG50";
      break;

    default:
      break;
  }

  unsigned int Major = (unsigned int)(devinfo.firmware_version >> 8);
  unsigned int Minjor = (unsigned int)(devinfo.firmware_version & 0xff);
  ydlidar::console.show("[YDLIDAR] Connection established in [%s]:\n"
                        "Firmware version: %u.%u\n"
                        "Hardware version: %u\n"
                        "Model: %s\n"
                        "Serial: ",
                        m_SerialPort.c_str(),
                        Major,
                        Minjor,
                        (unsigned int)devinfo.hardware_version,
                        modelName.c_str());

  for (int i = 0; i < 16; i++) {
    ydlidar::console.show("%01X", devinfo.serialnum[i] & 0xff);
  }

  ydlidar::console.show("\n");
  checkSampleRate();
  ydlidar::console.message("[YDLIDAR INFO] Current Sampling Rate : %dK",
                           m_SampleRate);
  checkScanFrequency();
  return true;


}

void CYdLidar::checkSampleRate() {
  sampling_rate _rate;
  _rate.rate = 3;
  int _samp_rate = 20;
  int try_count = 0;
  node_counts = 2880;
  result_t ans = lidarPtr->getSamplingRate(_rate);

  if (IS_OK(ans)) {
    switch (m_SampleRate) {
      case 10:
        _samp_rate = YDlidarDriver::YDLIDAR_RATE_4K;
        break;

      case 16:
        _samp_rate = YDlidarDriver::YDLIDAR_RATE_8K;
        break;

      case 18:
        _samp_rate = YDlidarDriver::YDLIDAR_RATE_9K;
        break;

      case 20:
        _samp_rate = YDlidarDriver::YDLIDAR_RATE_10K;
        break;

      default:
        _samp_rate = _rate.rate;
        break;
    }

    if (model == YDlidarDriver::YDLIDAR_G4) {
      _rate.rate = 2;
      _samp_rate = 9;
      try_count = 0;
      node_counts = 1440;

      switch (m_SampleRate) {
        case 4:
          _samp_rate = YDlidarDriver::YDLIDAR_RATE_4K;
          break;

        case 8:
          _samp_rate = YDlidarDriver::YDLIDAR_RATE_8K;
          break;

        case 9:
          _samp_rate = YDlidarDriver::YDLIDAR_RATE_9K;
          break;

        default:
          _samp_rate = _rate.rate;
          break;
      }
    }

    while (_samp_rate != _rate.rate) {
      ans = lidarPtr->setSamplingRate(_rate);
      try_count++;

      if (try_count > 6) {
        break;
      }
    }

    switch (_rate.rate) {
      case YDlidarDriver::YDLIDAR_RATE_4K:
        _samp_rate = 10;
        node_counts = 1440;

        if (model == YDlidarDriver::YDLIDAR_G4) {
          _samp_rate = 4;
          node_counts = 720;
        }

        break;

      case YDlidarDriver::YDLIDAR_RATE_8K:
        node_counts = 2400;
        _samp_rate = 16;

        if (model == YDlidarDriver::YDLIDAR_G4) {
          _samp_rate = 8;
          node_counts = 1440;
        }

        break;

      case YDlidarDriver::YDLIDAR_RATE_9K:
        node_counts = 2600;
        _samp_rate = 18;

        if (model == YDlidarDriver::YDLIDAR_G4) {
          _samp_rate = 9;
          node_counts = 1440;
        }

        break;

      case YDlidarDriver::YDLIDAR_RATE_10K:
        node_counts = 2800;
        _samp_rate = 20;
        break;

      default:
        break;
    }
  }

  m_SampleRate = _samp_rate;

}

/*-------------------------------------------------------------
                        checkScanFrequency
-------------------------------------------------------------*/
bool CYdLidar::checkScanFrequency() {
  float freq = 7.4f;
  scan_frequency _scan_frequency;
  float hz = 0;
  result_t ans = RESULT_FAIL;
  m_ScanFrequency += m_FrequencyOffset;
  int retryCount = 0;

  if (3.0 - m_FrequencyOffset <= m_ScanFrequency &&
      m_ScanFrequency <= 15 + m_FrequencyOffset) {
    ans = lidarPtr->getScanFrequency(_scan_frequency);

    if (!IS_OK(ans)) {
      ans = lidarPtr->getScanFrequency(_scan_frequency);
    }

    if (IS_OK(ans)) {
TryAgain:
      freq = _scan_frequency.frequency / 100.f;
      hz = m_ScanFrequency - freq;

      if (hz > 0) {
        while (hz > 0.95) {
          lidarPtr->setScanFrequencyAdd(_scan_frequency);
          hz = hz - 1.0;
        }

        while (hz > 0.09) {
          lidarPtr->setScanFrequencyAddMic(_scan_frequency);
          hz = hz - 0.1;
        }

        freq = _scan_frequency.frequency / 100.0f;
      } else {
        while (hz < -0.95) {
          lidarPtr->setScanFrequencyDis(_scan_frequency);
          hz = hz + 1.0;
        }

        while (hz < -0.09) {
          lidarPtr->setScanFrequencyAddMic(_scan_frequency);
          hz = hz + 0.1;
        }

        freq = _scan_frequency.frequency / 100.0f;
      }

      ans = lidarPtr->getScanFrequency(_scan_frequency);

      if (!IS_OK(ans)) {
        ans = lidarPtr->getScanFrequency(_scan_frequency);
      }

      if (IS_OK(ans)) {
        freq = _scan_frequency.frequency / 100.f;

        if (freq != m_ScanFrequency) {
          retryCount++;

          if (retryCount < 4) {
            goto TryAgain;
          } else {
            ydlidar::console.error("Failed to set scan frequency...");
          }
        }


      } else {
        ydlidar::console.warning("Failed to get scan frequency...");
      }

    } else {
      ydlidar::console.warning("Failed to get scan frequency......");
    }
  } else {
    ydlidar::console.warning("current scan frequency[%f] is out of range.",
                             m_ScanFrequency - m_FrequencyOffset);
  }

  ans = lidarPtr->getScanFrequency(_scan_frequency);

  if (IS_OK(ans)) {
    freq = _scan_frequency.frequency / 100.0f;
    m_ScanFrequency = freq;
  }

  m_ScanFrequency -= m_FrequencyOffset;
  node_counts = m_SampleRate * 1000 / (m_ScanFrequency - m_FrequencyOffset);
  ydlidar::console.message("[YDLIDAR INFO] Current Scan Frequency : %fHz",
                           freq - m_FrequencyOffset);

  return true;

}

/*-------------------------------------------------------------
                        checkCOMMs
-------------------------------------------------------------*/
bool  CYdLidar::checkCOMMs() {
  if (!lidarPtr) {
    // create the driver instance
    lidarPtr = new YDlidarDriver();

    if (!lidarPtr) {
      ydlidar::console.error("Create Driver fail");
      return false;
    }
  }

  if (lidarPtr->isconnected()) {
    return true;
  }

  // Is it COMX, X>4? ->  "\\.\COMX"
  if (m_SerialPort.size() >= 3) {
    if (tolower(m_SerialPort[0]) == 'c' && tolower(m_SerialPort[1]) == 'o' &&
        tolower(m_SerialPort[2]) == 'm') {
      // Need to add "\\.\"?
      if (m_SerialPort.size() > 4 || m_SerialPort[3] > '4') {
        m_SerialPort = std::string("\\\\.\\") + m_SerialPort;
      }
    }
  }

  // make connection...
  result_t op_result = lidarPtr->connect(m_SerialPort.c_str(), m_SerialBaudrate);

  if (!IS_OK(op_result)) {
    ydlidar::console.error("[CYdLidar] Error, cannot bind to the specified serial port %s",
                           m_SerialPort.c_str());
    return false;
  }

  return true;
}

/*-------------------------------------------------------------
                        checkStatus
-------------------------------------------------------------*/
bool CYdLidar::checkStatus() {

  if (!checkCOMMs()) {
    return false;
  }

  bool ret = getDeviceHealth();

  if (!ret) {
    delay(1000);
    ret = getDeviceHealth();

    if (!ret) {
      delay(1000);
    }
  }

  if (!getDeviceInfo()) {
    delay(1000);
    ret = getDeviceInfo();

    if (!ret) {
      return false;
    }
  }

  return true;
}

/*-------------------------------------------------------------
                        checkHardware
-------------------------------------------------------------*/
bool CYdLidar::checkHardware() {
  if (!lidarPtr) {
    return false;
  }

  if (isScanning && lidarPtr->isscanning()) {
    return true;
  }

  return false;
}

/*-------------------------------------------------------------
                        initialize
-------------------------------------------------------------*/
bool CYdLidar::initialize() {
  bool ret = true;

  if (!checkCOMMs()) {
    ydlidar::console.error("[CYdLidar::initialize] Error initializing YDLIDAR scanner.");
    return false;
  }

  if (!checkStatus()) {
    ydlidar::console.warning("[CYdLidar::initialize] Error initializing YDLIDAR scanner.Failed to get Device information.");
    return false;
  }

  return ret;

}
