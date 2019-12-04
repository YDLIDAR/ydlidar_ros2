
#pragma once
#include "line_feature.h"
#include "utils.h"
#include "ydlidar_driver.h"
#include <math.h>
#include <SimpleIni.h>
#include "angles.h"

using namespace ydlidar;
using namespace line_feature;
using namespace angles;

class YDLIDAR_API CYdLidar {
  PropertyBuilderByName(float, MaxRange,
                        private) ///< Constrained maximum distance(m)
  PropertyBuilderByName(float, MinRange,
                        private) ///< Constrained minimum distance(m)
  PropertyBuilderByName(float, MaxAngle,
                        private) ///< constrained maximum angle, Maximum 360 Deg(Deg)
  PropertyBuilderByName(float, MinAngle,
                        private) ///< constrained minimum angle, minmum 0 Deg(Deg)
  PropertyBuilderByName(float, ScanFrequency,
                        private) ///< scan frequency (5HZ~12HZ)(HZ)

  PropertyBuilderByName(float, RobotLidarDifference,
                        private) ///< lidar zero and robot zero difference(°)

  PropertyBuilderByName(bool, Intensities,
                        private) ///< intensity
  PropertyBuilderByName(bool, AutoReconnect,
                        private) ///< whether to support hot swap
  PropertyBuilderByName(bool, GlassNoise,
                        private) ///< whether to close glass noise
  PropertyBuilderByName(bool, SunNoise, private) ///< whether to close sun noise
  PropertyBuilderByName(int, SerialBaudrate, private) ///< serial baudrate
  PropertyBuilderByName(int, SampleRate, private) ///< sampling rate(KHz)
  PropertyBuilderByName(int, AbnormalCheckCount,
                        private) ///< Maximum number of abnormal checks
  PropertyBuilderByName(std::string, CalibrationFileName,
                        private) ///< calibration file
  PropertyBuilderByName(std::string, SerialPort, private) ///< serial port
  PropertyBuilderByName(std::vector<float>, IgnoreArray,
                        private) ///< Culling angle list
  PropertyBuilderByName(bool, FilterDataNoise,
                        private) ///< whether to close data overlap noise


 public:
  CYdLidar(); //!< Constructor
  virtual ~CYdLidar();  //!< Destructor: turns the laser off.
  /**
   * @brief initialize
   * @return
   */
  bool initialize();  //!< Attempts to connect and turns the laser on. Raises an exception on error.

  /** Returns true if the normal scan runs with the device. If it's not,
    * \return false on error.
    */
  bool checkHardware();

  // Return true if laser data acquistion succeeds, If it's not
  bool doProcessSimple(LaserScan &outscan, bool &hardwareError);

  //Turn on the motor enable
  bool  turnOn();  //!< See base class docs

  //Turn off the motor enable and close the scan
  bool  turnOff(); //!< See base class docs

  //Turn off lidar connection
  void disconnecting(); //!< Closes the comms with the laser. Shouldn't have to be directly needed by the user

  //lidar pointer
  YDlidarDriver *getYdlidarDriver();

  //get zero angle offset value
  float getAngleOffset() const;

  //Whether the zero offset angle is corrected?
  bool isAngleOffetCorrected() const;

  // get lidar relative robot offset angle
  float getRobotAngleOffset() const;

  // start lidar is corrected relative to the robot.
  void setStartRobotAngleOffset();

  //Whether the lidar relative robot offset angle is corrected?
  //After the corrrection is started,
  //the currect interface can be used to datermine when the correction is completed.
  bool isRobotAngleOffsetCorrected() const;

  int getFixedSize() const;

 protected:
  /** Returns true if communication has been established with the device. If it's not,
    *  try to create a comms channel.
    * \return false on error.
    */
  bool  checkCOMMs();

  /** Returns true if health status and device information has been obtained with the device. If it's not,
    * \return false on error.
    */
  bool  checkStatus();

  /**
   * @brief checkSampleRate
   */
  void checkSampleRate();

  /**
   * @brief checkCalibrationAngle
   */
  void checkCalibrationAngle(const std::string &serialNumber);

  /**
   * @brief checkRobotOffsetAngleCorrected
   * @param serialNumber
   */
  void checkRobotOffsetAngleCorrected(const std::string &serialNumber);

  /** Returns true if the device is in good health, If it's not*/
  bool getDeviceHealth();

  /** Returns true if the device information is correct, If it's not*/
  bool getDeviceInfo();

  /** Retruns true if the scan frequency is set to user's frequency is successful, If it's not*/
  bool checkScanFrequency();

  /** returns true if the lidar data is normal, If it's not*/
  bool checkLidarAbnormal();

  /**
   * @brief saveRobotOffsetAngle
   * @return
   */
  void saveRobotOffsetAngle();

  bool checkSingleChannelLidar();

  void fitLineFeature();

 private:
  bool    isScanning;
  float   frequencyOffset;
  float   m_AngleOffset;
  bool    m_isAngleOffsetCorrected;
  float   m_LRRAngleOffset;
  bool    m_isLRRAngleOffsetCorrected;//lidar relative robot angle offset
  bool    m_startRobotAngleOffset;
  uint8_t Major;
  uint8_t Minjor;
  CSimpleIniA ini;
  YDlidarDriver *lidarPtr;
  LineFeature line_feature_;
  std::string m_serial_number;
  node_info *global_nodes;
  uint32_t m_pointTime;
  uint64_t last_node_time;
  int     node_counts ;
  bool m_isSingleChannel;

  //line feature
  std::vector<double> bearings;
  std::vector<unsigned int> indices;
  RangeData range_data;

};	// End of class

