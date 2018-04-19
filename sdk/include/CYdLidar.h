
#pragma once
#include "utils.h"
#include "ydlidar_driver.h"

#define PI 3.1415926

class YDLIDAR_API CYdLidar
{
public:
	CYdLidar(); //!< Constructor
	virtual ~CYdLidar();  //!< Destructor: turns the laser off.

	void initialize();  //!< Attempts to connect and turns the laser on. Raises an exception on error.

			// See base class docs
	bool  doProcessSimple(LaserScan &outscan, bool &hardwareError);

			/** If set to non-empty, the serial port will be attempted to be opened automatically when this class is first used to request data from the laser.  */
	void setSerialPort(const std::string &port_name);
	const std::string getSerialPort() { return m_com_port; }  //!< Returns the currently set serial port \sa setSerialPort

			/** If set to non-empty, the serial port will be attempted to be opened automatically when this class is first used to request data from the laser.  */
	void setSerialBaud(const uint32_t baudrate);
	const uint32_t getSerialBaud() { return m_com_port_baudrate; }  //!< Returns the currently set serial port \sa setSerialPort


	void setIntensities( bool  intensity = false);
	const uint32_t getIntensities() { return m_intensity; }  //!< Returns the currently set serial port \sa setSerialPort

	void setMaxRange(const float range);

	void setMinRange(const float range);


	const float getMaxRange(){ return max_range;}
	const float getMinRange(){ return min_range;}

	bool  turnOn();  //!< See base class docs
	bool  turnOff(); //!< See base class docs

			/** Returns true if the device is connected & operative */
	bool getDeviceHealth() const;
	bool getDeviceDeviceInfo() const;

    void disconnecting(); //!< Closes the comms with the laser. Shouldn't have to be directly needed by the user

protected:
	std::string     m_com_port;
	uint32_t             m_com_port_baudrate;
	bool            m_intensity;
	float 			max_range, min_range;
			//float angle_max,angle_min;

			/** Returns true if communication has been established with the device. If it's not, 
			  *  try to create a comms channel. 
			  * \return false on error.
			  */
	bool  checkCOMMs(); 

	static int NODE_COUNTS ;
	static double EACH_ANGLE;

};	// End of class

