#include "CYdLidar.h"
#include "common.h"
#include <map>



using namespace std;
using namespace ydlidar;
using namespace impl;


/*-------------------------------------------------------------
						Constructor
-------------------------------------------------------------*/
CYdLidar::CYdLidar()
{
    m_SerialPort = "";
    m_SerialBaudrate = 115200;
    m_Intensities = false;
    m_FixedResolution = false;
    m_Exposure = false;
    m_HeartBeat = false;
    m_Reversion = false;
    m_AutoReconnect = true;
    m_EnableDebug = false;
    m_MaxAngle = 180.f;
    m_MinAngle = -180.f;
    m_MaxRange = 16.0;
    m_MinRange = 0.08;
    m_SampleRate = 9;
    m_ScanFrequency = 7;
    isScanning = false;
    reversion	= false;
    node_counts = 720;
    each_angle = 0.5;
    show_error = 0;
    m_IgnoreArray.clear();

    sensor_matrix.setIdentity();
    sensor_matrix_inv.setIdentity();
    robot_matrix.setIdentity();
    current_sensor_vector.setOne();
    lidar_sensor_vector.setOne();

}

/*-------------------------------------------------------------
                    ~CYdLidar
-------------------------------------------------------------*/
CYdLidar::~CYdLidar()
{
    disconnecting();
}

void CYdLidar::disconnecting()
{
    if (YDlidarDriver::singleton()) {
        YDlidarDriver::singleton()->disconnect();
        YDlidarDriver::done();
    }
}

void CYdLidar::setSyncOdometry(const odom_info &odom) {
    if( YDlidarDriver::singleton() )  {
        YDlidarDriver::singleton()->setSyncOdometry(odom);
    }
}

void CYdLidar::setSensorPose(const pose_info &pose) {
    sensor_matrix(0, 0) = cos(pose.phi);
    sensor_matrix(0, 1) = -sin(pose.phi);
    sensor_matrix(0, 2) = pose.x;

    sensor_matrix(1, 0) = sin(pose.phi);
    sensor_matrix(1, 1) = cos(pose.phi);
    sensor_matrix(1, 2) = pose.y;

    sensor_matrix(2, 0) = 0;
    sensor_matrix(2, 1) = 0;
    sensor_matrix(2, 2) = 1;
    sensor_matrix_inv = matrix::inv(sensor_matrix);

}

/*-------------------------------------------------------------
						doProcessSimple
-------------------------------------------------------------*/
bool  CYdLidar::doProcessSimple(LaserScan &outscan,LaserScan &syncscan, PointCloud &pointcloud, std::vector<gline>& lines, bool &hardwareError){
	hardwareError			= false;

	// Bound?
    if (!checkHardware())
	{
        hardwareError = true;
        return false;
	}

    node_info nodes[2048];
    size_t   count = _countof(nodes);

    size_t all_nodes_counts = node_counts;

    //  wait Scan data:
    uint64_t tim_scan_start = getTime();
	result_t op_result =  YDlidarDriver::singleton()->grabScanData(nodes, count);
    const uint64_t tim_scan_end = getTime();

    std::vector<double> bearings;
    std::vector<unsigned int> indices;
    RangeData  rangedata;
    int current_index = 0;
    lines.clear();


	// Fill in scan data:
	if (op_result == RESULT_OK)
	{
		op_result = YDlidarDriver::singleton()->ascendScanData(nodes, count);
		//同步后的时间
		if(nodes[0].stamp > 0){
            tim_scan_start = nodes[0].stamp;
        }
		const double scan_time = tim_scan_end - tim_scan_start;
		if (op_result == RESULT_OK)
		{
            if(!m_FixedResolution){
                all_nodes_counts = count;
            } else {
                all_nodes_counts = node_counts;
            }
            each_angle = 360.0/all_nodes_counts;

            node_info *angle_compensate_nodes = new node_info[all_nodes_counts];
            memset(angle_compensate_nodes, 0, all_nodes_counts*sizeof(node_info));
            unsigned int i = 0;
            for( ; i < count; i++) {
                if ((nodes[i].distance_q) != 0) {
                    float angle = (float)((nodes[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
                    if(reversion&&!m_Reversion){
                       angle=angle+180;
                       if(angle>=360){ angle=angle-360;}
                        nodes[i].angle_q6_checkbit = ((uint16_t)(angle * 64.0f)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT;
                    }
                    int inter =(int)( angle / each_angle );
                    float angle_pre = angle - inter * each_angle;
                    float angle_next = (inter+1) * each_angle - angle;
                    if (angle_pre < angle_next) {
                        if(inter < all_nodes_counts)
                            angle_compensate_nodes[inter]=nodes[i];
                    } else {
                        if (inter < all_nodes_counts -1)
                            angle_compensate_nodes[inter+1]=nodes[i];
                    }
                }
             }

            LaserScan scan_msg;
            PointCloud pc;
            pc.system_time_stamp = tim_scan_start;

            if (m_MaxAngle< m_MinAngle) {
                float temp = m_MinAngle;
                m_MinAngle = m_MaxAngle;
                m_MaxAngle = temp;
            }


            int counts = all_nodes_counts*((m_MaxAngle-m_MinAngle)/360.0f);
            int angle_start = 180+m_MinAngle;
            int node_start = all_nodes_counts*(angle_start/360.0f);

            scan_msg.ranges.resize(counts);
            scan_msg.intensities.resize(counts);
            float range = 0.0;
            float intensity = 0.0;
            int index = 0;

            scan_msg.system_time_stamp = tim_scan_start;
            scan_msg.self_time_stamp = tim_scan_start;
            scan_msg.config.min_angle = DEG2RAD(m_MinAngle);
            scan_msg.config.max_angle = DEG2RAD(m_MaxAngle);
            scan_msg.config.ang_increment = (scan_msg.config.max_angle - scan_msg.config.min_angle) / (double)counts;
            scan_msg.config.time_increment = scan_time / (double)counts;
            scan_msg.config.scan_time = scan_time;
            scan_msg.config.min_range = m_MinRange;
            scan_msg.config.max_range = m_MaxRange;

            LaserScan correction_scan_msg = scan_msg;



            for (size_t i = 0; i < all_nodes_counts; i++) {
                range = (float)(angle_compensate_nodes[i].distance_q)/1000.f;
                intensity = (float)(angle_compensate_nodes[i].sync_quality);

                if (i<all_nodes_counts/2) {
                    index = all_nodes_counts/2-1-i;
                } else {
                    index =all_nodes_counts-1-(i-all_nodes_counts/2);
                }

                if (m_IgnoreArray.size() != 0) {
                    float angle = (float)((angle_compensate_nodes[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
                    if (angle>180) {
                        angle=360-angle;
                    } else {
                        angle=-angle;
                    }

                    for (uint16_t j = 0; j < m_IgnoreArray.size();j = j+2) {
                        if ((m_IgnoreArray[j] < angle) && (angle <= m_IgnoreArray[j+1])) {
                           range = 0.0;
                           break;
                        }
                    }
                }

                if (range > m_MaxRange|| range < m_MinRange) {
                    range = 0.0;
                }

                int pos = index - node_start ;
                if (0 <= pos && pos < counts) {
                    {
                        //校准激光雷达数据
                        current_sensor_vector(0) = range*cos(correction_scan_msg.config.min_angle + correction_scan_msg.config.ang_increment*pos);
                        current_sensor_vector(1) = range*sin(correction_scan_msg.config.min_angle + correction_scan_msg.config.ang_increment*pos);
                        current_sensor_vector(2) = 1;

                        double dth = angle_compensate_nodes[i].current_odom.dth;
                        double dx  = angle_compensate_nodes[i].current_odom.dx;
                        double dy  = angle_compensate_nodes[i].current_odom.dy;

                        robot_matrix.setIdentity();
                        robot_matrix(0, 0) = cos(dth);
                        robot_matrix(0, 1) = sin(dth);
                        robot_matrix(0, 2) = dx;
                        robot_matrix(1, 0) = -sin(dth);
                        robot_matrix(1, 1) = cos(dth);
                        robot_matrix(1, 2) = dy;
                        lidar_sensor_vector = sensor_matrix_inv*robot_matrix*sensor_matrix*current_sensor_vector;

                        double lx = lidar_sensor_vector(0);
                        double ly = lidar_sensor_vector(1);
                        point_info point;
                        point.x = lx;
                        point.y = ly;
                        point.z = 0.0;
                        pc.points.push_back(point);
                        double newrange = hypot(lx, ly);
                        double angle = atan2(ly, lx);
                        int newindex = (angle - correction_scan_msg.config.min_angle) / correction_scan_msg.config.ang_increment;
                        if( 0 <= newindex && newindex < counts) {
                            if( newrange < correction_scan_msg.config.min_angle)
                                newrange = 0.0;
                            correction_scan_msg.ranges[newindex] = newrange;
                            correction_scan_msg.intensities[pos] = intensity;

                            if( newrange >= correction_scan_msg.config.min_range) {
                                bearings.push_back(angle);
                                indices.push_back(current_index);
                                rangedata.ranges.push_back(newrange);
                                rangedata.xs.push_back(lx);
                                rangedata.ys.push_back(ly);
                                current_index++;
                            }
                        }

                    }

                    scan_msg.ranges[pos] =  range;
                    scan_msg.intensities[pos] = intensity;
                }
            }


            line_feature_.setCachedRangeData(bearings, indices, rangedata);
            line_feature_.extractLines(lines);
            syncscan = correction_scan_msg;
            pointcloud = pc;
            outscan = scan_msg;
            delete[] angle_compensate_nodes;
            return true;
		}

    } else {
        if (op_result==RESULT_FAIL) {
			// Error? Retry connection
			//this->disconnect();
		}
	}

	return false;

}


/*-------------------------------------------------------------
						turnOn
-------------------------------------------------------------*/
bool  CYdLidar::turnOn()
{
    bool ret = false;
    if (isScanning) {
		YDlidarDriver::singleton()->startMotor();
        ret = true;
	}

	return ret;
}

/*-------------------------------------------------------------
						turnOff
-------------------------------------------------------------*/
bool  CYdLidar::turnOff()
{
	if (YDlidarDriver::singleton()) {
		YDlidarDriver::singleton()->stop();
		YDlidarDriver::singleton()->stopMotor();
        isScanning = false;
	}
	return true;
}

/** Returns true if the device is connected & operative */
bool CYdLidar::getDeviceHealth() const {
	if (!YDlidarDriver::singleton()) return false;

	result_t op_result;
    device_health healthinfo;

	op_result = YDlidarDriver::singleton()->getHealth(healthinfo);
    if (op_result == RESULT_OK) {
        printf("Yd Lidar running correctly ! The health status: %s\n", (int)healthinfo.status==0?"good":"bad");

        if (healthinfo.status == 2) {
            if (show_error == 3)
                fprintf(stderr, "Error, Yd Lidar internal error detected. Please reboot the device to retry.\n");
            return false;
        } else {
                return true;
        }

    } else {
        if (show_error == 3)
            fprintf(stderr, "Error, cannot retrieve Yd Lidar health code: %x\n", op_result);
        return false;
    }

}

bool CYdLidar::getDeviceInfo(int &type) {

	if (!YDlidarDriver::singleton()) return false;

	device_info devinfo;
    if (YDlidarDriver::singleton()->getDeviceInfo(devinfo) != RESULT_OK ) {
        if (show_error == 3)
            fprintf(stderr, "get DeviceInfo Error\n" );
		return false;
	}	 
	std::string model;
    sampling_rate _rate;
    int _samp_rate=4;
    result_t ans;
    int bad = 0;

    type = devinfo.model;
    switch (devinfo.model) {
        case 1:
            model="F4";
            break;
        case 2:
            model="T1";
            break;
        case 3:
            model="F2";
            break;
        case 4:
            model="S4";
            break;
        case 5:
        {
            model="G4";
            ans = YDlidarDriver::singleton()->getSamplingRate(_rate);
            if (ans == RESULT_OK) {
                switch (m_SampleRate) {
                case 4:
                    _samp_rate=0;
                    break;
                case 8:
                    _samp_rate=1;
                    break;
                case 9:
                    _samp_rate=2;
                    break;
                default:
                    _samp_rate = _rate.rate;
                    break;
                }

                while (_samp_rate != _rate.rate) {
                    ans = YDlidarDriver::singleton()->setSamplingRate(_rate);
                    if (ans != RESULT_OK) {
                        bad++;
                        if(bad>5){
                            break;
                        }
                    }
                }

                switch (_rate.rate) {
                    case 0:
                        _samp_rate = 4;
                        break;
                    case 1:
                        node_counts = 1440;
                        each_angle = 0.25;
                        _samp_rate=8;
                        break;
                    case 2:
                        node_counts = 1440;
                        each_angle = 0.25;
                        _samp_rate=9;
                        break;
                }


            }
	    reversion = true;

	    }
            break;
        case 6:
            model="X4";
        case 8:
        {
            model="F4Pro";
            ans = YDlidarDriver::singleton()->getSamplingRate(_rate);
            if (ans == RESULT_OK) {
                switch (m_SampleRate) {
                case 4:
                    _samp_rate=0;
                    break;
                case 6:
                    _samp_rate=1;
                    break;
                default:
                    _samp_rate = _rate.rate;
                    break;
                }
                while (_samp_rate != _rate.rate) {
                    ans = YDlidarDriver::singleton()->setSamplingRate(_rate);
                    if (ans != RESULT_OK) {
                        bad++;
                        if(bad>5){
                            break;
                        }
                    }
                }

                switch (_rate.rate) {
                    case 0:
                        _samp_rate = 4;
                        break;
                    case 1:
                        node_counts = 1440;
                        each_angle = 0.25;
                        _samp_rate=6;
                        break;
                }

            }

        }
            break;
        case 9:
            model = "G4C";
	    reversion = true;
            break;
        default:
            model = "Unknown";
            break;
    }

    m_SampleRate = _samp_rate;



    unsigned int maxv = (unsigned int)(devinfo.firmware_version>>8);
    unsigned int midv = (unsigned int)(devinfo.firmware_version&0xff)/10;
    unsigned int minv = (unsigned int)(devinfo.firmware_version&0xff)%10;
    fprintf(stderr, "firmware: %i\n", devinfo.firmware_version);

	printf("[YDLIDAR] Connection established in [%s]:\n"
			   "Firmware version: %u.%u.%u\n"
			   "Hardware version: %u\n"
			   "Model: %s\n"
			   "Serial: ",
                m_SerialPort.c_str(),
			    maxv,
			    midv,
                minv,
			    (unsigned int)devinfo.hardware_version,
			    model.c_str());

		for (int i=0;i<16;i++)
			printf("%01X",devinfo.serialnum[i]&0xff);
		printf("\n");

        printf("[YDLIDAR INFO] Current Sampling Rate : %dK\n" , _samp_rate);


        float freq = 7.0f;
        if (devinfo.model == 5 || devinfo.model ==8 || devinfo.model == 9) {
            checkScanFrequency();
            checkHeartBeat();
        } else {
            printf("[YDLIDAR INFO] Current Scan Frequency : %fHz\n" , freq);
        }

		return true;
	

}

/*-------------------------------------------------------------
                        checkScanFrequency
-------------------------------------------------------------*/
bool CYdLidar::checkScanFrequency()
{
    float freq = 7.0f;
    scan_frequency _scan_frequency;
    int hz = 0;
    if (5 <= m_ScanFrequency && m_ScanFrequency <= 12) {
        result_t ans = YDlidarDriver::singleton()->getScanFrequency(_scan_frequency) ;
        if (ans == RESULT_OK) {
            freq = _scan_frequency.frequency/100.f;
            hz = m_ScanFrequency - freq;
            if (hz>0) {
                while (hz != 0) {
                    YDlidarDriver::singleton()->setScanFrequencyAdd(_scan_frequency);
                    hz--;
                }
                freq = _scan_frequency.frequency/100.0f;
            } else {
                while (hz != 0) {
                    YDlidarDriver::singleton()->setScanFrequencyDis(_scan_frequency);
                    hz++;
                }
                freq = _scan_frequency.frequency/100.0f;
            }
        }
        if(fabs(m_ScanFrequency - freq) < 1.0) {
            hz = (m_ScanFrequency - freq)*10;
            if (hz>0) {
                while (hz != 0) {
                    YDlidarDriver::singleton()->setScanFrequencyAddMic(_scan_frequency);
                    hz--;
                }
                freq = _scan_frequency.frequency/100.0f;
            } else {
                while (hz != 0) {
                    YDlidarDriver::singleton()->setScanFrequencyDisMic(_scan_frequency);
                    hz++;
                }
                freq = _scan_frequency.frequency/100.0f;
            }

        }

        if (m_ScanFrequency < 7 && m_SampleRate>6) {
            node_counts = 1600;

        } else if ( m_ScanFrequency < 6 && m_SampleRate == 9) {
            node_counts = 2000;

        } else if ( m_ScanFrequency < 6 && m_SampleRate == 4) {
            node_counts = 900;
        }
        each_angle = 360.0/node_counts;
    }

    printf("[YDLIDAR INFO] Current Scan Frequency : %fHz\n" , freq);

    return true;

}

/*-------------------------------------------------------------
                        checkHeartBeat
-------------------------------------------------------------*/

bool CYdLidar::checkHeartBeat() const
{
    bool ret = false;
    scan_heart_beat beat;
    if( m_HeartBeat ) {
        Sync:
        result_t ans = YDlidarDriver::singleton()->setScanHeartbeat(beat);
        if( ans == RESULT_OK) {
            if( beat.enable ) {
                ans = YDlidarDriver::singleton()->setScanHeartbeat(beat);
                if( ans == RESULT_OK) {
                    if(!beat.enable) {
                        YDlidarDriver::singleton()->setHeartBeat(true);
                        ret = true;
                        return ret;
                    }
                }
                goto Sync;
            } else  {
                YDlidarDriver::singleton()->setHeartBeat(true);
                ret = true;
                return ret;
            }
        }
        goto Sync;
    }else {
        YDlidarDriver::singleton()->setHeartBeat(false);
        ret = true;
    }

    return ret;

}
/*-------------------------------------------------------------
						checkCOMMs
-------------------------------------------------------------*/
bool  CYdLidar::checkCOMMs()
{
    if (!YDlidarDriver::singleton()) {
        // create the driver instance
        YDlidarDriver::initDriver();
        if (!YDlidarDriver::singleton()) {
             fprintf(stderr, "Create Driver fail\n");
            return false;

        }

    }
    if (YDlidarDriver::singleton()->isconnected()) {
        return true;
    }

	// Is it COMX, X>4? ->  "\\.\COMX"
    if (m_SerialPort.size()>=3) {
        if ( tolower( m_SerialPort[0]) =='c' && tolower( m_SerialPort[1]) =='o' && tolower( m_SerialPort[2]) =='m' ) {
			// Need to add "\\.\"?
            if (m_SerialPort.size()>4 || m_SerialPort[3]>'4')
                m_SerialPort = std::string("\\\\.\\") + m_SerialPort;
		}
	}

	// make connection...
    YDlidarDriver::singleton()->setSaveParse(m_EnableDebug, "/tmp/ydldiar_scan.txt");
    result_t op_result = YDlidarDriver::singleton()->connect(m_SerialPort.c_str(), m_SerialBaudrate);
    if (op_result != RESULT_OK) {
        fprintf(stderr, "[CYdLidar] Error, cannot bind to the specified serial port %s\n",  m_SerialPort.c_str() );
		return false;
	}

	return true;
}

/*-------------------------------------------------------------
                        checkStatus
-------------------------------------------------------------*/
bool CYdLidar::checkStatus()
{

    if (!YDlidarDriver::singleton())
        return false;
    if (YDlidarDriver::singleton()->isscanning())
        return true;

    std::map<int, bool> checkmodel;
    checkmodel.insert(std::map<int, bool>::value_type(115200, false));
    checkmodel.insert(std::map<int, bool>::value_type(128000, false));
    checkmodel.insert(std::map<int, bool>::value_type(153600, false));
    checkmodel.insert(std::map<int, bool>::value_type(230400, false));

    again:
    // check health:
    bool ret = getDeviceHealth();

    int m_type;
    bool check_error = false;
    if (!ret || !getDeviceInfo(m_type)){
        check_error = true;
        checkmodel[m_SerialBaudrate] = true;
        map<int,bool>::iterator it;
        for (it=checkmodel.begin(); it!=checkmodel.end(); ++it) {
            if(it->second)
                continue;

            show_error++;
            YDlidarDriver::singleton()->disconnect();
            YDlidarDriver::done();
            YDlidarDriver::initDriver();
            if (!YDlidarDriver::singleton()) {
                printf("YDLIDAR Create Driver fail, exit\n");
                return false;
            }
            m_SerialBaudrate = it->first;

            bool ret = checkCOMMs();
            if (!ret) {
                return false;
            }
            goto again;
        }

        return false;
    }

    show_error = 0;
    if(m_type != 4 || check_error)
        m_Intensities = false;
    if (m_type == 4) {
        if (m_SerialBaudrate == 153600)
            m_Intensities = true;
        if (m_Intensities) {
            scan_exposure exposure;
            int cnt = 0;
            while ((YDlidarDriver::singleton()->setLowExposure(exposure) == RESULT_OK) && (cnt<3)) {
                if (exposure.exposure != m_Exposure) {
                        printf("set EXPOSURE MODEL SUCCESS!!!\n");
                        break;
                }
                cnt++;
            }
            if (cnt>=3) {

                fprintf(stderr,"set LOW EXPOSURE MODEL FALIED!!!\n");
            }
        }
    }

    YDlidarDriver::singleton()->setIntensities(m_Intensities);

     // start scan...
    result_t s_result= YDlidarDriver::singleton()->startScan();
    if (s_result != RESULT_OK) {
        fprintf(stderr, "[CYdLidar] Error starting scanning mode: %x\n", s_result);
        isScanning = false;
        return false;
    }
    YDlidarDriver::singleton()->setAutoReconnect(m_AutoReconnect);
    printf("[YDLIDAR INFO] Now YDLIDAR is scanning ......\n");
    fflush(stdout);
    fflush(stderr);
    isScanning = true;
    return true;

}

/*-------------------------------------------------------------
                        checkHardware
-------------------------------------------------------------*/
bool CYdLidar::checkHardware()
{
    bool ret = true;
    if (!isScanning) {
        ret = false;
        if (checkCOMMs()) {
            if (checkStatus()) {
                if (turnOn()) {
                    ret = true;
                }
            }
        }
    }

    return ret;
}

/*-------------------------------------------------------------
						initialize
-------------------------------------------------------------*/
bool CYdLidar::initialize()
{
	bool ret = true;
    if (!checkCOMMs()) {
        fprintf(stderr,"[CYdLidar::initialize] Error initializing YDLIDAR scanner.\n");
        return false;
	}
    if (!checkStatus()) {
        fprintf(stderr,"[CYdLidar::initialize] Error initializing YDLIDAR scanner.because of failure in scan mode.\n");
    }
    if (!turnOn()) {
        fprintf(stderr,"[CYdLidar::initialize] Error initializing YDLIDAR scanner. Because the motor falied to start.\n");
		
	}
    return ret;
	
}
