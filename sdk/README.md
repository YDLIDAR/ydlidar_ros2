YDLIDAR SDK PACKAGE V1.3.6
=====================================================================

SDK [test](https://github.com/yangfuyuan/sdk) application for YDLIDAR

Visit EAI Website for more details about [YDLIDAR](http://www.ydlidar.com/) .

How to build YDLIDAR SDK samples
=====================================================================

    $ git clone https://github.com/yangfuyuan/sdk

    $ cd sdk

    $ git checkout master

    $ cd ..

    $ mkdir build

    $ cd build

    $ cmake ../sdk

    $ make			###linux

    $ vs open Project.sln	###windows

How to run YDLIDAR SDK samples
=====================================================================

    $ cd samples

linux:

    $ ./ydlidar_test

    $ Please enter the lidar port:/dev/ttyUSB0

    $ Please enter the lidar baud rate:115200

    $ Please enter the lidar intensity:0


windows:

    $ ydlidar_test.exe

    $ Please enter the lidar port:COM3
    
    $ Please enter the lidar baud rate:115200

    $ Please enter the lidar intensity:0

=====================================================================

You should see YDLIDAR's scan result in the console:

     	YDLIDAR C++ TEST
	Please enter the lidar port:/dev/ttyUSB0
	Please enter the lidar baud rate:115200
	Please enter the lidar intensity:1
	fhs_lock: creating lockfile:      25148

	Yd Lidar running correctly ! The health status: good
	firmware: 273
	[YDLIDAR] Connection established in [/dev/ttyUSB0]:
	Firmware version: 1.1.7
	Hardware version: 1
	Model: S4
	Serial: 2018022700000003
	[YDLIDAR INFO] Current Sampling Rate : 4K
	[YDLIDAR INFO] Current Scan Frequency : 7.000000Hz
	set EXPOSURE MODEL SUCCESS!!!
	[YDLIDAR INFO] Now YDLIDAR is scanning ......
	min_angle: -3.141593 
	max_angle: 3.141593 
	Scan received: 571 ranges
	fit line size: 9 
	line length: 0.127150,   line angle: -1.888069
	line length: 0.149980,   line angle: -2.520781
	line length: 0.149141,   line angle: -2.590903
	line length: 0.186178,   line angle: -2.221969
	line length: 0.123318,   line angle: -2.354801
	line length: 0.086761,   line angle: 2.476112
	line length: 0.035698,   line angle: 2.552944
	line length: 0.108063,   line angle: 0.907348
	line length: 0.115837,   line angle: -1.229582



Lidar point data structure
=====================================================================

data structure:

    struct odom_info {

      uint64_t   stamp; ///< 时间戳

      double x;	      ///< x位置

      double y;	     ///< y位置

      double phi;     ///< 角度方向

      double v;       ///< 线速度

      double w;       ///< 角速度

      double dx;       ///< x位置增量

      double dy;       ///< y位置增量

      double dth;       ///< 方向增量

    };

    struct node_info {

       uint8_t    sync_flag;//new scan flag

       uint16_t    sync_quality;//!intensity

       uint16_t   angle_q6_checkbit; //!angle

       uint16_t   distance_q; //! distance

       uint64_t   stamp; //! time stamp

       uint8_t    scan_frequence;//! current_frequence = scan_frequence/10.0, If the current value equals zero, it is an invalid value

       odom_info   current_odom; //! current odometry sync pose

    } __attribute__((packed)) ;

example:

    if(data[i].scan_frequence != 0) {

        current_frequence = data[i].scan_frequence/10.0;
    }

    current_time_stamp = data[i].stamp;

    current_distance = data[i].distance_q;　//v1.3.5版本之后距离不用右移２位

    current_angle = ((data[i].angle_q6_checkbit>>LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);

    current_intensity = (float)(data[i].sync_quality);//v1.3.5版本之后信号质量不用右移２位

    ###note:current_frequence = data[0].scan_frequence/10.0.

    ###if the current_frequence value equals zero, it is an invalid value.

code:

        void ParseScan(node_info* data, const size_t& size) {

            double current_frequence, current_distance, current_angle, current_intensity;

            uint64_t current_time_stamp;

            for (size_t i = 0; i < size; i++ ) {

                if( data[i].scan_frequence != 0) {

                    current_frequence =  data[i].scan_frequence;//or current_frequence = data[0].scan_frequence

                }

                current_time_stamp = data[i].stamp;

                current_angle = ((data[i].angle_q6_checkbit>>LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);//LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT equals 8

                current_distance =  data[i].distance_q;

                current_intensity = (float)(data[i].sync_quality );

            }

            if (current_frequence != 0 ) {

                printf("current lidar scan frequency: %f\n", current_frequence);

            } else {

                printf("Current lidar does not support return scan frequency\n");

            }
        }





Upgrade Log
=====================================================================

2018-07-17 version:1.3.6

  1.add fit line.


2018-05-23 version:1.3.5

  1.add sync imu or odometry.

  2.update scan protocol.

2018-05-23 version:1.3.4

  1.add automatic reconnection if there is an exception

  2.add serial file lock.

2018-05-14 version:1.3.3

   1.add the heart function constraint.

   2.add packet type with scan frequency support.

2018-04-16 version:1.3.2

   1.add multithreading support.

2018-04-16 version:1.3.1

   1.Compensate for each laser point timestamp.
