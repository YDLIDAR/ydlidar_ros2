YDLIDAR SDK PACKAGE V1.4.2
=====================================================================

SDK [test](https://github.com/ydlidar/sdk) application for YDLIDAR

Visit EAI Website for more details about [YDLIDAR](http://www.ydlidar.com/) .

How to build YDLIDAR SDK samples
=====================================================================
    $ git clone https://github.com/ydlidar/sdk
    $ cd sdk
    $ git checkout TG30
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
	$Lidar[ydlidar7] detected, whether to select current radar(yes/no)?:yes

windows:

	$ ydlidar_test.exe
	$Lidar[ydlidar7] detected, whether to select current radar(yes/no)?:yes

You should see YDLIDAR's scan result in the console:

	[YDLidar]: YDLidar running correctly ! The health status is good
	[YDLIDAR] Connection established in [/dev/ttyUSB0]:
	Firmware version: 1.0.5
	Hardware version: 2
	Model: TG30
	Serial: 2018112000000017
	[YDLidar]: [YDLIDAR INFO] Current Sampling Rate : 20K
	[YDLidar]: [YDLIDAR INFO] Current Scan Frequency : 8.000000Hz
	[YDLidar]: [YDLIDAR INFO] Now YDLIDAR is scanning ......

	[YDLidar]: Scan received[1545813311357484110]: 2087 ranges is [8.629041]Hz
	[YDLidar]: Scan received[1545813311481785110]: 2309 ranges is [7.799038]Hz
	[YDLidar]: Scan received[1545813311611886110]: 2348 ranges is [7.669442]Hz
	[YDLidar]: Scan received[1545813311744358110]: 2380 ranges is [7.566280]Hz
	[YDLidar]: Scan received[1545813311879407110]: 2406 ranges is [7.484483]Hz
	[YDLidar]: Scan received[1545813312016237110]: 2432 ranges is [7.404435]Hz
	[YDLidar]: Scan received[1545813312153357110]: 2453 ranges is [7.341020]Hz
	[YDLidar]: Scan received[1545813312292580110]: 2505 ranges is [7.188571]Hz
	[YDLidar]: Scan received[1545813312434769110]: 2527 ranges is [7.125962]Hz




Lidar point data structure
=====================================================================

data structure:

	struct node_info {
    	uint8_t    sync_flag;
    	uint16_t   sync_quality;//!信号质量
    	uint16_t   angle_q6_checkbit; //!测距点角度
    	uint16_t   distance_q2; //! 当前测距点距离
    	uint64_t   stamp; //! 时间戳
    	uint8_t    scan_frequence;//! 特定版本此值才有效,无效值是0, 当前扫描频率current_frequence = scan_frequence/10.0
	} __attribute__((packed)) ;

example:

	if(data[i].scan_frequence != 0) {
 		current_frequence = data[i].scan_frequence/10.0;
	}

	current_time_stamp = data[i].stamp;


	current_distance = (float)data[i].distance_q2/2000.f;


	current_angle = ((data[i].angle_q6_checkbit>>LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);

	current_intensity = (float)(data[i].sync_quality >> LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);

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

				
		current_distance = (float)data[i].distance_q2/2000.f;

                current_angle = ((data[i].angle_q6_checkbit>>LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);//LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT equals 8

                current_intensity = (float)(data[i].sync_quality >> LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);

            }

            if (current_frequence != 0 ) {

                printf("current lidar scan frequency: %f\n", current_frequence);

            } else {

                printf("Current lidar does not support return scan frequency\n");

            }
        }





Upgrade Log
=====================================================================

2019-11-27 version:1.4.2

  1.define USE_LOCK_FILE macros 

  2.use steady time for wait.

2019-05-10 version:1.4.1

  1.optimize timestamp.

2018-12-3 version:1.3.9

1.Optimized interface.

2.Getting Radar Port Lists.

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

