#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <thread>

#include "node_lidar.h"

#include "timer.h"
#include "serial_port.h"
#include "lidar_data_processing.h"

using namespace std;


float AngCorrect_fk = 19.16;
float AngCorrect_fx = 90.15;
float AngCorrect_fa = 12;


int LidarCoverBarNumber = 0;

short UltrasonicSim_ReduceDistances[360];

node_lidar_t::node_lidar_t()
{
	m_SerialBaudrate = 115200;
	m_PointTime = 1e9 / 3800;
	trans_delay = 0;
	m_intensities = false;
	scan_node_count = 0; //< 激光点数
	CheckSumCal = 0;
	SampleNumlAndCTCal = 0;
	LastSampleAngleCal = 0;
	CheckSumResult = true;
	Valu8Tou16 = 0;
	package_Sample_Index = 0; //< 包采样点索引
	IntervalSampleAngle = 0.0;
	IntervalSampleAngle_LastPackage = 0.0;
	FirstSampleAngle = 0; //< 起始采样角
	LastSampleAngle = 0;  //< 结束采样角
	CheckSum = 0;		  //< 校验和
	scan_frequence = 0;	  //< 协议中雷达转速
	has_package_error = false;
	PackageSampleBytes = 2; //< 一个包包含的激光点数
	package_index = 0;
	retryCount = 0; 
    last_device_byte=0x00;
	point_check = 0;

	GyroYaw_Present = {0};     //当前陀螺仪角度
	GyroYaw_Start   = {0};     //一帧开始处陀螺仪的角度
	GyroYaw_During  = {0};	   //一帧结束时陀螺仪的角度

	FilterEnable = false;		 //是否要做点云滤波
	GyroCompensateEnable = true; //是否要做旋转角度补偿
	last_node_time = getTime();

	scan_node_buf = new node_info[MAX_SCAN_NODES]; ///< 激光点信息
	globalRecvBuffer = new uint8_t[sizeof(node_packages)];

	isConnected = false;
	isScanning = false;

	isAutoReconnect = true;
	isAutoconnting = false;
}

node_lidar_t::~node_lidar_t()
{
	if (scan_node_buf)
	{
		delete[] scan_node_buf;
		scan_node_buf = NULL;
	}
	if (globalRecvBuffer)
	{
		delete[] globalRecvBuffer;
		globalRecvBuffer = NULL;
	}
	
	isAutoReconnect = false;
}

float AngCorrect(short Depth)
{
	float tempf = 0;

	tempf = atan(AngCorrect_fk * (Depth - AngCorrect_fx) / (AngCorrect_fx * Depth)) * 180 / PI - AngCorrect_fa;

	return tempf;
}

size_t read_serial1(std::string &buffer, size_t size)
{
	//ScopedReadLock lock(this->pimpl_);
	uint8_t *buffer_ = static_cast<uint8_t *>(alloca(size * sizeof(uint8_t)));
	size_t bytes_read = getData(buffer_, size);
	buffer.append(reinterpret_cast<const char *>(buffer_), bytes_read);
	return bytes_read;
}

string read_serial(size_t size)
{
	std::string buffer;
	read_serial1(buffer, size);
	return buffer;
}

void flushSerial(node_lidar_t &lidar_arg)
{

	node_lidar_t *node_lidar = &lidar_arg;
	if (!node_lidar->isConnected)
	{
		return;
	}

	size_t len = available();

	if (len)
	{
		read_serial(len);
	}

	delay(20);
}

result_t startAutoScan(node_lidar_t &lidar_arg,bool force, uint32_t timeout) {
  result_t ans;
  node_lidar_t *node_lidar = &lidar_arg;
  if (!node_lidar->isConnected) {
    return RESULT_FAIL;
  }

  flushSerial(lidar_arg);
  /*
  delay(10);
  {

    ScopedLocker l(node_lidar->_lock);
	
    if ((ans = sendCommand(force ? LIDAR_CMD_FORCE_SCAN : LIDAR_CMD_SCAN)) !=
        RESULT_OK) {
      return ans;
    }

    if (!m_SingleChannel) {
      lidar_ans_header response_header;

      if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
        return ans;
      }

      if (response_header.type != LIDAR_ANS_TYPE_MEASUREMENT) {
        return RESULT_FAIL;
      }

      if (response_header.size < 5) {
        return RESULT_FAIL;
      }
    }

  }

  if (isSupportMotorCtrl(model)) {
    startMotor();
  }*/

  return RESULT_OK;
}


result_t checkAutoConnecting(node_lidar_t &lidar_arg)
{
	node_lidar_t *node_lidar = &lidar_arg;
	result_t ans = RESULT_FAIL;
	node_lidar->isAutoconnting = true;

	while (node_lidar->isAutoReconnect && node_lidar->isAutoconnting)
	{
		{
			ScopedLocker l(node_lidar->_serial_lock);

		
			if (node_lidar->isConnected)
			{
				node_lidar->isConnected = false;
				close();
			}
		
		}
		node_lidar->retryCount++;

		if (node_lidar->retryCount > 100)
		{
			node_lidar->retryCount = 100;
		}

		delay(100 * node_lidar->retryCount);
		int retryConnect = 0;

		while(node_lidar->isAutoReconnect && connect(node_lidar->port.c_str(), node_lidar->m_SerialBaudrate,lidar_arg) != RESULT_OK)
		{
			retryConnect++;

			if (retryConnect > 25)
			{
				retryConnect = 25;
			}

			delay(200 * retryConnect);
		}

		if (!node_lidar->isAutoReconnect)
		{
			node_lidar->isScanning = false;
			return RESULT_FAIL;
		}

		if (node_lidar->isConnected)
		{
			delay(100);
			{
				ScopedLocker l(node_lidar->_serial_lock);
				ans = startAutoScan(lidar_arg);

				if (!IS_OK(ans)){
					ans = startAutoScan(lidar_arg);
				}
			}

			if (IS_OK(ans)){
				node_lidar->isAutoconnting = false;
				return ans;
			}
		}
	}
	return RESULT_FAIL;
}


/************************************************************************/
/*  激光数据解析线程　Laser data analysis thread                           */
/************************************************************************/
int read_forever(node_lidar_t &lidar_arg)
{
	node_lidar_t *node_lidar = &lidar_arg;
	
	node_info local_buf[128];
	node_info local_scan[500];
	size_t count = 128;
	
	size_t scan_count = 0;
	result_t ans = RESULT_FAIL;
	memset(local_scan, 0, sizeof(local_scan));

	int timeout_count = 0;
	flushSerial(lidar_arg);
	waitScanData(local_buf, count, lidar_arg);

	node_lidar->GyroYaw_Start.GyroYaw = node_lidar->GyroYaw_Present.GyroYaw;

	while (1)
	{
		if(true)
		{
			count = 128;
			ans = waitScanData(local_buf, count, lidar_arg);
			if (!IS_OK(ans))
			{
				if (IS_FAIL(ans) || timeout_count > DEFAULT_TIMEOUT_COUNT)
				{
					if (!node_lidar->isAutoReconnect)
					{
						fprintf(stderr, "exit scanning thread!!\n");
						fflush(stderr);
						{
							node_lidar->isScanning = false;
						}
						return RESULT_FAIL;
					}
					else
					{
						ans = checkAutoConnecting(lidar_arg);

						if (IS_OK(ans))
						{
							timeout_count = 0;
							local_scan[0].sync_flag = Node_NotSync;
						}
						else
						{
							node_lidar->isScanning = false;
							return RESULT_FAIL;
						}
					}
				}
				
				else
				{
					timeout_count++;
					local_scan[0].sync_flag = Node_NotSync;
					fflush(stderr);
				}
			}
			else
			{
				timeout_count = 0;
				node_lidar->retryCount = 0;
			}
			for (size_t pos = 0; pos < count; ++pos)
			{
				if (local_buf[pos].sync_flag & LIDAR_RESP_MEASUREMENT_SYNCBIT)
				{
					if ((local_scan[0].sync_flag & LIDAR_RESP_MEASUREMENT_SYNCBIT))
					{
						local_scan[0].stamp = local_buf[pos].stamp;
						local_scan[0].scan_frequence = local_buf[pos].scan_frequence;
						memcpy(node_lidar->scan_node_buf, local_scan, scan_count * sizeof(node_info));
						node_lidar->scan_node_count = scan_count;
						node_lidar->scan_time_t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
						get_lidar_data(lidar_arg);
						node_lidar->gyro_index=0;
						node_lidar->GyroYaw_Start.GyroYaw = node_lidar->GyroYaw_Present.GyroYaw;
					}
					scan_count = 0;
				}
				local_scan[scan_count++] = local_buf[pos];
				if (scan_count == _countof(local_scan))
				{
					scan_count -= 1;
				}
			}
		}
		else{
			flushSerial(lidar_arg);
			sleep(1);
		}
		
	}
	return RESULT_OK;
}


double from_degrees(double degrees)
{
	return degrees * M_PI / 180.0;
}

int get_lidar_data(node_lidar_t &lidar_arg)
{
	
	node_lidar_t *node_lidar = &lidar_arg;
	sensor_msgs::LaserScan scan_pub;
	scan_pub.ranges.resize(node_lidar->scan_node_count);
	scan_pub.intensities.resize(node_lidar->scan_node_count);
	printf("data number=%d\n",node_lidar->scan_node_count);
	size_t count = MAX_SCAN_NODES;
	
	uint16_t lidar_zero_count = 0;

	if(true)
	{
		uint64_t scan_time = node_lidar->m_PointTime * (count - 1);

		int all_node_count = count;

		scan_pub.angle_increment = (2.0*M_PI/node_lidar->scan_node_count);
		scan_pub.angle_min = 0;
		scan_pub.angle_max = 2*M_PI;
		scan_pub.range_min = 0.10;
		scan_pub.range_max = 10.0; //测量的最远距离是10m
		scan_pub.header.frame_id = node_lidar->frame_id;
		scan_pub.header.stamp = ros::Time::now();

		float range = 0.0;
		uint8_t intensity = 0;
		float angle = 0.0;
		count = node_lidar->scan_node_count;
	
		for (int i = 0; i < count; i++)
		{
			angle = static_cast<float>((node_lidar->scan_node_buf[i].angle_q6_checkbit >>
										LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) /
									   64.0f);
			range = static_cast<short>(node_lidar->scan_node_buf[i].distance_q2 / 4);
			intensity = static_cast<unsigned char>(node_lidar->scan_node_buf[i].sync_quality);
			
			LaserPoint point;
			
			if(0 <= angle && angle <= 360)
			{
				point.angle = angle;
				point.range = range;
				point.intensity = intensity;
			}
			else
			{
				point.angle = 0;
				point.range = 0;
				point.intensity = 0;
			}
			if(range < 100)
			{
				lidar_zero_count++;
			}
			else
			{
				node_lidar->point_check = 0;
			}
			scan_pub.ranges[i] = range / 1000.0f;
			scan_pub.intensities[i] = intensity;
		}
		node_lidar->laser_pub.publish(scan_pub);

	}
}

result_t connect(const char *port_path, uint32_t baudrate,node_lidar_t &lidar_arg)
{
	node_lidar_t *node_lidar = &lidar_arg;
	ScopedLocker lk(node_lidar->_serial_lock);
	{
		ScopedLocker l(node_lidar->_lock);

		if (!open_port(node_lidar->port,node_lidar->m_SerialBaudrate))
		{
			return RESULT_FAIL;
		}
		node_lidar->isConnected = true;
	}
	delay(100);
	setDTR(0); 
	return RESULT_OK;
}

bool initialize(node_lidar_t &lidar_arg)
{
	node_lidar_t *node_lidar = &lidar_arg;
	if (node_lidar->isConnected)
	{
		return true;
	}
	result_t op_result = connect(node_lidar->port.c_str(), node_lidar->m_SerialBaudrate,lidar_arg);
	if (!IS_OK(op_result)) {
    	return false;
  	}
	node_lidar->isScanning=true;
	return true;
}

int node_start(node_lidar_t &node_lidar)
{
	bool ret_init = initialize(node_lidar);	
	thread t1(read_forever, ref(node_lidar)); //read lidar data
	t1.detach();
	
	while(ros::ok()){
		sleep(1);
	}
	close();
	return 0;
}
