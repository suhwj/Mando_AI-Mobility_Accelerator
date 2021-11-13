#ifndef NODE_LIDAR_H
#define NODE_LIDAR_H
#include <stdint.h>
#include <vector>
#include <string>
#include "locker.h"
#include <atomic>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;

#define PI 3.141592654
#define ULTRASONIC_ANGLE_INC_DEG 0.5

typedef struct
{
  float f_begin;
  float f_end;
} LidarCoverAngleStr;

typedef struct
{
  uint64_t timeStamp;
  float GyroYaw;
} GyroYawStr;

struct LaserConfig
{
  //! Start angle for the laser scan [rad].  0 is forward and angles are measured clockwise when viewing YDLIDAR from the top.
  float min_angle;
  //! Stop angle for the laser scan [rad].   0 is forward and angles are measured clockwise when viewing YDLIDAR from the top.
  float max_angle;
  //! angle resoltuion [rad]
  float angle_increment;
  //! Scan resoltuion [s]
  float time_increment;
  //! Time between scans
  float scan_time;
  //! Minimum range [m]
  float min_range;
  //! Maximum range [m]
  float max_range;
  LaserConfig &operator=(const LaserConfig &data)
  {
    min_angle = data.min_angle;
    max_angle = data.max_angle;
    angle_increment = data.angle_increment;
    time_increment = data.time_increment;
    scan_time = data.scan_time;
    min_range = data.min_range;
    max_range = data.max_range;
    return *this;
  }
};

struct LaserPoint
{
  //! lidar angle
  float angle;
  //! lidar range
  short range;
  //! lidar intensity
  unsigned char intensity;
  LaserPoint &operator=(const LaserPoint &data)
  {
    this->angle = data.angle;
    this->range = data.range;
    this->intensity = data.intensity;
    return *this;
  }
};

struct LaserScan
{
  //! System time when first range was measured in nanoseconds
  uint64_t stamp;
  //! Array of lidar points
  std::vector<LaserPoint> points;
  //! Configuration of scan
  LaserConfig config;
  LaserScan &operator=(const LaserScan &data)
  {
    this->points = data.points;
    this->stamp = data.stamp;
    this->config = data.config;
    return *this;
  }
};

struct node_info
{
  uint8_t sync_flag;          //sync flag
  uint16_t sync_quality;      //!信号质量
  uint16_t angle_q6_checkbit; //!测距点角度
  uint16_t distance_q2;       //! 当前测距点距离
  uint64_t stamp;             //! 时间戳
  uint8_t scan_frequence;     //! 特定版本此值才有效,无效值是0
  uint8_t debug_info[12];
  uint8_t index;
};

enum
{
  DEFAULT_TIMEOUT = 2000,    /**< 默认超时时间. */
  DEFAULT_HEART_BEAT = 1000, /**< 默认检测掉电功能时间. */
  MAX_SCAN_NODES = 3600,     /**< 最大扫描点数. */
  DEFAULT_TIMEOUT_COUNT = 1,
};

#define RESULT_OK 0
#define RESULT_TIMEOUT -1
#define RESULT_FAIL -2

#define IS_OK(x) ((x) == RESULT_OK)
#define IS_TIMEOUT(x) ((x) == RESULT_TIMEOUT)
#define IS_FAIL(x) ((x) == RESULT_FAIL)

typedef int32_t result_t;

struct PackageNode
{
  uint8_t PakageSampleQuality;
  uint16_t PakageSampleDistance;
} __attribute__((packed));

#define PackageSampleMaxLngth 0x100
typedef enum
{
  CT_Normal = 0,
  CT_RingStart = 1,
  CT_Tail,
} CT;
#define Node_Default_Quality (10)
#define Node_Sync 1
#define Node_NotSync 2
#define PackagePaidBytes 10
#define PH 0x55AA
#define NORMAL_PACKAGE_SIZE 90
#define INTENSITY_NORMAL_PACKAGE_SIZE 130

struct node_package
{
  uint16_t package_Head;
  uint8_t package_CT;
  uint8_t nowPackageNum;
  uint16_t packageFirstSampleAngle;
  uint16_t packageLastSampleAngle;
  uint16_t checkSum;
  PackageNode packageSample[PackageSampleMaxLngth];
} __attribute__((packed));

struct node_packages
{
  uint16_t package_Head;
  uint8_t package_CT;
  uint8_t nowPackageNum;
  uint16_t packageFirstSampleAngle;
  uint16_t packageLastSampleAngle;
  uint16_t checkSum;
  uint16_t packageSampleDistance[PackageSampleMaxLngth];
} __attribute__((packed));

//uint16_t package_Sample_Index;

#ifndef M_PI
#define M_PI 3.1415926
#endif

#define ANGLESTRLENMAX 32

#define SUNNOISEINTENSITY 0xff
#define GLASSNOISEINTENSITY 0xfe

#define LIDAR_CMD_STOP 0x65
#define LIDAR_CMD_SCAN 0x60
#define LIDAR_CMD_FORCE_SCAN 0x61
#define LIDAR_CMD_RESET 0x80
#define LIDAR_CMD_FORCE_STOP 0x00
#define LIDAR_CMD_GET_EAI 0x55
#define LIDAR_CMD_GET_DEVICE_INFO 0x90
#define LIDAR_CMD_GET_DEVICE_HEALTH 0x92
#define LIDAR_ANS_TYPE_DEVINFO 0x4
#define LIDAR_ANS_TYPE_DEVHEALTH 0x6
#define LIDAR_CMD_SYNC_BYTE 0xA5
#define LIDAR_CMDFLAG_HAS_PAYLOAD 0x80
#define LIDAR_ANS_SYNC_BYTE1 0xA5
#define LIDAR_ANS_SYNC_BYTE2 0x5A
#define LIDAR_ANS_TYPE_MEASUREMENT 0x81
#define LIDAR_RESP_MEASUREMENT_SYNCBIT (0x1 << 0)
#define LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT 2
#define LIDAR_RESP_MEASUREMENT_CHECKBIT (0x1 << 0)
#define LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT 1
#define LIDAR_RESP_MEASUREMENT_DISTANCE_SHIFT 2
#define LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT 8


#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#define UNUSED(x) (void)x

struct node_Gyro
{
  float angle;
  int count;
};

struct node_lidar_t
{
  node_lidar_t();
  ~node_lidar_t();

  node_info *scan_node_buf; 
  vector<node_Gyro> Gyro_nodes;
  uint8_t *globalRecvBuffer;
  uint16_t point_check;

  size_t recvNodeCount;
  size_t gyro_index;

  node_package package;   
  node_packages packages; 

  Event _dataEvent;     
  Locker _lock;	        
  Locker _serial_lock;  

  string port;          
  int m_SerialBaudrate; 

  GyroYawStr GyroYaw_Start;   
  GyroYawStr GyroYaw_During;  
  GyroYawStr GyroYaw_Present; 

  size_t scan_node_count;	   

  uint32_t m_PointTime;
  uint8_t last_device_byte;
  uint32_t trans_delay;
  uint16_t CheckSumCal;
  uint16_t SampleNumlAndCTCal;
  uint16_t LastSampleAngleCal;
  uint16_t Valu8Tou16;
  uint16_t package_Sample_Index; 
  uint16_t FirstSampleAngle;     
  uint16_t LastSampleAngle;      
  uint16_t CheckSum;		         
  uint8_t scan_frequence;	       
  uint64_t scan_time_t;          
  uint64_t last_node_time;

  int ROBOT_DIAMETER_mm;
  int LIDAR_ROBOT_CENTER_DISTANCE_mm;

  float motion_angle_start;   
  float motion_angle_end;     

  vector<LidarCoverAngleStr> LidarCoverAngle;

  bool m_Reversion; 
  bool m_Inverted;  
  bool m_intensities;        
  bool CheckSumResult;
  bool has_package_error;
  bool FilterEnable;        
  bool GyroCompensateEnable; 
  bool isConnected;         
  bool isScanning;           
  bool isAutoReconnect;     
  bool isAutoconnting;      

  float IntervalSampleAngle;
  float IntervalSampleAngle_LastPackage;
  
  
  int PackageSampleBytes; 
  int package_index;
  int retryCount;

  string frame_id;
  int baudrate;
  ros::Publisher laser_pub;
};

result_t grabScanData(node_info *nodebuffer, size_t &count, node_lidar_t &lidar_arg, uint32_t timeout = DEFAULT_TIMEOUT);
result_t checkAutoConnecting(); 
result_t startAutoScan(node_lidar_t &lidar_arg,bool force = false, uint32_t timeout = DEFAULT_TIMEOUT);
result_t connect(const char *port_path, uint32_t baudrate,node_lidar_t &lidar_arg);

void getLidarCoverAngle(char *charbuf,node_lidar_t &lidar_arg);
bool initialize(node_lidar_t &lidar_arg); 
int node_start(node_lidar_t &node_lidar);
int get_lidar_data(node_lidar_t &lidar_arg);

#endif
