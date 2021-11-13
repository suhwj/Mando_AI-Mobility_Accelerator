#ifndef LIDAR_DATA_PROCESSING
#define LIDAR_DATA_PROCESSING

#include "node_lidar.h"
#include <stdint.h>

struct cmd_packet {
  uint8_t syncByte;
  uint8_t cmd_flag;
  uint8_t size;
  uint8_t data;
} __attribute__((packed)) ;

struct lidar_ans_header {
  uint8_t  syncByte1;
  uint8_t  syncByte2;
  uint32_t size: 30;
  uint32_t subType: 2;
  uint8_t  type;
} __attribute__((packed));


/************************************************************************/
/*  向激光雷达发布控制指令　Issue control command to lidar                  */
/************************************************************************/
result_t sendCommand(uint8_t cmd, node_lidar_t &lidar_arg,const void *payload = NULL,
                       size_t payloadsize = 0);

result_t sendData(const uint8_t *data, size_t size);

result_t waitResponseHeader(lidar_ans_header *header,node_lidar_t &lidar_arg,
                              uint32_t timeout = DEFAULT_TIMEOUT);

result_t waitResponseLidarHeader(uint8_t *header, uint32_t timeout = DEFAULT_TIMEOUT);

/*!
  * @brief 发送数据到雷达 \n
  * @param[in] nodebuffer 激光信息指针
  * @param[in] count      激光点数大小
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_TIMEOUT  等待超时
  * @retval RESULT_FAILE    失败
  */
result_t waitScanData(node_info *nodebuffer, size_t &count, 
                      node_lidar_t &lidar_arg, uint32_t timeout = DEFAULT_TIMEOUT);


/*!
* @brief 解包激光数据 \n
* @param[in] node 解包后激光点信息
* @param[in] timeout     超时时间
*/
result_t waitPackage(node_info *node, node_lidar_t &lidar_arg,uint32_t timeout = DEFAULT_TIMEOUT);










#endif