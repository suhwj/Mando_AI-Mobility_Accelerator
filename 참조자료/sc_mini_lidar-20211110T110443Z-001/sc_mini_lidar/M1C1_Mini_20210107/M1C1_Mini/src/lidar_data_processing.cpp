#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <thread>
#include "timer.h"
#include "serial_port.h"
#include "lidar_data_processing.h"

bool has_device_header = false;


/************************************************************************/
/*  向激光雷达发布控制指令　Issue control command to lidar                  */
/************************************************************************/
result_t sendCommand(uint8_t cmd, node_lidar_t &lidar_arg, const void *payload,
                     size_t payloadsize)
{
  node_lidar_t *node_lidar = &lidar_arg;
  uint8_t pkt_header[10];
  cmd_packet *header = reinterpret_cast<cmd_packet *>(pkt_header);
  uint8_t checksum = 0;

  if (!node_lidar->isConnected)
  {
    return RESULT_FAIL;
  }

  if (payloadsize && payload)
  {
    cmd |= LIDAR_CMDFLAG_HAS_PAYLOAD;
  }

  header->syncByte = LIDAR_CMD_SYNC_BYTE;
  header->cmd_flag = cmd;
  sendData(pkt_header, 2);

  if ((cmd & LIDAR_CMDFLAG_HAS_PAYLOAD) && payloadsize && payload)
  {
    checksum ^= LIDAR_CMD_SYNC_BYTE;
    checksum ^= cmd;
    checksum ^= (payloadsize & 0xFF);

    for (size_t pos = 0; pos < payloadsize; ++pos)
    {
      checksum ^= ((uint8_t *)payload)[pos];
    }

    uint8_t sizebyte = (uint8_t)(payloadsize);
    sendData(&sizebyte, 1);

    sendData((const uint8_t *)payload, sizebyte);

    sendData(&checksum, 1);
  }

  return RESULT_OK;
}

result_t sendData(const uint8_t *data, size_t size)
{

  /*
  if (!isConnected) {
    return RESULT_FAIL;
  }*/

  if (data == NULL || size == 0)
  {
    return RESULT_FAIL;
  }

  size_t r;

  while (size)
  {
    r = write_data(data, size);

    if (r < 1)
    {
      return RESULT_FAIL;
    }

    size -= r;
    data += r;
  }

  return RESULT_OK;
}

result_t waitResponseHeader(lidar_ans_header *header, node_lidar_t &lidar_arg,
                            uint32_t timeout)
{
  node_lidar_t *node_lidar = &lidar_arg;
  int recvPos = 0;
  uint32_t startTs = getms();
  uint8_t recvBuffer[sizeof(lidar_ans_header)];
  uint8_t *headerBuffer = reinterpret_cast<uint8_t *>(header);
  uint32_t waitTime = 0;
  has_device_header = false;

  node_lidar->last_device_byte = 0x00;

  while ((waitTime = getms() - startTs) <= timeout)
  {
    size_t remainSize = sizeof(lidar_ans_header) - recvPos;
    size_t recvSize = 0;
    result_t ans = waitForData(remainSize, timeout - waitTime, &recvSize);

    if (!IS_OK(ans))
    {
      return ans;
    }

    if (recvSize > remainSize)
    {
      recvSize = remainSize;
    }

    ans = getData(recvBuffer, recvSize);

    if (IS_FAIL(ans))
    {
      return RESULT_FAIL;
    }

    for (size_t pos = 0; pos < recvSize; ++pos)
    {
      uint8_t currentByte = recvBuffer[pos];

      switch (recvPos)
      {
      case 0:
        if (currentByte != LIDAR_ANS_SYNC_BYTE1)
        {
          if (node_lidar->last_device_byte == (PH & 0xFF) && currentByte == (PH >> 8))
          {
            has_device_header = true;
          }

          node_lidar->last_device_byte = currentByte;
          continue;
        }

        break;

      case 1:
        if (currentByte != LIDAR_ANS_SYNC_BYTE2)
        {
          node_lidar->last_device_byte = currentByte;
          recvPos = 0;
          continue;
        }

        break;
      }

      headerBuffer[recvPos++] = currentByte;
      node_lidar->last_device_byte = currentByte;

      if (recvPos == sizeof(lidar_ans_header))
      {
        return RESULT_OK;
      }
    }
  }

  return RESULT_FAIL;
}


result_t waitPackage(node_info *node, node_lidar_t &lidar_arg, uint32_t timeout)
{
  node_lidar_t *node_lidar = &lidar_arg;
  
  int recvPos = 0;
  uint32_t startTs = getms();
  uint32_t waitTime = 0;
  uint8_t *packageBuffer = (node_lidar->m_intensities) ? (uint8_t *)&node_lidar->package.package_Head : (uint8_t *)&node_lidar->packages.package_Head;
  uint8_t package_Sample_Num = 0;
  int32_t AngleCorrectForDistance = 0;
  int package_recvPos = 0;
  uint8_t package_type = 0;

  if (node_lidar->package_Sample_Index == 0)
  {
    recvPos = 0;

    while ((waitTime = getms() - startTs) <= timeout)
    {
      size_t remainSize = PackagePaidBytes - recvPos;
      size_t recvSize = 0;
      result_t ans = waitForData(remainSize, timeout - waitTime, &recvSize);

      if (!IS_OK(ans))
      {
        return ans;
      }

      if (recvSize > remainSize)
      {
        recvSize = remainSize;
      }

      getData(node_lidar->globalRecvBuffer, recvSize);

      for (size_t pos = 0; pos < recvSize; ++pos)
      {
        uint8_t currentByte = node_lidar->globalRecvBuffer[pos];
        switch (recvPos)
        {
        case 0:
          if (currentByte == (PH & 0xFF))
          {
          }
          else
          {
            continue;
          }
          break;
        case 1:
          node_lidar->CheckSumCal = PH;
          if (currentByte == (PH >> 8))
          {
          }
          else
          {
            recvPos = 0;
            continue;
          }
          break;
        case 2:
          node_lidar->SampleNumlAndCTCal = currentByte;
          package_type = currentByte & 0x01;

          if ((package_type == CT_Normal) || (package_type == CT_RingStart))
          {
            if (package_type == CT_RingStart)
            {
              node_lidar->scan_frequence = (currentByte & 0xFE) >> 1;
            }
          }
          else
          {
            node_lidar->has_package_error = true;
            recvPos = 0;
            continue;
          }

          break;

        case 3:
          node_lidar->SampleNumlAndCTCal += (currentByte * 0x100);
          package_Sample_Num = currentByte;
          break;

        case 4:
          if (currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT)
          {
            node_lidar->FirstSampleAngle = currentByte;
          }
          else
          {
            node_lidar->has_package_error = true;
            recvPos = 0;
            continue;
          }

          break;

        case 5:
          node_lidar->FirstSampleAngle += currentByte * 0x100;
          node_lidar->CheckSumCal ^= node_lidar->FirstSampleAngle;
          node_lidar->FirstSampleAngle = node_lidar->FirstSampleAngle >> 1;
          break;

        case 6:
          if (currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT)
          {
            node_lidar->LastSampleAngle = currentByte;
          }
          else
          {
            node_lidar->has_package_error = true;
            recvPos = 0;
            continue;
          }
          break;
        case 7:
          node_lidar->LastSampleAngle = currentByte * 0x100 + node_lidar->LastSampleAngle;
          node_lidar->LastSampleAngleCal = node_lidar->LastSampleAngle;
          node_lidar->LastSampleAngle = node_lidar->LastSampleAngle >> 1;

          if (package_Sample_Num == 1)
          {
            node_lidar->IntervalSampleAngle = 0;
          }
          else
          {
            if (node_lidar->LastSampleAngle < node_lidar->FirstSampleAngle)
            {
              if ((node_lidar->FirstSampleAngle > 270 * 64) && (node_lidar->LastSampleAngle < 90 * 64))
              {
                node_lidar->IntervalSampleAngle = (float)((360 * 64 + node_lidar->LastSampleAngle -
                                                           node_lidar->FirstSampleAngle) /
                                                          ((
                                                               package_Sample_Num - 1) *
                                                           1.0));
                node_lidar->IntervalSampleAngle_LastPackage = node_lidar->IntervalSampleAngle;
              }
              else
              {
                node_lidar->IntervalSampleAngle = node_lidar->IntervalSampleAngle_LastPackage;
              }
            }
            else
            {
              node_lidar->IntervalSampleAngle = (float)((node_lidar->LastSampleAngle - node_lidar->FirstSampleAngle) / ((
                                                                                                                            package_Sample_Num - 1) *
                                                                                                                        1.0));
              node_lidar->IntervalSampleAngle_LastPackage = node_lidar->IntervalSampleAngle;
            }
          }

          break;

        case 8:
          node_lidar->CheckSum = currentByte;
          break;

        case 9:
          node_lidar->CheckSum += (currentByte * 0x100);
          break;
        }

        packageBuffer[recvPos++] = currentByte;
      }

      if (recvPos == PackagePaidBytes)
      {
        package_recvPos = recvPos;
        break;
      }
    }

    if (PackagePaidBytes == recvPos)
    {
      startTs = getms();
      recvPos = 0;

      while ((waitTime = getms() - startTs) <= timeout)
      {
        size_t remainSize = package_Sample_Num * node_lidar->PackageSampleBytes - recvPos;
        size_t recvSize = 0;
        result_t ans = waitForData(remainSize, timeout - waitTime, &recvSize);

        if (!IS_OK(ans))
        {
          return ans;
        }

        if (recvSize > remainSize)
        {
          recvSize = remainSize;
        }

        getData(node_lidar->globalRecvBuffer, recvSize);

        for (size_t pos = 0; pos < recvSize; ++pos)
        {
          if (node_lidar->m_intensities)
          {
            if (recvPos % 3 == 2)
            {
              node_lidar->Valu8Tou16 += node_lidar->globalRecvBuffer[pos] * 0x100;
              node_lidar->CheckSumCal ^= node_lidar->Valu8Tou16;
            }
            else if (recvPos % 3 == 1)
            {
              node_lidar->Valu8Tou16 = node_lidar->globalRecvBuffer[pos];
            }
            else
            {
              node_lidar->CheckSumCal ^= node_lidar->globalRecvBuffer[pos];
            }
          }
          else
          {
            if (recvPos % 2 == 1)
            {
              node_lidar->Valu8Tou16 += node_lidar->globalRecvBuffer[pos] * 0x100;
              node_lidar->CheckSumCal ^= node_lidar->Valu8Tou16;
            }
            else
            {
              node_lidar->Valu8Tou16 = node_lidar->globalRecvBuffer[pos];
            }
          }

          packageBuffer[package_recvPos + recvPos] = node_lidar->globalRecvBuffer[pos];
          recvPos++;
        }

        if (package_Sample_Num * node_lidar->PackageSampleBytes == recvPos)
        {
          package_recvPos += recvPos;
          break;
        }
      }

      if (package_Sample_Num * node_lidar->PackageSampleBytes != recvPos)
      {
        return RESULT_FAIL;
      }
    }
    else
    {
      return RESULT_FAIL;
    }

    node_lidar->CheckSumCal ^= node_lidar->SampleNumlAndCTCal;
    node_lidar->CheckSumCal ^= node_lidar->LastSampleAngleCal;

    if (node_lidar->CheckSumCal != node_lidar->CheckSum)
    {
      node_lidar->CheckSumResult = false;
      node_lidar->has_package_error = true;
    }
    else
    {
      node_lidar->CheckSumResult = true;
    }
  }
  uint8_t package_CT;

  if (node_lidar->m_intensities)
  {
    package_CT = node_lidar->package.package_CT;
  }
  else
  {
    package_CT = node_lidar->packages.package_CT;
  }

  (*node).scan_frequence = 0;

  if ((package_CT & 0x01) == CT_Normal)
  {
    (*node).sync_flag = Node_NotSync;
    memset((*node).debug_info, 0xff, sizeof((*node).debug_info));

    if (!node_lidar->has_package_error)
    {
      if (node_lidar->package_index < 10)
      {
        (*node).debug_info[node_lidar->package_index] = (package_CT >> 1);
        (*node).index = node_lidar->package_index;
      }
      else
      {
        (*node).index = 0xff;
      }
      if (node_lidar->package_Sample_Index == 0)
      {
        node_lidar->package_index++;
      }
    }
    else
    {
      (*node).index = 255;
      node_lidar->package_index = 0;
    }
  }
  else
  {
    (*node).sync_flag = Node_Sync;
    (*node).index = 255;
    node_lidar->package_index = 0;

    if (node_lidar->CheckSumResult)
    {
      node_lidar->has_package_error = false;
      (*node).scan_frequence = node_lidar->scan_frequence;
    }
  }

  (*node).sync_quality = Node_Default_Quality;
  (*node).stamp = 0;

  if (node_lidar->CheckSumResult)
  {
    if (node_lidar->m_intensities)
    {
      (*node).sync_quality = ((uint16_t)((node_lidar->package.packageSample[node_lidar->package_Sample_Index].PakageSampleDistance & 0x03)
                                         << LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT) |
                              (node_lidar->package.packageSample[node_lidar->package_Sample_Index].PakageSampleQuality));
      (*node).distance_q2 =
          node_lidar->package.packageSample[node_lidar->package_Sample_Index].PakageSampleDistance & 0xfffc;
    }
    else
    {
      (*node).distance_q2 = node_lidar->packages.packageSampleDistance[node_lidar->package_Sample_Index];
      /*(*node).sync_quality = ((uint16_t)(0xfc |node_lidar->packages.packageSampleDistance[node_lidar->package_Sample_Index]
							 &0x0003))<< LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;*/
      (*node).sync_quality = ((uint16_t)((node_lidar->packages.packageSampleDistance[node_lidar->package_Sample_Index]) & 0x03));
    }

    if ((*node).distance_q2 != 0)
    {
      /*
      AngleCorrectForDistance = (int32_t)(((atan(((21.8 * (155.3 - ((*node).distance_q2 / 4.0))) / 155.3) /
                                                 ((*node).distance_q2 / 4.0))) *
                                           180.0 / 3.1415) *
                                      64.0);*/

      AngleCorrectForDistance = 0;
      
    }
    else
    {
      AngleCorrectForDistance = 0;
    }

    float sampleAngle = node_lidar->IntervalSampleAngle * node_lidar->package_Sample_Index;

    if ((node_lidar->FirstSampleAngle + sampleAngle +
         AngleCorrectForDistance) < 0)
    {
      (*node).angle_q6_checkbit = (((uint16_t)(node_lidar->FirstSampleAngle + sampleAngle +
                                               AngleCorrectForDistance + 23040))
                                   << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) +
                                  LIDAR_RESP_MEASUREMENT_CHECKBIT;
    }
    else
    {
      if ((node_lidar->FirstSampleAngle + sampleAngle + AngleCorrectForDistance) > 23040)
      {
        (*node).angle_q6_checkbit = (((uint16_t)(node_lidar->FirstSampleAngle + sampleAngle +
                                                 AngleCorrectForDistance - 23040))
                                     << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) +
                                    LIDAR_RESP_MEASUREMENT_CHECKBIT;
      }
      else
      {
        (*node).angle_q6_checkbit = (((uint16_t)(node_lidar->FirstSampleAngle + sampleAngle +
                                                 AngleCorrectForDistance))
                                     << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) +
                                    LIDAR_RESP_MEASUREMENT_CHECKBIT;
      }
    }
  }
  else
  {
    (*node).sync_flag = Node_NotSync;
    (*node).sync_quality = Node_Default_Quality;
    (*node).angle_q6_checkbit = LIDAR_RESP_MEASUREMENT_CHECKBIT;
    (*node).distance_q2 = 0;
    (*node).scan_frequence = 0;
  }

  uint8_t nowPackageNum;

  if (node_lidar->m_intensities)
  {
    nowPackageNum = node_lidar->package.nowPackageNum;
  }
  else
  {
    nowPackageNum = node_lidar->packages.nowPackageNum;
  }

  node_lidar->package_Sample_Index++;

  if (node_lidar->package_Sample_Index >= nowPackageNum)
  {
    node_lidar->package_Sample_Index = 0;
    node_lidar->CheckSumResult = false;
  }
  return RESULT_OK;
}

result_t waitScanData(node_info *nodebuffer, size_t &count, node_lidar_t &lidar_arg, uint32_t timeout)
{
  node_lidar_t *node_lidar = &lidar_arg;
  node_lidar->recvNodeCount = 0;
  uint32_t startTs = getms();
  uint32_t waitTime = 0;
  result_t ans = RESULT_FAIL;
  /*超时处理及点数判断*/
  while ((waitTime = getms() - startTs) <= timeout && node_lidar->recvNodeCount < count)
  {
    node_info node;
    /*解析出一个点的数据*/
    ans = waitPackage(&node, lidar_arg, timeout - waitTime);

    if (!IS_OK(ans))
    {
      count = node_lidar->recvNodeCount;
      return ans;
    }

    nodebuffer[node_lidar->recvNodeCount++] = node;
    node_lidar->gyro_index++;

    if (node.sync_flag & LIDAR_RESP_MEASUREMENT_SYNCBIT)
    {
      /*检查缓冲区中的字节数*/
      size_t size = available();
      uint64_t delayTime = 0;
      /*数据包的数据量*/
      size_t PackageSize = (node_lidar->m_intensities ? INTENSITY_NORMAL_PACKAGE_SIZE : NORMAL_PACKAGE_SIZE);

      if (size > PackagePaidBytes && size < PackagePaidBytes * PackageSize)
      {
        size_t packageNum = size / PackageSize;
        size_t Number = size % PackageSize;
        delayTime = packageNum * node_lidar->m_PointTime * PackageSize / 2;

        if (Number > PackagePaidBytes)
        {
          delayTime += node_lidar->m_PointTime * ((Number - PackagePaidBytes) / 2);
        }

        size = Number;

        if (packageNum > 0 && Number == 0)
        {
          size = PackageSize;
        }
      }

      nodebuffer[node_lidar->recvNodeCount - 1].stamp = size * node_lidar->trans_delay + delayTime;
      nodebuffer[node_lidar->recvNodeCount - 1].scan_frequence = node.scan_frequence;
      count = node_lidar->recvNodeCount;
      return RESULT_OK;
    }

    if (node_lidar->recvNodeCount == count)
    {
      return RESULT_OK;
    }
  }
  count = node_lidar->recvNodeCount;
  return RESULT_FAIL;
}

