#pragma once
#ifndef __LIDAR_DELAT_2A_H__
#define __LIDAR_DELAT_2A_H__
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#define PER_PACKAGE_NUM_POINT 0X49+2

#define angle_resolution 0.01f
#define dist_resolution 0.25f
#define NUMBER_OF_TEETH	16

int lidar_delat_get_data(void);
bool checksum_crc(unsigned char *buf,unsigned char frame_lens);

/////////////通信帧结构////////////////////////////
typedef struct _comm_frame_t
{
	uint8_t		frameStart;		//帧头
	uint16_t	frameLen;		//帧长度
	uint8_t		addr;			//地址码
	uint8_t		frameType;		//帧类型
	uint8_t		cmd;			//命令字
	uint16_t	paramLen;		//参数长度
	uint8_t		paramBuf[0];	//参数
} COMM_FRAME_T;

//激光雷达单个测量点的信息
typedef struct _rslidar_signal_distance_unit_t {
	uint8_t			signalValue;	//角度值
	float			speed;
	float			angleoffset;
	uint16_t		angle;			//角度
	uint16_t		distanceValue;	//距离值
}RSLIDAR_SIGNAL_DISTANCE_UNIT_T;

#endif