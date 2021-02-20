#include "lidar_delat_2a.h"
#include "win_serial.h"
#include "comm_config.h"
#include "stdlib.h"
#include <iostream>
#include <fstream>
#include <Windows.h>
#include "opencv_writer.h"

using namespace std;
Serial delat_serial;
bool retval = false;
	
static uint16_t find_head_offset(const uint8_t* buf, const uint16_t buf_len, const uint8_t* head_val, const uint16_t head_len) {
	uint16_t head_found_count = 0;
	uint16_t data_index = 0;
	for (; data_index < buf_len; ++data_index) {
		if (head_found_count >= head_len) {
			break;
		}
		if (buf[data_index] == head_val[head_found_count]) {
			++head_found_count;
		}
		else {
			head_found_count = 0;
		}
	}
	return data_index - head_found_count;
}

bool checksum_crc(unsigned char *buf,unsigned char frame_lens) {
	uint16_t chk32 = 0;
	for (int i = 0; i < frame_lens - 2; ++i) {
		chk32 += buf[i];
	}
	bool ret = (buf[frame_lens - 1] == (chk32 & 0xff)) && (buf[frame_lens - 2] = (chk32 / 256) & 0xff);
	if (!ret) {
		printf("check %x %x|%x %x\n",buf[frame_lens - 2], buf[frame_lens - 1], (chk32 / 256) & 0xff, (chk32 & 0xff));
		printf("checksum fail\r\n");
	}
	else {
		//printf("checksum successful!\r\n");
	}
	return ret;
}
int lidar_delat_get_data(void) {
	const uint32_t serial_baudrate = 115200;
	retval = delat_serial.open("COM7", 115200);
	if (retval) {
		printf("Serial com open successful.\n");
	}
	delat_serial.setDTR(true);
	delat_serial.flushInput();

	float laser_angle = 0.0;
	printf("laser_ydlidar_delat_2a angle=%f retval=%d\r\n", laser_angle, retval);

	int prev_angle = 0, angle_count = 0, raw_angle_count = 0;
	struct laser_ranges laser_ranges;
	memset(laser_ranges.ranges, 0, sizeof(laser_ranges.ranges));
	laser_ranges.raw_count = 0;
	uint8_t *buf = new uint8_t[520];
	if (buf == NULL)
	{
		printf("new buffer failed ,no space");
		return -1;
	}
	int valid_in_buf =0;
	int pre_v_angle = 0;
	//uint8_t simpleNum;
	//float scanFrequenceHz = 0.0;
	//uint16_t FirstSampleAngle, LastSampleAngle;
	//float IntervalSampleAngle = 0, IntervalSampleAngle_LastPackage = 0;
	constexpr uint8_t head_size = 6;
	constexpr uint8_t head[head_size] = { 0xaa, 0x00, 0x49, 0x01, 0x61, 0xad };
	uint16_t angleRange = 0;
	uint16_t angleStep = 0;
	uint16_t sampleNum;
	bool	flagOfNewNodes = false;
	size_t recvNodeCount = 0;
	int frame_byte_now;
	ofstream laser_file;
	laser_file.open("laser_data.txt",ios::out);
	if (!laser_file) {
		std::cout << "Open File Failed!" << std::endl;
	}
	while (retval) {
		valid_in_buf = 0;
		memset(buf, 0, sizeof(buf));

		while (valid_in_buf < 2)
		{
			valid_in_buf += delat_serial.read((uint8_t*)buf + valid_in_buf, 2 - valid_in_buf);
		}
		//printf("%2x %2x\r\n", buf[0], buf[1]);
		// now 4 bytes in buffer, check header
		int wasted = 0;
		while (buf[0] != 0xaa || buf[1] != 0x00 ) {
			buf[0] = buf[1];
			delat_serial.read(buf+1, 1);
			wasted++;
		}
		if (wasted > 0) {
			printf("wasted %d %d\n", wasted,valid_in_buf);
		}
		while (valid_in_buf <  6) {
			valid_in_buf += delat_serial.read((uint8_t*)buf + valid_in_buf, 6-valid_in_buf);
		}

		frame_byte_now = (int)(buf[1] << 8 | buf[2]);
		//printf("frame_byte_now:0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\r\n", frame_byte_now, buf[0], buf[1], buf[2]\
		, buf[3], buf[4], buf[5]);
		//printf("frame_byte:%0x\r\n",frame_byte_now);
		while (valid_in_buf < frame_byte_now+2) {
			valid_in_buf += delat_serial.read(buf + valid_in_buf, frame_byte_now+2 - valid_in_buf);
		}
		//check CRC-16	

		if (!checksum_crc(buf, frame_byte_now+2)) {
			std::cout << "Checksum failed!" << std::endl;
			continue;
		}
		uint16_t frameLen = buf[1] << 8 | buf[2];
		uint16_t param_lens = (buf[6] << 8) | buf[7];
		float scanFrequency = buf[8] * 0.05f;

		//cout << "Freq_hz:" << scanFrequency <<"hz"<< endl;

		sampleNum = (param_lens - 5) / 3;

		//std::cout << "frameLen/param/freq=(" <<frameLen<<" ,"<<param_lens<<" ,"<<scanFrequency<<" )"<< std::endl;
		float first_angle = ((buf[11] << 8) | buf[12])*0.01f;
		float last_angle = first_angle + 22.5*(sampleNum - 1) / sampleNum;
		float diff = last_angle - first_angle;
		//cout << "first_angle:" << first_angle <<" last_angle:"<<last_angle<<" diff:"<<diff<< endl;
		if (diff < 0) {
			diff += 360;
		}
		//diff /= (sampleNum - 1);

		for (int i = 0; i < sampleNum; ++i) {
			int angle;
			float fangle = (first_angle + diff*i)+ laser_angle;
			float dist = ((buf[14 + 3 * i] << 8) | buf[15 + 3 * i])*0.25f*.001f;
			//cout << "dist:" << dist << endl;
			

			while (fangle >= 360.)fangle -= 360.;
			while (fangle < 0.)fangle += 360.;
			angle = (int)fangle;
			if (angle < prev_angle&&angle_count>312) {
				//printf("scanFreq:%f angle:%d angle_cout:%d\r\n", scanFrequency,angle,angle_count);
				frame_debug(laser_ranges.ranges,"debug");
				angle_count = 0;
				memset(laser_ranges.ranges,0,sizeof(laser_ranges.ranges));
				laser_ranges.raw_count = 0;
			}
			laser_ranges.ranges[angle] = dist;
			//cout << "a=" << a <<" angle="<<angle<< " prev_angle=" << prev_angle << endl;
			cout << "fangle:" << fangle << " dist=" << dist << endl;
			if (laser_ranges.raw_count < MAX_LASER_POINTS) {
				laser_ranges.raw_angles[laser_ranges.raw_count] = fangle;
				laser_ranges.raw_dists[laser_ranges.raw_count] = dist;
				laser_ranges.raw_count++;
			}
			prev_angle = angle;
			angle_count++;

		}
		 
		//printf("FrameEnd...\r\n");
	}
	delat_serial.close();
	delete[]buf;
	return 0;
}