#include "win_serial.h"
#include "comm_config.h"
#include "stdlib.h"
#include <iostream>
#include <fstream>
#include <Windows.h>
#include "opencv_writer.h"
#include "laser_lidar_G4.h"
using namespace std;
Serial G4_serial;
bool serial_ret = false;
int lidar_get_EAI_G4_data(void) {
	const uint32_t serial_baudrate = 230400;
	serial_ret = G4_serial.open("COM7", 230400);
	if (serial_ret) {
		printf("Serial com open successful.\n");
	}
	G4_serial.setDTR(true);
	G4_serial.flushInput();

	float laser_angle = 0.0;
	printf("laser_ydlidar_delat_2a angle=%f retval=%d\r\n", laser_angle, serial_ret);

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
	int valid_in_buf = 0;
	int pre_v_angle = 0;
	constexpr uint8_t head_size = 6;
	constexpr uint8_t head[head_size] = { 0xaa, 0x00, 0x49, 0x01, 0x61, 0xad };
	uint16_t angleRange = 0;
	uint16_t angleStep = 0;
	uint16_t sampleNum;
	bool	flagOfNewNodes = false;
	size_t recvNodeCount = 0;
	int frame_byte_now;
	ofstream laser_file;
	laser_file.open("laser_data.txt", ios::out);
	if (!laser_file) {
		std::cout << "Open File Failed!" << std::endl;
	}
	while (serial_ret) {
		valid_in_buf = 0;
		memset(buf, 0, sizeof(buf));
		while (valid_in_buf < 2)
		{
			valid_in_buf += G4_serial.read((uint8_t*)buf + valid_in_buf, 2 - valid_in_buf);
		}
	}
	return 0;
}
