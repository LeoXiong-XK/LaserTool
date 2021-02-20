#ifndef __COMM_CONFIG_H__
#define __COMM_CONFIG_H__
#pragma once
#include <stdint.h>
#define MAX_LASER_POINTS 360
struct laser_ranges {
	float ranges[360];
	int raw_count;
	int angle[MAX_LASER_POINTS];
	float raw_angles[MAX_LASER_POINTS];
	float raw_dists[MAX_LASER_POINTS];
};
#endif // !__COMM_CONFIG_H__


