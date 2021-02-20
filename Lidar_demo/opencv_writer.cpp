#include <stdlib.h>
#include <string.h>
#include <cstdlib>
#include "opencv2/core/core.hpp"  
#include "opencv2/core/utility.hpp"  
#include "opencv2/imgproc/imgproc.hpp"  
#include "opencv2/imgcodecs.hpp"  
#include "opencv2/highgui/highgui.hpp"
#include <math.h>
#include "opencv_writer.h"

#define M_PI 3.141592653589793223846

using namespace cv;
//typedef struct CvPoint {
//	int x;
//	int y;
//}CvPoint;
//inline CvPoint cvPoint(int x, int y) {
//	CvPoint p;
//	p.x = x;
//	p.y = y;
//	return p;
//}
void frame_debug(const float r[360],const char* name) {
	int w = 800;
	int h = 800;
	float m = 8.;
	int half = w / 2;
	cv::Mat frame(h,w,CV_8UC1,cv::Scalar(128));
	for (int i = 0; i < 360; ++i) {
		float angle = -i + 90;
		float x = r[i] * cos(angle*M_PI / 180) / m;
		float y = r[i] * sin(angle*M_PI / 180) / m;
		cv::circle(frame, cv::Point(x*half + half, half - y*half), 1, cv::Scalar(255));
	}
	cv::circle(frame, cv::Point(half,half), 1, cv::Scalar(0));
	cv::imshow(name,frame);
	cv::waitKey(30);
}