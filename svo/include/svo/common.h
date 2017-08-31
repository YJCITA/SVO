#ifndef COMMON_H
#define COMMON_H

#include <iostream>
#include <cstdlib>
#include <list>
#include <eigen3/Eigen/Dense>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <fstream>
#include <string>
#include <cmath>
#include <iomanip>
#include <algorithm>

#include "gflags/gflags.h"
// 设置关于VLOG的打印等级
#define VLOG_FATAL 0
#define VLOG_ERROR 1
#define VLOG_WARNING 2
#define VLOG_INFO 3
#define VLOG_DEBUG 4

#if defined(USE_GLOG)
	#include "glog/logging.h"
#else
	#include <iostream>
	#define LOG(serverity) std::cout
	#define VLOG(serverity) std::cout
	#define LOG_IF(serverity, is_true) if(is_true) std::cout
#endif

// gflog
DECLARE_int32(start_frame_index); // 从哪一帧开始计算
// DECLARE_string(config_yaml_addr);


using namespace std;
using namespace cv;

// #define max(a, b) (((a) > (b)) ? (a) : (b))

// static string config_file = FLAGS_config_yaml_addr;
struct cam_{
	int width;
	int height;
	double cx;
	double cy;
	double fx;
	double fy;
	double k1;
	double k2;
	double k3;
	double p1;
	double p2;
	Eigen::Matrix3d rectify_rotation_matrix;
	Eigen::Matrix<double, 3, 4> projection_matrix;
	Eigen::Matrix3d r_cam_to_imu;
	Eigen::Vector3d p_cam_in_imu;
};
struct imuMeasurement_{
	Eigen::Vector3d am;
	Eigen::Vector3d wm;
	uint64_t timestamp;
};

struct image_{
	Mat image;
	uint64_t timestamp;
};

struct oxts_{
	uint64_t timestamp;
	double lat;
	double lon;
	double alt;
	double roll;
	double pitch;
	double yaw;
	Eigen::Vector3d am;
	Eigen::Vector3d wm;
	Eigen::Vector3d p_imu_in_home;
	Eigen::Vector3d vel_in_tangent_plane;
	Eigen::Quaterniond q_imu_to_home;
};

Eigen::Matrix3d crossMat(Eigen::Vector3d vec);

double sq(double x);

double distOfPoint2f(Point2d a, Point2d b);

double normOfPoint2f(Point2d a);

void printMtx(Eigen::MatrixXd m, string name);

Eigen::Vector3d quanternion2Eular(Eigen::Quaterniond q);

Eigen::Matrix4d omegaMtx(Eigen::Vector3d w);

class Log{
public:
	Log(string dir);
	~Log();
	ofstream file;
	
	void logMtx(string name,Eigen::MatrixXd mtx);
};


#endif // EIGEN_MATRIX_H

























































