#pragma once
#include <map>
#include <time.h>
#include <memory>
#include "common.h"

using namespace std;
class KittiParser{
public:
	KittiParser(string base_dir);
	
	~KittiParser();
	
	bool initialize();
	
	bool loadCalibration();
	
	bool loadTimestamp();
	
	bool readImage(uint64_t id, image_ &image_data);
	
	bool readOxts(uint64_t id, oxts_ &oxts);
	
	bool readOxts_yj(oxts_ &oxts);
	
	void SetStartOxtsTime(uint64_t time_ns);
	
	Eigen::Matrix3d getRVeloToCam();
	Eigen::Vector3d getPVeloInCam();
	Eigen::Matrix3d getRImuToVelo();
	Eigen::Vector3d getPImuInVelo();
	Eigen::Vector3d getP0InHome();
	Eigen::Vector3d getV0InHome();
	Eigen::Quaterniond getQ0ToHome();
	cam_ getCamCalib();
	
private:
	bool loadTimestampIntoVector(string filename, vector<uint64_t>* timestamp);
	
	string getFilenameForID(uint64_t id);
	
	bool parseVectorOfFloats(string &input,vector<double>* output);
	
	string m_base_dir;
	string m_calibration_dir;
	string m_image_00_data_dir;
	string m_image_00_timestamp_dir;
	string m_oxts_data_dir;
	string m_oxts_timestamp_dir;
	
	bool m_initial_home;
	double m_mercator_scale;
	double m_earth_radius;
	
	vector< uint64_t > m_timestamps_image00_ns;
	vector< uint64_t > m_timestamps_oxts_ns;
	
	cam_ m_cam_calib;
	Eigen::Matrix3d m_r_velo_to_cam;
	Eigen::Vector3d m_p_velo_in_cam;
	Eigen::Matrix3d m_r_imu_to_velo;
	Eigen::Vector3d m_p_imu_in_velo;
	Eigen::Vector3d m_p_home_in_global;
	Eigen::Quaterniond m_q_home_to_global;
	Eigen::Vector3d m_p0_in_home;
	Eigen::Vector3d m_v0_in_home;
	Eigen::Quaterniond m_q0_to_home;
	
	uint64_t m_id_oxts;
	uint64_t m_id_image;
	uint64 m_start_oxts_time;
	bool m_is_start_oxts_time_ok;
	
};