#include "kitti_parser.h"
using namespace std;
KittiParser::KittiParser(string base_dir)
{
	m_base_dir = base_dir;
	m_calibration_dir = m_base_dir + "/calibration.yaml";
	m_image_00_data_dir = m_base_dir + "/image_00/data/";
	m_image_00_timestamp_dir = m_base_dir + "/image_00/timestamps.txt";
	m_oxts_data_dir = m_base_dir + "/oxts/data/";
	m_oxts_timestamp_dir = m_base_dir + "/oxts/timestamps.txt";
	
	m_initial_home = false;
	
	m_earth_radius = 6378137;
}
KittiParser::~KittiParser(){}

bool KittiParser::initialize()
{
	m_id_oxts = 0;
	m_id_image = 0;
	m_start_oxts_time = 0;
	m_is_start_oxts_time_ok = false;
	
	return (loadCalibration()&&loadTimestamp());
}

bool KittiParser::loadCalibration()
{
	FileStorage fs(m_calibration_dir, FileStorage::READ);
	if(!fs.isOpened()){
		cout<<string("Could not open file ") + m_calibration_dir<<endl;
		return false;
	}
	else{
		if(!fs["image_width"].isNone()){
			m_cam_calib.width = static_cast<int>(fs["image_width"]);
			printf("m_cam_calib.width: %d\n", m_cam_calib.width);
		}
		if(!fs["image_height"].isNone()){
			m_cam_calib.height = static_cast<int>(fs["image_height"]);
			printf("m_cam_calib.height: %d\n", m_cam_calib.height);
		}
		if(!fs["camera_intrinsic"].isNone()){
			FileNode n = fs["camera_intrinsic"];
			m_cam_calib.fx = static_cast<double>(n["fx"]);
			m_cam_calib.fy = static_cast<double>(n["fy"]);
			m_cam_calib.cx = static_cast<double>(n["cx"]);
			m_cam_calib.cy = static_cast<double>(n["cy"]);
			printf("fx: %5.4f, fy: %5.4f, cx: %5.4f, cy: %5.4f\n", 
					m_cam_calib.fx, 
					m_cam_calib.fy,
					m_cam_calib.cx,
					m_cam_calib.cy
					);
		}
		if(!fs["camera_distortion"].isNone()){
			FileNode n = fs["camera_distortion"];
			m_cam_calib.k1 = static_cast<double>(n["k1"]);
			m_cam_calib.k2 = static_cast<double>(n["k2"]);
			m_cam_calib.k3 = static_cast<double>(n["k3"]);
			m_cam_calib.p1 = static_cast<double>(n["p1"]);
			m_cam_calib.p2 = static_cast<double>(n["p2"]);
			printf("k1: %5.4f, k2: %5.4f, k3: %5.4f, p1: %5.4f, p2: %5.4f\n", 
					m_cam_calib.k1, 
					m_cam_calib.k2,
					m_cam_calib.k3,
					m_cam_calib.p1,
					m_cam_calib.p2
					);
		}
		if(!fs["projection_matrix"].isNone()){
			Mat cv_R;
			fs["projection_matrix"] >> cv_R;
			cout<<"projection_matrix: \n"<<cv_R<<endl;
			cv::cv2eigen(cv_R, m_cam_calib.projection_matrix);
		}
		
		if(!fs["velo_to_cam_r"].isNone()){
			Mat cv_R;
			fs["velo_to_cam_r"] >> cv_R;
			cout<<"velo_to_cam_r: \n"<<cv_R<<endl;
			cv::cv2eigen(cv_R, m_r_velo_to_cam);
		}
		if(!fs["velo_to_cam_t"].isNone()){
			Mat cv_t;
			fs["velo_to_cam_t"] >> cv_t;
			cout<<"velo_to_cam_t: \n"<<cv_t<<endl;
			cv::cv2eigen(cv_t, m_p_velo_in_cam);
		}
		if(!fs["imu_to_velo_r"].isNone()){
			Mat cv_R;
			fs["imu_to_velo_r"] >> cv_R;
			cout<<"imu_to_velo_r: \n"<<cv_R<<endl;
			cv::cv2eigen(cv_R, m_r_imu_to_velo);
		}
		if(!fs["imu_to_velo_t"].isNone()){
			Mat cv_t;
			fs["imu_to_velo_t"] >> cv_t;
			cout<<"imu_to_velo_t: \n"<<cv_t<<endl;
			cv::cv2eigen(cv_t, m_p_imu_in_velo);
		}
		m_cam_calib.r_cam_to_imu = (m_r_velo_to_cam*m_r_imu_to_velo).transpose();
		m_cam_calib.p_cam_in_imu = -m_cam_calib.r_cam_to_imu*(m_p_velo_in_cam + m_r_velo_to_cam*m_p_imu_in_velo);
		return true;
	}
}


bool KittiParser::loadTimestamp()
{
	return 
		(loadTimestampIntoVector(m_image_00_timestamp_dir, &m_timestamps_image00_ns) && 
		loadTimestampIntoVector(m_oxts_timestamp_dir, &m_timestamps_oxts_ns));
}

bool KittiParser::readImage(uint64_t id, image_ &image_data)
{
	if(m_timestamps_image00_ns.size() <= id){
		printf("Warning: no timestamp for this id!\n");
		return false;
	}
	image_data.timestamp = m_timestamps_image00_ns[id];
	string filename = m_image_00_data_dir + getFilenameForID(id) + ".png";
	image_data.image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
	if(!image_data.image.data){
		printf("Could not load image data! \n");
		return false;
	}
	return true;
}


void KittiParser::SetStartOxtsTime(uint64_t time_ns)
{
	m_start_oxts_time = time_ns;
	m_is_start_oxts_time_ok = true;
}

bool KittiParser::readOxts_yj(oxts_ &oxts)
{
	if(m_is_start_oxts_time_ok){
		uint64_t id = m_id_oxts++;
		if(m_timestamps_oxts_ns.size()<=id){
			printf("Warning: no timestamp for this id!\n");
			return false;
		}
		// 直到读取到指定的时间
		while(m_timestamps_oxts_ns[id] < m_start_oxts_time){
			id = m_id_oxts++;
		}
			
		string filename = m_oxts_data_dir + getFilenameForID(id) + ".txt";
		ifstream oxts_file(filename, ios::in);
		if(!oxts_file) return false;
		oxts.timestamp = m_timestamps_oxts_ns[id];
		
		string line;
		vector<double> parsed_float;
		while(getline(oxts_file, line)){
			if(parseVectorOfFloats(line, &parsed_float)){
				oxts.lat = parsed_float[0];
				oxts.lon = parsed_float[1];
				oxts.alt = parsed_float[2];
				oxts.roll = parsed_float[3];
				oxts.pitch = parsed_float[4];
				oxts.yaw = parsed_float[5];
				oxts.vel_in_tangent_plane << parsed_float[8], parsed_float[9], parsed_float[10];
				oxts.am << parsed_float[11], parsed_float[12], parsed_float[13];
				oxts.wm << parsed_float[17], parsed_float[18], parsed_float[19];
				
				Eigen::AngleAxisd axis_roll(oxts.roll, Eigen::Vector3d::UnitX());
				Eigen::AngleAxisd axis_pitch(oxts.pitch, Eigen::Vector3d::UnitY());
				Eigen::AngleAxisd axis_yaw(oxts.yaw, Eigen::Vector3d::UnitZ());
				Eigen::Quaterniond q = axis_yaw*axis_pitch*axis_roll;
				
				if(!m_initial_home){
					m_mercator_scale = cos(oxts.lat*M_PI/180.0);
				}
				double x, y, z;
				z = oxts.alt;
				x = m_mercator_scale*m_earth_radius*oxts.lon*M_PI/180.0;
				y = m_mercator_scale*m_earth_radius*log(tan((90.0+oxts.lat)*M_PI/360.0));
				Eigen::Vector3d p;
				p << x, y, z;
				if(!m_initial_home){
					m_p_home_in_global = p;
					m_q_home_to_global = axis_yaw;
					m_p0_in_home = Eigen::Vector3d::Zero();
					m_q0_to_home = (m_q_home_to_global.conjugate()*q).normalized();
					m_v0_in_home = oxts.vel_in_tangent_plane;
					m_initial_home = true;
				}
				
				oxts.q_imu_to_home = (m_q_home_to_global.conjugate()*q).normalized();
				oxts.p_imu_in_home = m_q_home_to_global.toRotationMatrix().transpose()*(p-m_p_home_in_global);
				return true;
			}
		}
	}
	return false;
}



bool KittiParser::readOxts(uint64_t id, oxts_ &oxts)
{
	if(m_timestamps_oxts_ns.size()<=id){
		printf("Warning: no timestamp for this id!\n");
		return false;
	}
	string filename = m_oxts_data_dir + getFilenameForID(id) + ".txt";
	ifstream oxts_file(filename, ios::in);
	if(!oxts_file) return false;
	oxts.timestamp = m_timestamps_oxts_ns[id];
	string line;
	vector<double> parsed_float;
	while(getline(oxts_file, line)){
		if(parseVectorOfFloats(line, &parsed_float)){
			oxts.lat = parsed_float[0];
			oxts.lon = parsed_float[1];
			oxts.alt = parsed_float[2];
			oxts.roll = parsed_float[3];
			oxts.pitch = parsed_float[4];
			oxts.yaw = parsed_float[5];
			oxts.vel_in_tangent_plane << parsed_float[8], parsed_float[9], parsed_float[10];
			oxts.am << parsed_float[11], parsed_float[12], parsed_float[13];
			oxts.wm << parsed_float[17], parsed_float[18], parsed_float[19];
			
			Eigen::AngleAxisd axis_roll(oxts.roll, Eigen::Vector3d::UnitX());
			Eigen::AngleAxisd axis_pitch(oxts.pitch, Eigen::Vector3d::UnitY());
			Eigen::AngleAxisd axis_yaw(oxts.yaw, Eigen::Vector3d::UnitZ());
			Eigen::Quaterniond q = axis_yaw*axis_pitch*axis_roll;
			
			if(!m_initial_home){
				m_mercator_scale = cos(oxts.lat*M_PI/180.0);
			}
			double x, y, z;
			z = oxts.alt;
			x = m_mercator_scale*m_earth_radius*oxts.lon*M_PI/180.0;
			y = m_mercator_scale*m_earth_radius*log(tan((90.0+oxts.lat)*M_PI/360.0));
			Eigen::Vector3d p;
			p << x, y, z;
			if(!m_initial_home){
				m_p_home_in_global = p;
				m_q_home_to_global = axis_yaw;
				m_p0_in_home = Eigen::Vector3d::Zero();
				m_q0_to_home = (m_q_home_to_global.conjugate()*q).normalized();
				m_v0_in_home = oxts.vel_in_tangent_plane;
				m_initial_home = true;
			}
			
			oxts.q_imu_to_home = (m_q_home_to_global.conjugate()*q).normalized();
			oxts.p_imu_in_home = m_q_home_to_global.toRotationMatrix().transpose()*(p-m_p_home_in_global);
			return true;
		}
	}
	return false;
}


bool KittiParser::loadTimestampIntoVector(string filename, vector< uint64_t >* v_timestamp)
{
	ifstream import_file(filename, std::ios::in);
	if(!import_file){
		cout<<"Could not open file: "<<filename<<endl;
		return false;
	}
	v_timestamp->clear();
	string line;
	while(getline(import_file, line)){
		stringstream line_stream(line);
		string timestamp_string = line_stream.str();
		std::tm t = {};
		t.tm_year = std::stoi(timestamp_string.substr(0, 4)) - 1900;
		t.tm_mon  = std::stoi(timestamp_string.substr(5, 2)) - 1;
		t.tm_mday = std::stoi(timestamp_string.substr(8, 2));
		t.tm_hour = std::stoi(timestamp_string.substr(11, 2));
		t.tm_min  = std::stoi(timestamp_string.substr(14, 2));
		t.tm_sec  = std::stoi(timestamp_string.substr(17, 2));
		t.tm_isdst = -1;
		time_t time_since_epoch = mktime(&t);
		uint64_t timestamp = time_since_epoch * 1e9 + std::stoi(timestamp_string.substr(20, 9));
		v_timestamp->push_back(timestamp);
	}
	return true;
}
string KittiParser::getFilenameForID(uint64_t id){
	char buffer[20];
	sprintf(buffer, "%010llu", id);
	return string(buffer);
}

bool KittiParser::parseVectorOfFloats(string &input,vector<double>* output)
{
	output->clear();
	std::stringstream line_stream(input);
	if (line_stream.eof()) {
		return false;
	}

	while (!line_stream.eof()) {
		std::string element;
		std::getline(line_stream, element, ' ');
		if (element.empty()) {
			continue;
		}
		try {
			output->emplace_back(std::stof(element));
		} 
		catch (const std::exception& exception) {
			std::cout << "Could not parse number in import file.\n";
			return false;
		}
	}
	return true;
}

cam_ KittiParser::getCamCalib(){return m_cam_calib;}

Eigen::Vector3d KittiParser::getPImuInVelo(){return m_p_imu_in_velo;}

Eigen::Matrix3d KittiParser::getRImuToVelo(){return m_r_imu_to_velo;}

Eigen::Vector3d KittiParser::getPVeloInCam(){return m_p_velo_in_cam;}

Eigen::Matrix3d KittiParser::getRVeloToCam(){return m_r_velo_to_cam;}

Eigen::Vector3d KittiParser::getP0InHome(){return m_p0_in_home;}

Eigen::Quaterniond KittiParser::getQ0ToHome(){return m_q0_to_home;}

Eigen::Vector3d KittiParser::getV0InHome(){return m_v0_in_home;}





