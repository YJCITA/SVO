#include "common.h"

using namespace std;
using namespace cv;

// DEFINE_int32(start_frame_index, 1, "start frame index");
// DEFINE_int32(start_frame_index, 3, ">= fcw_warn_counter, then warn");

Eigen::Matrix3d crossMat(Eigen::Vector3d vec)
{
	Eigen::Matrix3d ret;
	ret << 0, -vec[2], vec[1],
		vec[2], 0, -vec[0],
		-vec[1], vec[0], 0;
	return ret;
}

double sq(double x){
	return x*x;
}

// 两点的距离
double distOfPoint2f(Point2d a, Point2d b)
{
	double dx = a.x - b.x;
	double dy = a.y - b.y;
	return sqrt(dx*dx + dy*dy);
}

double normOfPoint2f(Point2d a){
	return sqrt(a.x * a.x + a.y * a.y);
}

void printMtx(Eigen::MatrixXd m, string name)
{
	printf("%s ", name.c_str());
	int row = (int)m.rows();
	int col = (int)m.cols();
	for(int i=0; i<row; i++){
		for(int j=0; j<col; j++){
			if(fabs(m(i, j))<1e-10) cout<<0;
			else cout<<setprecision(4)<<m(i, j);
			cout<<" ";
		}
		cout<<";"<<endl;
	}
}

Eigen::Vector3d quanternion2Eular(Eigen::Quaterniond q)
{
	double w = q.w();
	double x = q.x();
	double y = q.y();
	double z = q.z();
	double roll;
	double pitch;
	double yaw;
	roll = atan2(2*(w*x+y*z), 1-2*(sq(w)+sq(y)));
	pitch = asin(2*(w*y-x*z));
	yaw = atan2(2*(w*z+x*y), 1-2*(sq(y)+sq(z)));
	return Eigen::Vector3d(roll, pitch, yaw);
}

Eigen::Matrix4d omegaMtx(Eigen::Vector3d w)
{
	double wx;
	double wy;
	double wz;
	wx = w[0];
	wy = w[1];
	wz = w[2];
	Eigen::Matrix4d res;
	res << 0, -wx, -wy, -wz,
			wx, 0, wz, -wy,
			wy, -wz, 0, wx,
			wz, wy, -wx, 0;
	return res;
}


Log::Log(string dir)
{
	file.open(dir.c_str(), std::ios::out);
	cout<<"begin to log "<<dir<<" ..."<<endl;
}

Log::~Log(){}

void Log::logMtx(string name,Eigen::MatrixXd mtx)
{
	file<<name<<": "<<"["<<mtx.rows()<<","<<mtx.cols()<<"]"<<endl;
	int row;
	int col;
	for(row=0; row!=mtx.rows(); row++){
		for(col=0; col!=mtx.cols()-1; col++){
			file<<mtx(row, col)<<",";
		}
		file<<mtx(row, col)<<endl;
	}
	file<<endl;
}
