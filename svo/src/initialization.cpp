// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <svo/config.h>
#include <svo/frame.h>
#include <svo/point.h>
#include <svo/feature.h>
#include <svo/initialization.h>
#include <svo/feature_detection.h>
#include <vikit/math_utils.h>
#include <vikit/homography.h>
#include <vikit/fundamental.h>

namespace svo {
namespace initialization {

InitResult KltHomographyInit::addFirstFrame(FramePtr frame_ref)
{
	reset();
	detectFeatures(frame_ref, px_ref_, f_ref_);
	VLOG(5)<<"KltHomographyInit::addFirstFrame--"<<"first img feature num: "<<px_ref_.size()<<std::endl;
	
	// 确保第一帧的图像中检测到的特征数大于100个
	if(px_ref_.size() < 100){
		SVO_WARN_STREAM_THROTTLE(2.0, "First image has less than 100 features. Retry in more textured environment.");
		return FAILURE;
	}
	frame_ref_ = frame_ref;
	// 先设置当前帧的特征与参考帧的特征一致
	px_cur_.insert(px_cur_.begin(), px_ref_.begin(), px_ref_.end());
	return SUCCESS;
}


InitResult KltHomographyInit::addSecondFrame(FramePtr frame_cur)
{
	trackKlt(frame_ref_, frame_cur, px_ref_, px_cur_, f_ref_, f_cur_, disparities_);
	VLOG(5)<<"Init: KLT tracked "<< disparities_.size() <<" features";

	// 符合光流跟踪的特征数 50
	if(disparities_.size() < Config::initMinTracked())
		return FAILURE;

	// 对两帧光流跟踪之后像素差值的中值
	double disparity = vk::getMedian(disparities_);
	VLOG(5)<<"Init: KLT= "<<disparity<<"px average disparity.";
	
	//  如果中值小于给定配置参数，则表明这一帧不是关键帧，也就是刚开始的时候两帧不能太近
	if(disparity < Config::initMinDisparity())  // 默认是50
		return NO_KEYFRAME;

	if(0){
		// computeHomography  计算单应矩阵
		computeHomography( f_ref_, f_cur_, frame_ref_->cam_->errorMultiplier2(), Config::poseOptimThresh(), 
							inliers_, xyz_in_cur_, T_cur_from_ref_ );
		VLOG(5)<<"Init: Homography RANSAC= "<<inliers_.size()<<" inliers.";
	}
	
	if(1){
		// -YJ-
		vector<int> inliers;
		vector<Vector3d> xyz_in_cur;  
		SE3 T_cur_from_ref;
		computeFundamental( frame_ref_->img_pyr_[0], frame_cur->img_pyr_[0], f_ref_, f_cur_, frame_ref_->cam_->errorMultiplier2(), Config::poseOptimThresh(),
							inliers_, xyz_in_cur_, T_cur_from_ref_);
		VLOG(5)<<"Init: fundamental RANSAC= "<<inliers_.size()<<" inliers.";
	}
	
	// draw features
	if(0){
		int img_width = frame_cur->img_pyr_[0].cols;
		int img_height = frame_cur->img_pyr_[0].rows;
		Mat show_image = Mat(2 *img_height, img_width, CV_8UC3);;
		Mat ref_image = show_image(Rect(0, 0, img_width, img_height));
		Mat curr_image = show_image(Rect(0, img_height, img_width, img_height));
		cvtColor(frame_ref_->img_pyr_[0], ref_image, CV_GRAY2BGR);
		cvtColor(frame_cur->img_pyr_[0], curr_image, CV_GRAY2BGR);
		for(vector<int>::iterator it=inliers_.begin(); it!=inliers_.end(); it++){
			circle(show_image, px_ref_[*it], 3, Scalar(0, 255, 0), 2);
			circle(show_image, px_cur_[*it]+Point2f(0, img_height), 3, Scalar(0, 255, 0), 2);
			line(show_image,px_ref_[*it],  px_cur_[*it] + Point2f(0, img_height), Scalar(0, 255, 0));
		}
		imshow("track", show_image);
	}

	if(inliers_.size() < Config::initMinInliers()){
		SVO_WARN_STREAM("Init WARNING: "<<Config::initMinInliers()<<" inliers minimum required.");
		return FAILURE;
	}

	// Rescale the map such that the mean scene depth is equal to the specified scale
	vector<double> depth_vec;
	for(size_t i=0; i<xyz_in_cur_.size(); ++i){
		depth_vec.push_back((xyz_in_cur_[i]).z());
	}
	double scene_depth_median = vk::getMedian(depth_vec);
	
	// 因为computeHomography 算出来的平移是带尺度的，要归一化到统一的尺度下
	double scale = Config::mapScale()/scene_depth_median;
	frame_cur->T_f_w_ = T_cur_from_ref_ * frame_ref_->T_f_w_;
	// 加了一个尺度因子
	frame_cur->T_f_w_.translation() =
		-frame_cur->T_f_w_.rotation_matrix()*(frame_ref_->pos() + scale*(frame_cur->pos() - frame_ref_->pos()));

	// For each inlier create 3D point and add feature in both frames
	SE3 T_world_cur = frame_cur->T_f_w_.inverse();
	for(vector<int>::iterator it=inliers_.begin(); it!=inliers_.end(); ++it){
		Vector2d px_cur(px_cur_[*it].x, px_cur_[*it].y);
		Vector2d px_ref(px_ref_[*it].x, px_ref_[*it].y);
		if(frame_ref_->cam_->isInFrame(px_cur.cast<int>(), 10) && frame_ref_->cam_->isInFrame(px_ref.cast<int>(), 10) 
					&& xyz_in_cur_[*it].z() > 0){
		Vector3d pos = T_world_cur * (xyz_in_cur_[*it]*scale);
		Point* new_point = new Point(pos);

		Feature* ftr_cur(new Feature(frame_cur.get(), new_point, px_cur, f_cur_[*it], 0));
		frame_cur->addFeature(ftr_cur);
		// 特征点也要存好它被哪些帧看到了。bundler adjustment 的时候要用
		new_point->addFrameRef(ftr_cur);

		Feature* ftr_ref(new Feature(frame_ref_.get(), new_point, px_ref, f_ref_[*it], 0));
		frame_ref_->addFeature(ftr_ref);
		new_point->addFrameRef(ftr_ref);
		}
	}
	return SUCCESS;
}

void KltHomographyInit::reset()
{
  px_cur_.clear();
  frame_ref_.reset();
}

/// 检测fast角度，输出的是对应的点和点的方向向量（可以考虑为点的反投影坐标）
void KltHomographyInit::detectFeatures( FramePtr frame, vector<cv::Point2f>& px_vec, vector<Vector3d>& f_vec)
{
	Features new_features;
	//参数: const int img_width,const int img_height,const int cell_size,const int n_pyr_levels
	feature_detection::FastDetector detector(
		frame->img().cols, frame->img().rows, Config::gridSize(), Config::nPyrLevels());
	
	detector.detect(frame.get(), frame->img_pyr_, Config::triangMinCornerScore(), new_features);

	// now for all maximum corners, initialize a new seed
	// 返回特征位置和特征的单位向量
	px_vec.clear(); px_vec.reserve(new_features.size());
	f_vec.clear(); f_vec.reserve(new_features.size());
	std::for_each(new_features.begin(), new_features.end(), [&](Feature* ftr){
		px_vec.push_back(cv::Point2f(ftr->px[0], ftr->px[1]));
		f_vec.push_back(ftr->f); // f:Unit-bearing vector of the feature.
		delete ftr;
	});
}

void KltHomographyInit::trackKlt(  FramePtr frame_ref, FramePtr frame_cur,
                vector<cv::Point2f>& px_ref, vector<cv::Point2f>& px_cur,
                vector<Vector3d>& f_ref, vector<Vector3d>& f_cur,
                vector<double>& disparities)
{
	const double klt_win_size = 30.0;
	const int klt_max_iter = 30;
	const double klt_eps = 0.001;
	vector<uchar> status;
	vector<float> error;
	vector<float> min_eig_vec;
	cv::TermCriteria termcrit(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, klt_max_iter, klt_eps);
	cv::calcOpticalFlowPyrLK(frame_ref->img_pyr_[0], frame_cur->img_pyr_[0],
							px_ref, px_cur, status, error,
							cv::Size2i(klt_win_size, klt_win_size),
							4, termcrit, cv::OPTFLOW_USE_INITIAL_FLOW);

	vector<cv::Point2f>::iterator px_ref_it = px_ref.begin();
	vector<cv::Point2f>::iterator px_cur_it = px_cur.begin();
	vector<Vector3d>::iterator f_ref_it = f_ref.begin();
	f_cur.clear(); f_cur.reserve(px_cur.size());
	disparities.clear(); disparities.reserve(px_cur.size());
	
	for(size_t i=0; px_ref_it != px_ref.end(); ++i){
		if(!status[i]){
			//如果光流没有发现，则删除
		px_ref_it = px_ref.erase(px_ref_it);
		px_cur_it = px_cur.erase(px_cur_it);
		f_ref_it = f_ref.erase(f_ref_it);
		continue;
		}
		f_cur.push_back(frame_cur->c2f(px_cur_it->x, px_cur_it->y));
		disparities.push_back(Vector2d(px_ref_it->x - px_cur_it->x, px_ref_it->y - px_cur_it->y).norm());
		++px_ref_it;
		++px_cur_it;
		++f_ref_it;
	}
}

// ??? 这边给出一个提示，因为要保证16字节对齐，对于Vector2d之类的容器形式要写成 
// std::vector<Vector2d, aligned_allocator<Vector2d>>
void KltHomographyInit::computeHomography( const vector<Vector3d>& f_ref,  const vector<Vector3d>& f_cur,
                        double focal_length, double reprojection_threshold, vector<int>& inliers,
                        vector<Vector3d>& xyz_in_cur,  SE3& T_cur_from_ref)
{
  vector<Vector2d > uv_ref(f_ref.size());
  vector<Vector2d > uv_cur(f_cur.size());
  for(size_t i=0, i_max=f_ref.size(); i<i_max; ++i){
    uv_ref[i] = vk::project2d(f_ref[i]); // uv_ref是深度归一化后的
    uv_cur[i] = vk::project2d(f_cur[i]);
  }
  vk::Homography Homography(uv_ref, uv_cur, focal_length, reprojection_threshold);
  Homography.computeSE3fromMatches();
  
  vector<int> outliers;
  vk::computeInliers(f_cur, f_ref,
                     Homography.T_c2_from_c1.rotation_matrix(), Homography.T_c2_from_c1.translation(),
                     reprojection_threshold, focal_length,
                     xyz_in_cur, inliers, outliers);
  T_cur_from_ref = Homography.T_c2_from_c1;
}


void KltHomographyInit::computeFundamental( 
                        const cv::Mat img_ref, const cv::Mat img_cur, 
                        const vector<Vector3d>& f_ref,  const vector<Vector3d>& f_cur,
                        double focal_length, double reprojection_threshold, vector<int>& inliers,
                        vector<Vector3d>& xyz_in_cur,  SE3& T_cur_from_ref)
{
	vector<Vector2d > uv_ref(f_ref.size());
	vector<Vector2d > uv_cur(f_cur.size());
	for(size_t i=0, i_max=f_ref.size(); i<i_max; ++i){
		uv_ref[i] = vk::project2d(f_ref[i]); // uv_ref是深度归一化后的
		uv_cur[i] = vk::project2d(f_cur[i]);
	}
	//   Eigen::Matrix3d K = frame_ref_->cam_->K_;
	vk::Fundamental Fundamental(img_ref, img_cur, uv_ref, uv_cur, focal_length, reprojection_threshold);
	Fundamental.computeSE3fromMatches();
	
	vector<int> outliers;
	vk::computeInliers(f_cur, f_ref,
						Fundamental.T_c2_from_c1.rotation_matrix(), Fundamental.T_c2_from_c1.translation(),
						reprojection_threshold, focal_length,
						xyz_in_cur, inliers, outliers);
	T_cur_from_ref = Fundamental.T_c2_from_c1;
}


} // namespace initialization
} // namespace svo
