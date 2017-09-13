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

#include <algorithm>
#include <stdexcept>
#include <svo/reprojector.h>
#include <svo/frame.h>
#include <svo/point.h>
#include <svo/feature.h>
#include <svo/map.h>
#include <svo/config.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <vikit/abstract_camera.h>
#include <vikit/math_utils.h>
#include <vikit/timer.h>

namespace svo {

Reprojector::Reprojector(vk::AbstractCamera* cam, Map& map) : map_(map)
{
	initializeGrid(cam);
}

Reprojector::~Reprojector()
{
	std::for_each(grid_.cells.begin(), grid_.cells.end(), [&](Cell* c){ delete c; });
}

void Reprojector::initializeGrid(vk::AbstractCamera* cam)
{
	grid_.cell_size = Config::gridSize();
	grid_.grid_n_cols = ceil(static_cast<double>(cam->width())/grid_.cell_size);
	grid_.grid_n_rows = ceil(static_cast<double>(cam->height())/grid_.cell_size);
	grid_.cells.resize(grid_.grid_n_cols*grid_.grid_n_rows);
	std::for_each(grid_.cells.begin(), grid_.cells.end(), [&](Cell*& c){ c = new Cell; });
	grid_.cell_order.resize(grid_.cells.size());
	for(size_t i=0; i<grid_.cells.size(); ++i)
		grid_.cell_order[i] = i;
	// 随机排序，一种策略
	random_shuffle(grid_.cell_order.begin(), grid_.cell_order.end()); // maybe we should do it at every iteration!
}

void Reprojector::resetGrid()
{
  n_matches_ = 0;
  n_trials_ = 0;
  std::for_each(grid_.cells.begin(), grid_.cells.end(), [&](Cell* c){ c->clear(); });
}

void Reprojector::reprojectMap( FramePtr frame, std::vector< std::pair<FramePtr,std::size_t> >& overlap_kfs)
{
	resetGrid();
	// Identify those Keyframes which share a common field of view.
	SVO_START_TIMER("reproject_kfs");
	list< pair<FramePtr,double> > close_kfs;
	// 获取所有有视野重叠的KF
	map_.getCloseKeyframes(frame, close_kfs);

	// Sort KFs with overlap according to their closeness
	// 对靠近的关键帧根据靠近程度进行排序
	//sort pairs based on the second element of the pair, the smaller is in front
	close_kfs.sort(boost::bind(&std::pair<FramePtr, double>::second, _1) <
					boost::bind(&std::pair<FramePtr, double>::second, _2));

	// Reproject all mappoints of the closest N kfs with overlap. We only store
	// in which grid cell the points fall.
	// 对有有重叠部分的N个关键帧对应的mappoints进行重投影，我们只存储格子中特征点减少的
	//把这些三维点投影到new frame 的所在单元格记录下来。
	//单元格会保存这个三维点的信息
	size_t n = 0;
	overlap_kfs.reserve(options_.max_n_kfs);
	// 对最近的N个关键帧进行迭代，找到有重叠视野
	for( auto it_frame=close_kfs.begin(), ite_frame=close_kfs.end();
				it_frame!=ite_frame && n<options_.max_n_kfs; ++it_frame, ++n ){
		FramePtr ref_frame = it_frame->first;
		overlap_kfs.push_back(pair<FramePtr,size_t>(ref_frame,0));

		// 1. map reprojection
		// Try to reproject each mappoint that the other KF observes
		// 对这个参考帧观察到的点投影到当前帧中
		for(auto it_ftr=ref_frame->fts_.begin(), ite_ftr=ref_frame->fts_.end(); it_ftr!=ite_ftr; ++it_ftr){
			// check if the feature has a mappoint assigned
			if((*it_ftr)->point == NULL)
				continue;

			// make sure we project a point only once
			// 同一帧中不同特征点可能对应地图上的同一个3D点,只投影一次
			if((*it_ftr)->point->last_projected_kf_id_ == frame->id_)
				continue;
			(*it_ftr)->point->last_projected_kf_id_ = frame->id_;
			
			// 将三维点投影到对应的cell
			if( reprojectPoint(frame, (*it_ftr)->point) )
				overlap_kfs.back().second++;
		}
	}
	SVO_STOP_TIMER("reproject_kfs");

	// candidates 保存的是当前keyframe中已经收敛的点，但是还没来得及插入地图
	// map 通过保存kf来保存3d点,kf中的特征点都记住了自己的3d point位置
	SVO_START_TIMER("reproject_candidates");
	{
		boost::unique_lock<boost::mutex> lock(map_.point_candidates_.mut_);
		auto it=map_.point_candidates_.candidates_.begin();
		while(it!=map_.point_candidates_.candidates_.end()){
		if(!reprojectPoint(frame, it->first)){
			it->first->n_failed_reproj_ += 3; // -YJ- why is 3??
			if(it->first->n_failed_reproj_ > 30){ // -YJ- why is 30?
				map_.point_candidates_.deleteCandidate(*it);
				it = map_.point_candidates_.candidates_.erase(it);
				continue;
			}
		}
		++it;
		}
	} // unlock the mutex when out of scope
	SVO_STOP_TIMER("reproject_candidates");

	// 2. feature align
	// Now we go through each grid cell and select one point to match.
	// At the end, we should have at maximum one reprojected point per cell.
	// 遍历所有的单元格，注意现在每个单元格中可能存在多个投影点
	SVO_START_TIMER("feature_align");
	for(size_t i=0; i<grid_.cells.size(); ++i){
		// we prefer good quality points over unkown quality (more likely to match)
		// and unknown quality over candidates (position not optimized)
		// 这里作者为了速度考虑，不是对图像上的所有cell都挑选，随机挑选了maxFts这么多个cell
		// 程序中grid_.cell_order[i]是用随机函数打乱了cell的排序.
		// 对同一单元格中的多个投影点选择一个进行alignment
		if(reprojectCell(*grid_.cells.at(grid_.cell_order[i]), frame))
			++n_matches_;
		if(n_matches_ > (size_t) Config::maxFts())
			break;
	}
	SVO_STOP_TIMER("feature_align");
}

bool Reprojector::pointQualityComparator(Candidate& lhs, Candidate& rhs)
{
  if(lhs.pt->type_ > rhs.pt->type_)
    return true;
  return false;
}

bool Reprojector::reprojectCell(Cell& cell, FramePtr frame)
{
  cell.sort(boost::bind(&Reprojector::pointQualityComparator, _1, _2));
  // cell 中保存了多个投影点
  Cell::iterator it=cell.begin();
  while(it!=cell.end()){
    ++n_trials_;

    if(it->pt->type_ == Point::TYPE_DELETED){
      it = cell.erase(it);
      continue;
    }

    bool found_match = true;
    if(options_.find_match_direct)
      found_match = matcher_.findMatchDirect(*it->pt, *frame, it->px);
    
	// 如果这个点附近没有好的优化位置，则删掉这个点继续寻找
    if(!found_match){
      it->pt->n_failed_reproj_++;
      if(it->pt->type_ == Point::TYPE_UNKNOWN && it->pt->n_failed_reproj_ > 15)
        map_.safeDeletePoint(it->pt);
      if(it->pt->type_ == Point::TYPE_CANDIDATE  && it->pt->n_failed_reproj_ > 30)
        map_.point_candidates_.deleteCandidatePoint(it->pt);
      it = cell.erase(it);
      continue;
    }
    it->pt->n_succeeded_reproj_++;
    if(it->pt->type_ == Point::TYPE_UNKNOWN && it->pt->n_succeeded_reproj_ > 10)
      it->pt->type_ = Point::TYPE_GOOD;

    Feature* new_feature = new Feature(frame.get(), it->px, matcher_.search_level_);
    frame->addFeature(new_feature);

    // Here we add a reference in the feature to the 3D point, the other way
    // round is only done if this frame is selected as keyframe.
    new_feature->point = it->pt;

    if(matcher_.ref_ftr_->type == Feature::EDGELET){
      new_feature->type = Feature::EDGELET;
      new_feature->grad = matcher_.A_cur_ref_*matcher_.ref_ftr_->grad;
      new_feature->grad.normalize();
    }

    // If the keyframe is selected and we reproject the rest, we don't have to
    // check this point anymore.
    it = cell.erase(it);

    // Maximum one point per cell.
    return true;
  }
  return false;
}

bool Reprojector::reprojectPoint(FramePtr frame, Point* point)
{
  Vector2d px(frame->w2c(point->pos_));
  if(frame->cam_->isInFrame(px.cast<int>(), 8)){ // 8px is the patch size in the matcher
    const int k = static_cast<int>(px[1]/grid_.cell_size)*grid_.grid_n_cols
                + static_cast<int>(px[0]/grid_.cell_size);
    grid_.cells.at(k)->push_back(Candidate(point, px));
    return true;
  }
  return false;
}

} // namespace svo
