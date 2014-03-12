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

#include <vikit/abstract_camera.h>
#include <stdlib.h>
#include <Eigen/StdVector>
#include <boost/bind.hpp>
#include <fstream>
#include <svo/frame_handler_base.h>
#include <svo/config.h>
#include <svo/feature.h>
#include <svo/matcher.h>
#include <svo/map.h>
#include <svo/point.h>
#include <svo/reprojection.h>
#include <svo/depth_filter.h>
 
namespace svo {

// definition of global and static variables which were declared in the header
#ifdef SVO_TRACE
vk::PerformanceMonitor* g_permon = NULL;
#endif

FrameHandlerBase::
FrameHandlerBase() :
  stage_(PAUSED),
  set_reset_(false),
  set_start_(false),
  acc_frame_timings_(10),
  acc_num_obs_(10),
  num_obs_last_(0),
  tracking_quality_(BAD),
  depth_filter_(NULL)
{
#ifdef SVO_TRACE
  // Initialize Performance Monitor
  g_permon = new vk::PerformanceMonitor();
  g_permon->addTimer("pyramid_creation");
  g_permon->addTimer("sparse_img_align");
  g_permon->addTimer("reproject");
  g_permon->addTimer("pose_optimizer");
  g_permon->addTimer("point_optimizer");
  g_permon->addTimer("local_ba");
  g_permon->addTimer("tot_time");
  g_permon->addLog("img_align_n_tracked");
  g_permon->addLog("repr_n_mps");
  g_permon->addLog("repr_n_new_references");
  g_permon->addLog("sfba_thresh");
  g_permon->addLog("sfba_error_init");
  g_permon->addLog("sfba_error_final");
  g_permon->addLog("sfba_n_edges_final");
  g_permon->addLog("loba_n_erredges_init");
  g_permon->addLog("loba_n_erredges_fin");
  g_permon->addLog("loba_err_init");
  g_permon->addLog("loba_err_fin");
  g_permon->addLog("n_candidates");
  g_permon->addLog("dropout");
  g_permon->init(Config::traceName(), Config::traceDir());
#endif

  // create depth filter and set callback
  feature_detection::DetectorPtr feature_detector(new feature_detection::FastDetector());
  DepthFilter::callback_t depth_filter_cb = boost::bind(&MapPointCandidates::newCandidatePoint, &map_.point_candidates_, _1, _2);
  depth_filter_ = new DepthFilter(feature_detector, depth_filter_cb);
  depth_filter_->startThread();

  SVO_INFO_STREAM("NanoSlam initialized");
}

FrameHandlerBase::
~FrameHandlerBase()
{
  SVO_INFO_STREAM("SVO destructor invoked.");
  delete depth_filter_;
  SVO_INFO_STREAM("SVO destructed.");
}

bool FrameHandlerBase::
startFrameProcessingCommon()
{
  if(set_start_)
  {
    resetAll();
    stage_ = FIRST_FRAME;
    set_start_ = false;
  }

  if(stage_ == PAUSED)
    return false;

  SVO_DEBUG_STREAM("New Frame");
  SVO_START_TIMER("tot_time");
  timer_.start();

  // some cleanup from last iteration, can't do before because of visualization
  map_.emptyTrash();
  return true;
}

int FrameHandlerBase::
finishFrameProcessingCommon(size_t update_id, UpdateResult dropout, size_t num_observations)
{
  SVO_DEBUG_STREAM("Frame: "<<update_id<<"\t fps-avg = "<< 1.0/acc_frame_timings_.getMean()<<"\t nObs = "<<acc_num_obs_.getMean());
  SVO_LOG(dropout);

  // save processing time to calculate fps
  acc_frame_timings_.push_back(timer_.stop());
  if(stage_ == DEFAULT_FRAME)
    acc_num_obs_.push_back(num_observations);
  num_obs_last_ = num_observations;
  SVO_STOP_TIMER("tot_time");

  #ifdef SVO_TRACE
    g_permon->writeToFile();
    {
      boost::unique_lock<boost::mutex> lock(map_.point_candidates_.mut_);
      size_t n_candidates = map_.point_candidates_.candidates_.size();
      SVO_LOG(n_candidates);
    }
  #endif

  // reset if failure or requested
  if(dropout == FAILURE || set_reset_)
    resetAll();
  return 0;
}

void FrameHandlerBase::
resetCommon()
{
  map_.reset();
  stage_ = PAUSED;
  set_reset_ = false;
  tracking_quality_ = BAD;
  depth_filter_->reset();
  num_obs_last_ = 0;
  SVO_INFO_STREAM("RESET");
}

void FrameHandlerBase::
setTrackingQuality(const size_t num_observations)
{
  tracking_quality_ = GOOD;
  if(num_observations < Config::qualityMinFts())
  {
    ROS_WARN_STREAM_THROTTLE(0.5, "Tracking less than "<< Config::qualityMinFts() <<" features!");
    tracking_quality_ = BAD;
  }
  const int feature_drop = static_cast<int>(num_obs_last_) - num_observations;
  if(feature_drop > Config::qualityMaxFtsDrop())
  {
    ROS_WARN_STREAM("Lost "<< feature_drop <<" features!");
    tracking_quality_ = BAD;
  }
  if(num_observations < 20)
  {
    ROS_ERROR_STREAM("Tracking less than 20 features!");
    tracking_quality_ = INSUFFICIENT;
  }
}

bool ptLastOptimComparator(Point* lhs, Point* rhs)
{
  return (lhs->last_structure_optim_ < rhs->last_structure_optim_);
}

void FrameHandlerBase::
optimizeStructure(FramePtr frame, size_t max_n_pts, int max_iter)
{
  deque<Point*> pts;
  for(Features::iterator it=frame->fts_.begin(); it!=frame->fts_.end(); ++it)
  {
    if((*it)->point != NULL)
      pts.push_back((*it)->point);
  }
  max_n_pts = min(max_n_pts, pts.size());
  nth_element(pts.begin(), pts.begin() + max_n_pts, pts.end(), ptLastOptimComparator);
  for(deque<Point*>::iterator it=pts.begin(); it!=pts.begin()+max_n_pts; ++it)
  {
    (*it)->optimize(max_iter);
    (*it)->last_structure_optim_ = frame->id_;
  }
}


} // namespace svo