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
#include <ros/package.h>
#include <string>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/config.h>
#include <svo_ros/visualizer.h>
#include <vikit/params_helper.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include <vikit/abstract_camera.h>
#include <vikit/camera_loader.h>
#include <vikit/user_input_thread.h>

#include "kitti_parser.h"
#include "common.h"
namespace svo {

/// SVO Interface
class VoNode
{
    public:
        svo::FrameHandlerMono* vo_;
        svo::Visualizer visualizer_;
        bool publish_markers_;                 //!< publish only the minimal amount of info (choice for embedded devices)
        bool publish_dense_input_;
        boost::shared_ptr<vk::UserInputThread> user_input_thread_;
        ros::Subscriber sub_remote_key_;
        std::string remote_input_;
        vk::AbstractCamera* cam_;
        bool quit_;
        VoNode();
        ~VoNode();
        void imgCb(const sensor_msgs::ImageConstPtr& msg);
        void processUserActions();
        void remoteKeyCb(const std_msgs::StringConstPtr& key_input);
        void RunSVO(const image_ img_new);
};

VoNode::VoNode() :
  vo_(NULL),
  publish_markers_(vk::getParam<bool>("svo/publish_markers", true)),
  publish_dense_input_(vk::getParam<bool>("svo/publish_dense_input", false)),
  remote_input_(""),
  cam_(NULL),
  quit_(false)
{
	// Start user input thread in parallel thread that listens to console keys
	if(vk::getParam<bool>("svo/accept_console_user_input", true))
	user_input_thread_ = boost::make_shared<vk::UserInputThread>();

	// Create Camera,load param: /spg_svo/svo_ros/param/..
	if(!vk::camera_loader::loadFromRosNs("svo", cam_))
	throw std::runtime_error("Camera model not correctly specified.");

	// Get initial position and orientation
	visualizer_.T_world_from_vision_ = Sophus::SE3(
		vk::rpy2dcm(Vector3d(vk::getParam<double>("svo/init_rx", 0.0),
							vk::getParam<double>("svo/init_ry", 0.0),
							vk::getParam<double>("svo/init_rz", 0.0))),
		Eigen::Vector3d(vk::getParam<double>("svo/init_tx", 0.0),
						vk::getParam<double>("svo/init_ty", 0.0),
						vk::getParam<double>("svo/init_tz", 0.0)));

	// Init VO and start
	vo_ = new svo::FrameHandlerMono(cam_);
	vo_->start();
}

VoNode::~VoNode()
{
	delete vo_;
	delete cam_;
	if(user_input_thread_ != NULL)
		user_input_thread_->stop();
}

void VoNode::imgCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat img;
    try {
        img = cv_bridge::toCvShare(msg, "mono8")->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    processUserActions();
    vo_->addImage(img, msg->header.stamp.toSec());
    visualizer_.publishMinimal(img, vo_->lastFrame(), *vo_, msg->header.stamp.toSec());

    if(publish_markers_ && vo_->stage() != FrameHandlerBase::STAGE_PAUSED)
        visualizer_.visualizeMarkers(vo_->lastFrame(), vo_->coreKeyframes(), vo_->map());

    if(publish_dense_input_)
        visualizer_.exportToDense(vo_->lastFrame());

    if(vo_->stage() == FrameHandlerMono::STAGE_PAUSED)
        usleep(100000);
}

void VoNode::processUserActions()
{
	char input = remote_input_.c_str()[0];
	remote_input_ = "";

	if(user_input_thread_ != NULL){
		char console_input = user_input_thread_->getInput();
		if(console_input != 0)
			input = console_input;
	}

	switch(input){
		case 'q':
			quit_ = true;
			printf("SVO user input: QUIT\n");
			break;
		case 'r':
			vo_->reset();
			printf("SVO user input: RESET\n");
			break;
		case 's':
			vo_->start();
			printf("SVO user input: START\n");
			break;
		default: ;
	}
}

void VoNode::remoteKeyCb(const std_msgs::StringConstPtr& key_input)
{
	remote_input_ = key_input->data;
}

void VoNode::RunSVO(const image_ img_new)
{
	// main function
    double time_s = img_new.timestamp*1e-9;
    vo_->addImage(img_new.image, time_s);
	
    visualizer_.publishMinimal(img_new.image, vo_->lastFrame(), *vo_, time_s);
    if(publish_markers_ && vo_->stage() != FrameHandlerBase::STAGE_PAUSED)
		visualizer_.visualizeMarkers(vo_->lastFrame(), vo_->coreKeyframes(), vo_->map());

    if(publish_dense_input_)
		visualizer_.exportToDense(vo_->lastFrame());

    if(vo_->stage() == FrameHandlerMono::STAGE_PAUSED)
		usleep(100000);
}

} // namespace svo

int main(int argc, char **argv)
{
    // parse gflags
    google::ParseCommandLineFlags(&argc, &argv, true);
    #if defined(USE_GLOG)
        printf("USE GLOG......\n");
        FLAGS_log_dir = "/home/yj/src/CV/SLAM/svo_workspace/run/log";
        FLAGS_v = 6;
        google::InitGoogleLogging(argv[0]);
        google::SetStderrLogging(6);        //设置级别高于 google::INFO 的日志同时输出到屏幕
        FLAGS_colorlogtostderr = true;    //设置输出到屏幕的日志显示相应颜色
        FLAGS_logbufsecs = 0;            //缓冲日志输出，默认为30秒，此处改为立即输出
        google::InstallFailureSignalHandler();      //捕捉 core dumped
    #endif
        
    ros::init(argc, argv, "svo");
    ros::NodeHandle nh;

    std::cout << "create vo_node" << std::endl;
    svo::VoNode vo_node;
    // subscribe to cam msgs
    std::string cam_topic(vk::getParam<std::string>("svo/cam_topic", "camera/image_raw"));
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber it_sub = it.subscribe(cam_topic, 5, &svo::VoNode::imgCb, &vo_node);
    // subscribe to remote input
    vo_node.sub_remote_key_ = nh.subscribe("svo/remote_key", 5, &svo::VoNode::remoteKeyCb, &vo_node);

  // init window to show
	cv::namedWindow("img", 2);
	cv::resizeWindow("img", 600, 300);
    
  // start processing callbacks
    while(ros::ok() && !vo_node.quit_){
        // read kitti data
        string data_base_addr = "/home/yj/bak/data/kitti/drive/2011_09_30_drive_0034_extract";
        KittiParser* m_kitti_parser = new KittiParser(data_base_addr);
        m_kitti_parser->initialize();

        uint64_t id_image_start = 80;
		 uint64_t id_image = id_image_start;
        image_ img_cur;
        while(1){
            if(!m_kitti_parser->readImage(id_image++, img_cur)){
                printf("all the image have been read out, exe OVER !!!!!\n");
                exit(0);
            }
            // main
            vo_node.RunSVO(img_cur);
            VLOG(5)<<"************* image raw ID: "<< id_image-id_image_start <<" *************";
            cv::imshow("img", img_cur.image);
            cv::waitKey(-1);
            
        }

        ros::spinOnce();
        // TODO check when last image was processed. when too long ago. publish warning that no msgs are received!
    }

    printf("SVO terminated.\n");
    return 0;
}
