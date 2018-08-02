/*
 * remote_video_recorder.cpp
 *
 *  Created on: Aug 1, 2018
 *      Author: reubena
 */

//////
/// https://github.com/HARPLab/image_pipeline/blob/indigo/image_view/src/nodes/video_recorder.cpp
/****************************************************************************
 * Software License Agreement (Apache License)
 *
 *     Copyright (C) 2012-2013 Open Source Robotics Foundation
 *
 *     Licensed under the Apache License, Version 2.0 (the "License");
 *     you may not use this file except in compliance with the License.
 *     You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *     Unless required by applicable law or agreed to in writing, software
 *     distributed under the License is distributed on an "AS IS" BASIS,
 *     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *     See the License for the specific language governing permissions and
 *     limitations under the License.
 *
 *****************************************************************************/

#include <boost/scoped_ptr.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h>
#if CV_MAJOR_VERSION == 3
#include <opencv2/videoio.hpp>
#endif

#include "remote_video_recorder/RemoteRecord.h"

struct Recorder {
	cv::VideoWriter outputVideo;

	int g_count;
	ros::Time g_last_wrote_time;
	std::string encoding;
	std::string codec;
	int fps;
	std::string filename;
//	double min_depth_range;
//	double max_depth_range;
//	bool use_dynamic_range;
//	int colormap;
	image_transport::Subscriber sub_image;

	Recorder(ros::NodeHandle & nh, remote_video_recorder::RemoteRecord::Request const & req) :
			filename(req.filename), fps(req.fps), codec(req.codec), encoding(
					req.encoding), g_count(0), g_last_wrote_time(ros::Time(0)) {
		std::string topic_ref = req.topic;
		// Fill in some defaults
		if (filename.empty()) {
			filename = "output.avi";
		}
		if (codec.empty()) {
			codec = "MJPG";
		}
		if (fps == 0) {
			fps = 15;
		}
		if (encoding.empty()) {
			encoding = "bgr8";
		}
		if (topic_ref.empty()) {
			topic_ref = "/image";
		}

		if (codec.size() != 4) {
			ROS_ERROR("The video codec must be a FOURCC identifier (4 chars)");
			exit(-1);
		}

		image_transport::ImageTransport it(nh);
		std::string topic = nh.resolveName(topic_ref);
		sub_image = it.subscribe(topic, 1,
				&Recorder::callback, this);
	}

	void callback(const sensor_msgs::ImageConstPtr& image_msg) {
		if (!outputVideo.isOpened()) {

			cv::Size size(image_msg->width, image_msg->height);

			outputVideo.open(filename,
#if CV_MAJOR_VERSION == 3
					cv::VideoWriter::fourcc(codec.c_str()[0],
#else
							CV_FOURCC(codec.c_str()[0],
#endif
							codec.c_str()[1], codec.c_str()[2],
							codec.c_str()[3]),
							fps,
							size,
							true);

			if (!outputVideo.isOpened()) {
				ROS_ERROR(
						"Could not create the output video! Check filename and/or support for codec.");
				sub_image.shutdown();
			}

			ROS_INFO_STREAM(
					"Starting to record " << codec << " video at " << size << "@" << fps << "fps.");

		}

		if ((image_msg->header.stamp - g_last_wrote_time)
				< ros::Duration(1. / fps)) {
			// Skip to get video with correct fps
			return;
		}

		try {
			cv_bridge::CvtColorForDisplayOptions options;
//			options.do_dynamic_scaling = use_dynamic_range;
//			options.min_image_value = min_depth_range;
//			options.max_image_value = max_depth_range;
//			options.colormap = colormap;
			const cv::Mat image = cv_bridge::cvtColorForDisplay(
					cv_bridge::toCvShare(image_msg), encoding, options)->image;
			if (!image.empty()) {
				outputVideo << image;
				ROS_INFO_STREAM("Recording frame " << g_count << "\x1b[1F");
				g_count++;
				g_last_wrote_time = image_msg->header.stamp;
			} else {
				ROS_WARN("Frame skipped, no data!");
			}
		} catch (cv_bridge::Exception &) {
			ROS_ERROR("Unable to convert %s image to %s",
					image_msg->encoding.c_str(), encoding.c_str());
			return;
		}
	}


};

struct RecorderNode {
	RecorderNode() : nh() {}
	void start() {
		service = nh.advertiseService("control", &RecorderNode::callback, this);
	}

	bool callback(remote_video_recorder::RemoteRecord::Request & req, remote_video_recorder::RemoteRecord::Response & rep) {
		std::string msg;
		if (recorder) {
			recorder.reset();
			ROS_INFO("Stopped previous recording.");
			if (req.command == remote_video_recorder::RemoteRecord::Request::STOP) {
				rep.ok = true;
				rep.message = "Stopped previous recording";
				return true;
			}
		} else if (req.command == remote_video_recorder::RemoteRecord::Request::STOP) {
			rep.ok = true;
			rep.message = "No recording to stop";
			return true;
		}

		try {
			recorder.reset(new Recorder(nh, req));
			rep.ok = true;
			rep.message = "Recording to " + recorder->filename;
			ROS_INFO_STREAM("Recording to " << recorder->filename);
			return true;
		} catch (std::runtime_error & err) {
			rep.ok = false;
			rep.message = err.what();
			return true;
		}
	}

private:
	ros::NodeHandle nh;
	ros::ServiceServer service;
	boost::scoped_ptr<Recorder> recorder;
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "remote_video_recorder");
	RecorderNode node;
	node.start();
	ROS_INFO("Started remote video recorder.");
	ros::spin();
}

