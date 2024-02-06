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

#include <fstream>
#include <memory>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
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
private:
	cv::VideoWriter output_video;
	std::ofstream output_time;

	ros::Time g_last_wrote_time;
	std::string codec;
	int fps;
	std::string filename;

public:
	Recorder(remote_video_recorder::RemoteRecord::Request const & req) :
			filename(req.filename), fps(req.fps), codec(req.codec), 
			g_last_wrote_time(ros::Time(0)) {
		if (filename.empty()) {
			filename = "output.avi";
		}
		if (codec.empty()) {
			codec = "MJPG";
		}
		if (fps == 0) {
			fps = 15;
		}

		if (codec.size() != 4) {
			ROS_ERROR_STREAM("Invalid codec [" << codec 
				<< "]: The video codec must be a FOURCC identifier (4 chars)");
			throw std::runtime_error("invalid codec");
		}

		// Create the parent directory of the filename.
		boost::filesystem::path p(filename);
		if (p.has_parent_path()) {
			boost::filesystem::create_directories(p.parent_path());
		}
		boost::filesystem::path pt = boost::filesystem::path(filename).replace_extension(".ts.txt");
		if (boost::filesystem::exists(p) || boost::filesystem::exists(pt)) {
			ROS_ERROR_STREAM("Requested file exists: " << p);
			throw std::runtime_error("file exists");
		}
		this->output_time.open(pt.native());
		if (!this->output_time.is_open()) {
			ROS_ERROR_STREAM("Could not open file: " << pt);
			throw std::runtime_error("failed to create ts file");
		}
	}

	bool init_video(int const width, int const height) {
		output_video.open(filename,
#if CV_MAJOR_VERSION >= 3
				cv::VideoWriter::fourcc(codec.c_str()[0],
#else
						CV_FOURCC(codec.c_str()[0],
#endif
						codec.c_str()[1], codec.c_str()[2],
						codec.c_str()[3]),
						fps,
						cv::Size(width, height),
						true);

		if (!output_video.isOpened()) {
			ROS_ERROR(
					"Could not create the output video! Check filename and/or support for codec.");
			return false;
		}
		ROS_INFO_STREAM(
				"Starting to record " << codec << " video at " << cv::Size(width, height) << "@" << fps << "fps.");
		return true;
	}

	void record_frame(cv::Mat const & frame, ros::Time const & rec_time) {
		// init if first frame
		if (this->g_last_wrote_time.is_zero()) {
			if (!this->init_video(frame.cols, frame.rows)) {
				throw std::runtime_error("failed to init video");
			}
		}
		if ((rec_time - this->g_last_wrote_time) < ros::Duration(1. / this->fps)) {
			return;
		}

		this->g_last_wrote_time = rec_time;
		this->output_video.write(frame);
		this->output_time << rec_time.sec << "," << rec_time.nsec << std::endl;
	}
};

struct Source {
protected:
	Recorder recorder;
public:
	Source(remote_video_recorder::RemoteRecord::Request const & req) : recorder(req) {}
	virtual ~Source() {}
};

struct TopicSource : public Source {
private:
	std::string encoding;
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Subscriber sub_image;

public:
	TopicSource(remote_video_recorder::RemoteRecord::Request const & req) :
			Source(req), encoding(req.encoding), nh(), it(nh) {
		if (encoding.empty()) {
			encoding = "bgr8";
		}
		std::string topic = nh.resolveName(req.source);
		sub_image = it.subscribe(topic, 1,
				&TopicSource::callback, this);
	}
	virtual ~TopicSource() {}
	void callback(const sensor_msgs::ImageConstPtr& image_msg) {
		try {
			cv_bridge::CvtColorForDisplayOptions options;
			const cv::Mat image = cv_bridge::cvtColorForDisplay(
					cv_bridge::toCvShare(image_msg), encoding, options)->image;
			if (!image.empty()) {
				this->recorder.record_frame(image, image_msg->header.stamp);
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

struct DevSource : public Source {
private:
	boost::thread thread;
	std::unique_ptr<cv::VideoCapture> video_source;
	bool is_running;

public:
	DevSource(remote_video_recorder::RemoteRecord::Request const & req) :
			Source(req), is_running(true)  {

		video_source.reset(new cv::VideoCapture(boost::lexical_cast<int>(req.source)));
		if (!video_source->isOpened()) {
			ROS_ERROR_STREAM("Failed to open device: " << req.source);
			throw std::runtime_error("failed to open device");
		}

		this->thread = boost::thread(&DevSource::run, this);
	}
	
	virtual ~DevSource() {
		this->is_running = false;
		this->thread.join();
	}

	void run() {
		while (this->is_running) {
			// boost::thread::interruption_point(); // allow for external cancel
			// ROS_INFO_STREAM("Waiting for grab...");
			// this->video_source.grab();
			// ROS_INFO_STREAM("Grabbed");
			cv::Mat frame;
			// ROS_INFO_STREAM("reading...");
			bool ok = this->video_source->read(frame);
			ros::Time cap_time = ros::Time::now();
			// ROS_INFO_STREAM("read");
			if (!ok) {
				ROS_WARN("Grabbed frame but no retrieval");
			} else {
				// ROS_INFO_STREAM("recording...");
				this->recorder.record_frame(frame, cap_time);
				// ROS_INFO_STREAM("recorded...");
			}
		}
	}
};

std::unique_ptr<Source> create_source(remote_video_recorder::RemoteRecord::Request const & req) {
	switch (req.source_type) {
		case remote_video_recorder::RemoteRecord::Request::SOURCE_TOPIC:
			return std::unique_ptr<Source>(new TopicSource(req));
		case remote_video_recorder::RemoteRecord::Request::SOURCE_DEV:
			return std::unique_ptr<Source>(new DevSource(req));
		default:
			ROS_ERROR_STREAM("Unknown source type: " << req.source_type);
			throw std::runtime_error("unknown source");
	} 
}

struct RecorderNode {
	RecorderNode() : nh("~") {
		service = nh.advertiseService("control", &RecorderNode::callback, this);
		ROS_INFO("Ready to receive data.");
	}

	bool callback(remote_video_recorder::RemoteRecord::Request & req, remote_video_recorder::RemoteRecord::Response & rep) {
		std::string msg;
		if (source) {
			source.reset();
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
			ROS_INFO_STREAM("Got request: " << req);
			source = create_source(req);
			rep.ok = true;
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
	std::unique_ptr<Source> source;
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "remote_video_recorder");
	RecorderNode node;
	ros::spin();
}

