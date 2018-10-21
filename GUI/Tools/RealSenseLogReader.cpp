/*
 *
 * Copyright (C) 2018 Intel Corp
 * This file is part of REFrame
 *
 * Author: Kshitij Khode
 *
 */

#include "RealSenseLogReader.h"

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace rs2;

rs2::pipeline _pipe2;

RealSenseLogReader::RealSenseLogReader(std::string file, bool flipColors) : LogReader(file, flipColors) {

  rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
  std::cout << "Creating live capture... ";

  _pipe2.start();

  rs2::frameset frameSet = _pipe2.wait_for_frames();
  depth_frame depthFrame = frameSet.get_depth_frame();

  data.allocateRGBD(depthFrame.get_width(), depthFrame.get_height());
}

RealSenseLogReader::~RealSenseLogReader() { }

void RealSenseLogReader::getNext() {

  rs2::frameset frameSet = _pipe2.wait_for_frames();
  depth_frame depthFrame = frameSet.get_depth_frame();
  video_frame colorFrame = frameSet.get_color_frame();

  // KBK Channge get_height to HEIGHT, (void*) to (*uchar)
  cv::Mat depth16(depthFrame.get_height(), depthFrame.get_width(), CV_16UC1, (uchar*)depthFrame.get_data());
  cv::Mat rgb8(colorFrame.get_height(), colorFrame.get_width(), CV_8UC3, (uchar*)colorFrame.get_data());

  depth16.convertTo(data.depth, CV_32FC1, 0.001);
  rgb8.convertTo(data.rgb, CV_8UC3);

  data.timestamp = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
  if (flipColors) data.flipColors();
}

const std::string RealSenseLogReader::getFile() { return Parse::get().baseDir().append("live"); }

FrameData RealSenseLogReader::getFrameData() { return data; }

int RealSenseLogReader::getNumFrames() { return std::numeric_limits<int>::max(); }

bool RealSenseLogReader::hasMore() { return true; }

void RealSenseLogReader::setAuto(bool value) {}
