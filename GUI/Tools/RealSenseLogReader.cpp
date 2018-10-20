/*
 *
 * Copyright (C) 2018 Intel Corp
 * This file is part of REFrame
 *
 * Author: Kshitij Khode
 *
 */

#include "RealSenseLogReader.h"

#include <librealsense/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace rs;

int const FRAMERATE = 60;

context _rs_ctx;
device* _rs_camera = NULL;
intrinsics _depth_intrin;
intrinsics _color_intrin;

RealSenseLogReader::RealSenseLogReader(std::string file, bool flipColors) : LogReader(file, flipColors) {
  rs::log_to_console(rs::log_severity::warn);
  std::cout << "Creating live capture... ";

  if(!initStreaming()) {
    rs::log_to_console(rs::log_severity::fatal);
    std::cout << "Unable to locate a camera" << std::endl;
    std::cout.flush();

    return;
  }

  _depth_intrin = _rs_camera->get_stream_intrinsics(rs::stream::depth);
  _color_intrin = _rs_camera->get_stream_intrinsics(rs::stream::color);

  data.allocateRGBD(_depth_intrin.width, _depth_intrin.height);

  if(_rs_camera->is_streaming()) _rs_camera->wait_for_frames();
}

RealSenseLogReader::~RealSenseLogReader() { _rs_camera->stop(); }

bool RealSenseLogReader::initStreaming() {
  if(_rs_ctx.get_device_count() > 0) {
    _rs_camera = _rs_ctx.get_device(0);
    _rs_camera->enable_stream(rs::stream::color,
                              Resolution::getInstance().width(),
                              Resolution::getInstance().height(),
                              rs::format::rgb8,
                              FRAMERATE);
    _rs_camera->enable_stream(rs::stream::depth,
                              Resolution::getInstance().width(),
                              Resolution::getInstance().height(),
                              rs::format::z16,
                              FRAMERATE);
    _rs_camera->start();

    return true;
  }
  return false;
}

void RealSenseLogReader::getNext() {

  if(_rs_camera->is_streaming()) _rs_camera->wait_for_frames();

  cv::Mat depth16(_depth_intrin.height, _depth_intrin.width, CV_16U, (uchar*)_rs_camera->get_frame_data(rs::stream::depth));
  cv::Mat rgb(_color_intrin.height, _color_intrin.width, CV_8UC3, (uchar*)_rs_camera->get_frame_data(rs::stream::color));

  depth16.convertTo(data.depth, CV_32FC1, 0.001);
  rgb.convertTo(data.rgb, CV_8UC3);

  data.timestamp = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
  if (flipColors) data.flipColors();
}

const std::string RealSenseLogReader::getFile() { return Parse::get().baseDir().append("live"); }

FrameData RealSenseLogReader::getFrameData() { return data; }

int RealSenseLogReader::getNumFrames() { return std::numeric_limits<int>::max(); }

bool RealSenseLogReader::hasMore() { return true; }

void RealSenseLogReader::setAuto(bool value) {}
