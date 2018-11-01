/*
 *
 * Copyright (C) 2018 Intel Corp
 * This file is part of REFrame
 *
 * Author: Kshitij Khode
 *
 */

#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <poll.h>
#include <signal.h>

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Utils/Parse.h"
#include "LogReader.h"

using namespace rs2;

#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480
#define FRAME_RATE 60

class RealSenseLogReader : public LogReader {

 public:
  rs2::pipeline pipe2;

  RealSenseLogReader(std::string file, bool flipColors) : LogReader(file, flipColors) {

    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
    std::cout << "Creating live capture... ";

    rs2::config cfg;

    cfg.enable_stream(RS2_STREAM_DEPTH, FRAME_WIDTH, FRAME_HEIGHT, RS2_FORMAT_Z16, FRAME_RATE);
    cfg.enable_stream(RS2_STREAM_COLOR, FRAME_WIDTH, FRAME_HEIGHT, RS2_FORMAT_RGB8, FRAME_RATE);

    rs2::device selectedDev = pipe2.start(cfg).get_device();

    rs2::sensor depthSensor = selectedDev.query_sensors()[0];
    rs2::sensor colorSensor = selectedDev.query_sensors()[1];

    depthSensor.set_option(RS2_OPTION_LASER_POWER, 1.f);
    depthSensor.set_option(RS2_OPTION_ACCURACY, 1.f);
    depthSensor.set_option(RS2_OPTION_MOTION_RANGE, 9.f);
    depthSensor.set_option(RS2_OPTION_FILTER_OPTION, 7.f);
    depthSensor.set_option(RS2_OPTION_CONFIDENCE_THRESHOLD, 0.f);

    rs2::frameset frameSet = pipe2.wait_for_frames();
    depth_frame depthFrame = frameSet.get_depth_frame();
    video_frame colorFrame = frameSet.get_color_frame();

    assert(depthFrame.get_height() == colorFrame.get_height() && depthFrame.get_width() == colorFrame.get_width());

    data.allocateRGBD(depthFrame.get_width(), depthFrame.get_height());
  }

  ~RealSenseLogReader() {}

  void getNext() {

    rs2::align align(RS2_STREAM_COLOR);
    rs2::frameset frameSet = pipe2.wait_for_frames();
    frameSet = align.process(frameSet);

    depth_frame depthFrame = frameSet.get_depth_frame();
    video_frame colorFrame = frameSet.get_color_frame();

    cv::Mat depth16(depthFrame.get_height(), depthFrame.get_width(), CV_16UC1, (uchar*)depthFrame.get_data());
    cv::Mat rgb8(colorFrame.get_height(), colorFrame.get_width(), CV_8UC3, (uchar*)colorFrame.get_data());

    depth16.convertTo(data.depth, CV_32FC1, 0.001f * 6.f/32.f);
    rgb8.convertTo(data.rgb, CV_8UC3);

    data.timestamp = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
    if (flipColors) data.flipColors();
  }

  const std::string getFile() { return Parse::get().baseDir().append("live"); }
  FrameData getFrameData() { return data; }
  int getNumFrames() { return std::numeric_limits<int>::max(); }
  bool hasMore() { return true; }
  void setAuto(bool value) {}
  bool rewind() { return false; }
  void getPrevious() {}
  void fastForward(int frame) {}


 private:
  FrameData data;

};


// if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
// {
//     depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
//     depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Disable emitter
// }
// if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
// {
//     // Query min and max values:

