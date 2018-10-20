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

#include <Utils/Parse.h>

#include "LogReader.h"

class RealSenseLogReader : public LogReader {

 public:
  RealSenseLogReader(std::string file, bool flipColors);
  virtual ~RealSenseLogReader();

  bool initStreaming();
  void getNext();
  int getNumFrames();
  bool hasMore();
  bool rewind() { return false; }
  void getPrevious() {}
  void fastForward(int frame) {}
  const std::string getFile();
  FrameData getFrameData();
  void setAuto(bool value);

 private:
  FrameData data;
};
