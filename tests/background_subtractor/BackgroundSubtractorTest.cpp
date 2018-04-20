#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

#include "BackgroundSubtractor.hpp"

TEST(BackgroundSubtractorTest, SubtractValidFrame) {
  BackgroundSubtractor subtractor;
  cv::Mat frame = cv::Mat::zeros(640, 480, CV_8UC1);
  cv::Mat foreground_frame;
  subtractor.subtract(frame, foreground_frame);
  ASSERT_NE(frame.data, foreground_frame.data);
}

TEST(BackgroundSubtractorTest, SubtractInvalidFrame) {
  BackgroundSubtractor subtractor;
  cv::Mat frame;
  cv::Mat foreground_frame;
  ASSERT_DEATH(subtractor.subtract(frame, foreground_frame), "Assertion failed: \\(!input_frame\\.empty\\(\\).*.*");
}


