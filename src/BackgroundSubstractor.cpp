/**
 * BackgroundSubtractor.cpp
 *
 * This class implements the BackgroundSubtraction functionality provided by OpenCV (https://opencv.org/) for the usage
 * of isolating cars from the highway they are travelling on.
 */

#include <opencv2/opencv.hpp>

#include "BackgroundSubtractor.hpp"

/**
 * Constructor for BackgroundSubtractor
 * @param alpha     See: https://en.wikipedia.org/wiki/Alpha_compositing for more information
 * @param threshold     value chosen between [0,255] indicating below what value to convert the pixels to black (0)
 * @param enable_threshold      bool indicating whether or not to incorporate thresholding on the frames
 * @param first_occurrence      bool indicating whether or not this is the first time BackgroundSubtraction has been
 * performed
 */
BackgroundSubtractor::BackgroundSubtractor() :
  alpha(0.05),
  threshold(160),
  enable_threshold(true),
  enable_open_close(true),
  first_occurrence(true) {

}

BackgroundSubtractor::~BackgroundSubtractor() = default;

BackgroundSubtractor::BackgroundSubtractor(cv::Mat &frame) :
  foreground_frame(frame) {};

void BackgroundSubtractor::set_foreground_frame(const cv::Mat &foreground_frame_) {
  BackgroundSubtractor::foreground_frame = foreground_frame_;
}

const cv::Mat &BackgroundSubtractor::get_foreground_frame() const {
  return foreground_frame;
}

/**
 * Removes the foreground objects in a frame. Necessary to remove any environmental noise such as trees slightly moing
 * or shadows being casted.
 * @param input_frame cv::Mat   the original frame to remove the background from
 * @param output_frame cv::Mat  a container for the foreground objects
 */
void BackgroundSubtractor::subtract(cv::Mat &input_frame, cv::Mat &output_frame) {
  assert(!input_frame.empty());

  if (first_occurrence) {
    mog_subtractor = cv::createBackgroundSubtractorMOG2();
    mog_subtractor->setDetectShadows(true);
    mog_subtractor->setHistory(100);
    mog_subtractor->setBackgroundRatio(0.4);
    mog_subtractor->setVarThreshold(8);
    mog_subtractor->setShadowThreshold(0.5);
    mog_subtractor->setNMixtures(3);
    mog_subtractor->setShadowValue(0);
    first_occurrence = false;
  }

  mog_subtractor->apply(input_frame, foreground_frame);

  // Threshold the foreground image to remove noise
  if (enable_threshold) {
    cv::threshold(foreground_frame, foreground_frame, threshold, 255, cv::THRESH_BINARY);
  }

  if (enable_open_close) {
    cv::Mat structuring_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
    cv::morphologyEx(foreground_frame, foreground_frame, cv::MORPH_ERODE, structuring_element);
    structuring_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(6, 6), cv::Point(-1, -1));
    cv::morphologyEx(foreground_frame, foreground_frame, cv::MORPH_CLOSE, structuring_element);
  }

  // Copy the contents of the processed foreground frame to the output image array
  foreground_frame.copyTo(output_frame);
}
