/**
 * BackgroundSubtractor.hpp
 */

#ifndef TRAFFIC_MONITOR_BACKGROUNDSUBTRACTOR_H
#define TRAFFIC_MONITOR_BACKGROUNDSUBTRACTOR_H

#include <opencv2/opencv.hpp>

class BackgroundSubtractor {
 public:
  BackgroundSubtractor();
  BackgroundSubtractor(cv::Mat &frame);
  virtual ~BackgroundSubtractor();
  void set_foreground_frame(const cv::Mat &foreground_frame_);
  const cv::Mat &get_foreground_frame() const;
  void subtract(cv::Mat &input_frame, cv::Mat &output_frame);

 private:
  cv::Mat foreground_frame;
  cv::Ptr<cv::BackgroundSubtractorMOG2> mog_subtractor;
  double alpha;
  int threshold;
  bool enable_threshold;
  bool enable_open_close;
  bool first_occurrence;
};
#endif //TRAFFIC_MONITOR_BACKGROUNDSUBTRACTOR_H
