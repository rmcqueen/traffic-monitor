#include <opencv2/opencv.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#ifndef TRAFFIC_MONITOR_TRANSFORM_H
#define TRAFFIC_MONITOR_TRANSFORM_H

class Transform {
 private:
  std::vector<cv::Point2f> src_vec[4];
  std::vector<cv::Point2f> dst_vec[4];

 public:
  Transform();
  Transform(const cv::Mat &frame);
  Transform(cv::Point2f top_left_, cv::Point2f top_right_, cv::Point2f bottom_right_, cv::Point2f bottom_left_);
  Transform(cv::Mat frame,
            cv::Point2f top_left_,
            cv::Point2f top_right_,
            cv::Point2f bottom_right_,
            cv::Point2f bottom_left_);

  // Coordinates that correspond to the raw frame
  cv::Point2f top_left;
  cv::Point2f top_right;
  cv::Point2f bottom_right;
  cv::Point2f bottom_left;
  cv::Mat frame;

  void set_src_vec(cv::Point2f top_left_, cv::Point2f top_right_, cv::Point2f bottom_right_, cv::Point2f bottom_left_);

  void set_dst_vec(cv::Point2f top_left_, cv::Point2f top_right_, cv::Point2f bottom_right_, cv::Point2f bottom_left_);

  std::vector<cv::Point2f> get_src_vec();

  std::vector<cv::Point2f> get_dst_vec();

  void compute_birds_eye_view(std::vector<cv::Point2f> src_,
                              std::vector<cv::Point2f> dst_,
                              cv::Mat frame,
                              cv::Mat &OutputArray);

  cv::Mat compute_birds_eye_view(std::vector<cv::Point2f> src_, std::vector<cv::Point2f> dst_, cv::Mat frame);
};

#endif //TRAFFIC_MONITOR_TRANSFORM_H
