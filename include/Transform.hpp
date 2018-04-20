#ifndef TRAFFIC_MONITOR_TRANSFORM_H
#define TRAFFIC_MONITOR_TRANSFORM_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

class Transform {
 private:
  std::vector<cv::Point2f> src_vec;
  std::vector<cv::Point2f> dst_vec;
  std::vector<cv::Point2f> calibration_rect;

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

  const std::vector<cv::Point2f> &get_src_vec() const;

  void set_src_vec(const std::vector<cv::Point2f> &src_vec);

  const std::vector<cv::Point2f> &get_dst_vec() const;

  void set_dst_vec(const std::vector<cv::Point2f> &dst_vec);

  const std::vector<cv::Point2f> &get_calibration_rect() const;

  void set_calibration_rect(const std::vector<cv::Point2f> &calibration_rect);

  void set_src_vec(cv::Point2f top_left_,
                   cv::Point2f top_right_,
                   cv::Point2f bottom_right_,
                   cv::Point2f bottom_left_);

  void set_dst_vec(cv::Point2f top_left_,
                   cv::Point2f top_right_,
                   cv::Point2f bottom_right_,
                   cv::Point2f bottom_left_);

  void compute_birds_eye_view(std::vector<cv::Point2f> src_,
                              std::vector<cv::Point2f> dst_,
                              cv::Mat frame,
                              cv::Mat &OutputArray);

  cv::Mat compute_birds_eye_view(std::vector<cv::Point2f> src_,
                                 std::vector<cv::Point2f> dst_,
                                 cv::Mat frame);

  const std::vector<cv::Point2f> transform_calibration_rectangle();

  const std::vector<cv::Point2f> transform_calibration_rectangle(std::vector<cv::Point2f> &calibration_rect_);

  void draw_calibration_rectangle(cv::Mat &frame);

  void draw_transformed_calibration_rectangle(cv::Mat &frame, std::vector<cv::Point2f> &transformed_calib_rect);
};

#endif //TRAFFIC_MONITOR_TRANSFORM_H
