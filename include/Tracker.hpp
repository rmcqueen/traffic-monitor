#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "Blob.hpp"

class Tracker {
 private:
  unsigned int car_count;
  cv::Mat frame1;
  cv::Mat frame2;
  std::vector<Blob> blobs;
  double fps;

 public:
  const cv::Scalar SCALAR_BLACK = cv::Scalar(0.0, 0.0, 0.0);
  const cv::Scalar SCALAR_WHITE = cv::Scalar(255.0, 255.0, 255.0);
  const cv::Scalar SCALAR_YELLOW = cv::Scalar(0.0, 255.0, 255.0);
  const cv::Scalar SCALAR_GREEN = cv::Scalar(0.0, 200.0, 0.0);
  const cv::Scalar SCALAR_RED = cv::Scalar(0.0, 0.0, 255.0);

  Tracker();
  Tracker(unsigned int car_count_, cv::Mat &frame1, cv::Mat &frame2, std::vector<Blob> &blobs, double fps_);
  void set_car_count(const unsigned int &car_count_);
  void set_frame1(const cv::Mat &frame1_);
  void set_frame2(const cv::Mat &frame2_);
  void set_blobs(const std::vector<Blob> &blobs_);
  void set_fps(const double &fps_);
  const unsigned int &get_car_count() const;
  const cv::Mat &get_frame1();
  const cv::Mat &get_frame2();
  const std::vector<Blob> &get_blobs();
  const double &get_fps();
  void match_current_frame_to_existing_blobs(std::vector<Blob> &existingBlobs, std::vector<Blob> &currentFrameBlobs);
  void add_blob_to_existing_blobs(Blob &currentFrameBlob, std::vector<Blob> &existingBlobs, int &intIndex);
  void add_new_blob(Blob &currentFrameBlob, std::vector<Blob> &existingBlobs);
  double distance_between_points(cv::Point point1, cv::Point point2);
  void draw_and_show_contours(cv::Size imageSize, std::vector<std::vector<cv::Point> > contours, std::string strImageName);
  void draw_and_show_contours(cv::Size imageSize, std::vector<Blob> blobs, std::string strImageName);
  void calculate_speed(Blob &blob, double conversion);
  void track_car_speed(std::vector<Blob> &blobs,
                       std::vector<cv::Point> &start_point,
                       std::vector<cv::Point> &end_point,
                       double conversion,
                       unsigned int frame_count);
  bool blob_crossed_line(std::vector<Blob> &blobs, int x_line_pos);
  void draw_blob_info_on_image(std::vector<Blob> &blobs, cv::Mat &imgFrame2Copy);
  void draw_car_count_on_image(const int &carCount, cv::Mat &imgFrame2Copy);
  void write_tracked_car_image(const cv::Mat &frame, cv::Rect bounding_rectangle, unsigned int car_id, std::string file_path);
  void write_tracked_car_speed(double speed, int blob_id, std::string file_path);
};
