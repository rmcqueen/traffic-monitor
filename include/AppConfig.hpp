#ifndef TRAFFIC_MONITOR_RUN_H
#define TRAFFIC_MONITOR_RUN_H

#include "Tracker.hpp"
#include "BackgroundSubtractor.hpp"

class AppConfig {
 private:
  Tracker tracker;
  BackgroundSubtractor bgs;
  cv::VideoCapture capVideo;
  std::vector<cv::Point> start_points;
  std::vector<cv::Point> end_points;
  std::vector<cv::Point> crossing_lines;
  double FPS;
  int FRAME_WIDTH;
  int FRAME_HEIGHT;
  std::string SOURCE_VIDEO_PATH;
  int calibration_region_area;
  bool live_capture;

 public:
  AppConfig();
  AppConfig(Tracker &tracker_,
            BackgroundSubtractor &bgs_,
            std::vector<cv::Point> &crossing_lines_,
            std::vector<cv::Point> &start_points_,
            std::vector<cv::Point> &end_points_,
            std::string video_path_,
            double fps_,
            int frame_width_,
            int frame_height_,
            int calibration_region_area_);

  virtual ~AppConfig();

  const Tracker &get_tracker() const;
  void set_tracker(const Tracker &tracker);

  const BackgroundSubtractor &get_background_subtractor() const;
  void set_background_subtractor(const BackgroundSubtractor &bgs);

  const cv::Point &get_crossing_lines() const;
  void set_crossing_lines(const std::vector<cv::Point> crossing_lines_);

  const std::string &get_SOURCE_VIDEO_PATH() const;
  void set_SOURCE_VIDEO_PATH(const std::string &PATH_);

  const std::vector<cv::Point> &get_start_points() const;
  void set_start_points(const std::vector<cv::Point> &start_points);

  const std::vector<cv::Point> &get_end_points() const;
  void set_end_points(const std::vector<cv::Point> &end_points);

  const double &get_FPS() const;
  void set_FPS(const double FPS_);

  const int &get_FRAME_WIDTH() const;
  void set_FRAME_WIDTH(const int WIDTH_);

  const int &get_FRAME_HEIGHT() const;
  void set_FRAME_HEIGHT(const int HEIGHT_);

  const int &get_calibration_region_area() const;
  void set_calibration_region_area(const int area_);


  void run();
};
#endif //TRAFFIC_MONITOR_RUN_H
