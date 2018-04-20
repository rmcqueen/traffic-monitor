#include <opencv2/opencv.hpp>


#include "AppConfig.hpp"

int main(int argc, char *argv[]) {
  Tracker tracker;
  BackgroundSubtractor bgs;
  std::vector<cv::Point> crossing_lines;
  std::vector<cv::Point> start_points;
  std::vector<cv::Point> end_points;
  std::string video_path = "data/car_only.mov";
  double fps = 30.0;
  int frame_width = 640;
  int frame_height = 480;
  int calibration_region_area = 4;

  AppConfig app(tracker,
                bgs,
                crossing_lines,
                start_points,
                end_points,
                video_path,
                fps,
                frame_width,
                frame_height,
                calibration_region_area
  );

  app.run();

  return 0;
}