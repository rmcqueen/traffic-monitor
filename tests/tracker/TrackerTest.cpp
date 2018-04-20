#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

#include "AppConfig.hpp"

class TrackerTest : public ::testing::Test {

 protected:
  Tracker tracker;
  virtual void SetUp() {}

  virtual void TearDown() {
    std::remove("tests/tracker/test_speed.log");
    std::remove("tests/tracker/1.jpg");
  }
};

TEST_F(TrackerTest, match_current_frame_to_existing_blobs) {

}

TEST_F(TrackerTest, add_new_blob) {
  std::vector<Blob> existing_blobs;

  std::vector<cv::Point> contour;
  contour.emplace_back(cv::Point(381, 145));
  contour.emplace_back(cv::Point(380, 143));
  contour.emplace_back(cv::Point(379, 140));
  contour.emplace_back(cv::Point(380, 141));
  contour.emplace_back(cv::Point(381, 143));
  Blob test_blob(contour);
  tracker.add_new_blob(test_blob, existing_blobs);
  ASSERT_TRUE(existing_blobs.size() == 1);

  tracker.add_new_blob(test_blob, existing_blobs);
  ASSERT_TRUE(existing_blobs.size() == 2);
}

TEST_F(TrackerTest, add_blob_to_existing_blobs) {

  std::vector<Blob> existing_blobs;

  std::vector<cv::Point> contour;
  contour.emplace_back(cv::Point(381, 145));
  contour.emplace_back(cv::Point(380, 143));
  contour.emplace_back(cv::Point(379, 140));
  contour.emplace_back(cv::Point(380, 141));
  contour.emplace_back(cv::Point(381, 143));
  Blob test_blob(contour);
  tracker.add_new_blob(test_blob, existing_blobs);
  tracker.add_new_blob(test_blob, existing_blobs);

  int blob_idx = 0;
  tracker.add_blob_to_existing_blobs(test_blob, existing_blobs, blob_idx);

  blob_idx = 1;
  tracker.add_blob_to_existing_blobs(test_blob, existing_blobs, blob_idx);

  ASSERT_EQ(existing_blobs.at(0).currentContour, test_blob.currentContour);
  ASSERT_EQ(existing_blobs.at(1).currentContour, test_blob.currentContour);
}

TEST_F(TrackerTest, distance_between_points) {
  cv::Point p1 = cv::Point(150, 150);
  cv::Point p2 = cv::Point(100, 100);

  double EXPECTED_VALUE = 70.710678118654755;
  double CALCULATED_VALUE = tracker.distance_between_points(p1, p2);
  ASSERT_DOUBLE_EQ(CALCULATED_VALUE, EXPECTED_VALUE);

}

TEST_F(TrackerTest, calculate_speed) {
  tracker.set_fps(30);
  std::vector<cv::Point> contour;
  contour.emplace_back(cv::Point(381, 145));
  contour.emplace_back(cv::Point(380, 143));
  contour.emplace_back(cv::Point(379, 140));
  contour.emplace_back(cv::Point(380, 141));
  contour.emplace_back(cv::Point(381, 143));
  Blob test_blob(contour);
  test_blob.start_frame = 10;
  test_blob.end_frame = 130;
  test_blob.start_dist = 381;
  test_blob.currentBoundingRect.x = 600;
  test_blob.currentBoundingRect.width = 60;

  tracker.calculate_speed(test_blob, 15);

  const double TRUE_SPEED = 16.74;
  ASSERT_DOUBLE_EQ(test_blob.speed, TRUE_SPEED);
}

TEST_F(TrackerTest, track_car_speed) {

}

TEST_F(TrackerTest, write_tracked_car_image) {
  cv::Mat frame = cv::Mat::zeros(100, 100, CV_8U);
  cv::Rect rectangle(5, 5, 5, 5);
  tracker.write_tracked_car_image(frame, rectangle, 1, "tests/tracker/");
  cv::Mat EXPECTED_FRAME = cv::imread("tests/tracker/1.jpg");
  frame = frame(cv::Rect(rectangle));

  ASSERT_TRUE(frame.rows == EXPECTED_FRAME.rows);
  ASSERT_TRUE(frame.cols == EXPECTED_FRAME.cols);

}

TEST_F(TrackerTest, write_tracked_car_speed) {
  tracker.write_tracked_car_speed(60, 1, "tests/tracker/test_speed.log");
  std::ifstream in_file("tests/tracker/test_speed.log");
  std::string line;
  std::getline(in_file, line);
  ASSERT_EQ(line, "1 60");
}

TEST_F(TrackerTest, check_if_blob_crossed_line) {
  std::vector<cv::Point> contour;
  contour.emplace_back(cv::Point(381, 145));
  contour.emplace_back(cv::Point(380, 143));
  contour.emplace_back(cv::Point(379, 140));
  contour.emplace_back(cv::Point(380, 141));
  contour.emplace_back(cv::Point(381, 143));
  Blob test_blob(contour);

  test_blob.centerPositions.emplace_back(640);
  test_blob.centerPositions.emplace_back(100);
  std::vector<Blob> blobs;
  blobs.emplace_back(test_blob);
  ASSERT_TRUE(tracker.blob_crossed_line(blobs, 400));
}

TEST_F(TrackerTest, number_of_cars_counted) {
  std::vector<cv::Point> crossing_lines;
  std::vector<cv::Point> start_points;
  std::vector<cv::Point> end_points;
  std::string video_path = "data/car_only.mp4";
  double fps = 30.0;
  int frame_width = 640;
  int frame_height = 480;
  int calibration_region_area = 4;
  Tracker tracker;
  BackgroundSubtractor bgs;
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
  const int TRUE_NUM_CARS = 1;
  app.run();

  ASSERT_EQ(app.get_tracker().get_car_count(), TRUE_NUM_CARS);
}