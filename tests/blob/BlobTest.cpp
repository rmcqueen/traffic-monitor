#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

#include "Blob.hpp"

TEST(BlobTest, predict_next_position) {
  std::vector<cv::Point> contour;
  contour.emplace_back(cv::Point(381, 145));
  contour.emplace_back(cv::Point(380, 143));
  contour.emplace_back(cv::Point(379, 140));
  contour.emplace_back(cv::Point(380, 141));
  contour.emplace_back(cv::Point(381, 143));

  Blob blob(contour);
  blob.predict_next_position();
  ASSERT_EQ(blob.predictedNextPosition.x, 380);
  ASSERT_EQ(blob.predictedNextPosition.y, 143);
}


