#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "AppConfig.hpp"

using namespace std;

class BlobTest : public ::testing::Test {

 private:
  int frame_count = 0;
  double fps = 0;

 protected:
  cv::VideoCapture capVideo;
  virtual void SetUp() {
    capVideo.open("../data/CarsDrivingUnderBridge.mp4");
    frame_count = (int) capVideo.get(CV_CAP_PROP_FRAME_COUNT);
    fps = capVideo.get(CV_CAP_PROP_FPS);
  }
  virtual void TearDown() {
  }
};

TEST_F(BlobTest, matchCurrentFrameBlobsToExistingBlobs) {

}

TEST_F(BlobTest, addBlobToExistingBlobs) {

}

TEST_F(BlobTest, addNewBlob) {

}

TEST_F(BlobTest, distanceBetweenPoints) {

}

TEST_F(BlobTest, checkIfBLobsCrossedTheLine) {

}

/*
 * This test takes 33 seconds to perform as it must iterate over the entire video sequence
 * TODO: Refactor to instead test specific intervals of the video instead
 */
TEST_F(BlobTest, NumberOfCarsCounted) {
  AppConfig app;
  Tracker tracker;
  app.setup(tracker, "../data/CarsDrivingUnderBridge.mp4");
  ASSERT_EQ(52, tracker.get_car_count());

}
