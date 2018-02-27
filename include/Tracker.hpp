#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "Blob.hpp"

class Tracker {
 private:
  int car_count;
  cv::Mat frame1;
  cv::Mat frame2;
  std::vector<Blob> blobs;

 public:
  const cv::Scalar SCALAR_BLACK = cv::Scalar(0.0, 0.0, 0.0);
  const cv::Scalar SCALAR_WHITE = cv::Scalar(255.0, 255.0, 255.0);
  const cv::Scalar SCALAR_YELLOW = cv::Scalar(0.0, 255.0, 255.0);
  const cv::Scalar SCALAR_GREEN = cv::Scalar(0.0, 200.0, 0.0);
  const cv::Scalar SCALAR_RED = cv::Scalar(0.0, 0.0, 255.0);

  Tracker();
  Tracker(int car_count_, cv::Mat frame1, cv::Mat frame2, std::vector<Blob> blobs);
  void set_car_count(const int &car_count_);
  void set_frame1(const cv::Mat &frame1_);
  void set_frame2(const cv::Mat &frame2_);
  void set_blobs(const std::vector<Blob> &blobs_);
  const int &get_car_count();
  const cv::Mat &get_frame1();
  const cv::Mat &get_frame2();
  const std::vector<Blob> &get_blobs();
  void matchCurrentFrameBlobsToExistingBlobs(std::vector<Blob> &existingBlobs, std::vector<Blob> &currentFrameBlobs);
  void addBlobToExistingBlobs(Blob &currentFrameBlob, std::vector<Blob> &existingBlobs, int &intIndex);
  void addNewBlob(Blob &currentFrameBlob, std::vector<Blob> &existingBlobs);
  double distanceBetweenPoints(cv::Point point1, cv::Point point2);
  void drawAndShowContours(cv::Size imageSize, std::vector<std::vector<cv::Point> > contours, std::string strImageName);
  void drawAndShowContours(cv::Size imageSize, std::vector<Blob> blobs, std::string strImageName);
  bool checkIfBlobsCrossedTheLine(std::vector<Blob> &blobs, int &intHorizontalLinePosition, const int &carCount);
  void drawBlobInfoOnImage(std::vector<Blob> &blobs, cv::Mat &imgFrame2Copy);
  void drawCarCountOnImage(const int &carCount, cv::Mat &imgFrame2Copy);
};