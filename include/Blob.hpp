// Blob.h

#ifndef MY_BLOB
#define MY_BLOB

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


class Blob {
 public:
  std::vector<cv::Point> currentContour;
  cv::Rect currentBoundingRect;
  std::vector<cv::Point> centerPositions;
  bool blnCurrentMatchFoundOrNewBlob;
  bool blnStillBeingTracked;
  double dblCurrentDiagonalSize;
  int intNumOfConsecutiveFramesWithoutAMatch;
  unsigned int start_frame;
  unsigned int end_frame;
  double speed;
  bool tracking_speed;
  double start_dist;
  double end_dist;
  cv::Point predictedNextPosition;
  unsigned int id;
  bool moving_left;

  explicit Blob(std::vector<cv::Point> _contour);
  void predict_next_position();
};

#endif    // MY_BLOB
