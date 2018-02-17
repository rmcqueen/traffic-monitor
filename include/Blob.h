// Blob.h

#ifndef MY_BLOB
#define MY_BLOB

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

class Blob {
 public:
  std::vector<cv::Point> currentContour;

  cv::Rect currentBoundingRect;

  std::vector<cv::Point> centerPositions;

  double dblCurrentDiagonalSize;
  double dblCurrentAspectRatio;

  bool blnCurrentMatchFoundOrNewBlob;

  bool blnStillBeingTracked;

  int intNumOfConsecutiveFramesWithoutAMatch;

  cv::Point predictedNextPosition;

  explicit Blob(std::vector<cv::Point> _contour);
  void predictNextPosition();

};

#endif    // MY_BLOB
