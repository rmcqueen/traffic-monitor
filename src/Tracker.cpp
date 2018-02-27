/**
 * Tracker.cpp
 */

#include "Tracker.hpp"

using namespace std;

Tracker::Tracker() {};

Tracker::Tracker(const int car_count_, cv::Mat frame1_, cv::Mat frame2_, std::vector<Blob> blobs_)
    : car_count(car_count_),
      frame1(frame1_),
      frame2(frame2_),
      blobs(blobs_) {};

void Tracker::set_car_count(const int &car_count_) {
  car_count = car_count_;
}

void Tracker::set_frame1(const cv::Mat &frame1_) {
  frame1 = frame1_;
}

void Tracker::set_frame2(const cv::Mat &frame2_) {
  frame2 = frame2_;
}

void Tracker::set_blobs(const std::vector<Blob> &blobs_) {
  blobs = blobs_;
}

const int &Tracker::get_car_count() {
  return car_count;
}

const cv::Mat &Tracker::get_frame1() {
  return frame1;
}

const cv::Mat &Tracker::get_frame2() {
  return frame2;
}

const std::vector<Blob> &Tracker::get_blobs() {
  return blobs;
}

void Tracker::matchCurrentFrameBlobsToExistingBlobs(std::vector<Blob> &existingBlobs,
                                                    std::vector<Blob> &currentFrameBlobs) {
  for (Blob &existingBlob : existingBlobs) {
    existingBlob.blnCurrentMatchFoundOrNewBlob = false;
    existingBlob.predictNextPosition();
  }

  for (Blob &currentFrameBlob : currentFrameBlobs) {
    int intIndexOfLeastDistance = 0;
    double dblLeastDistance = 100000.0;
    for (unsigned int i = 0; i < existingBlobs.size(); i++) {
      if (existingBlobs[i].blnStillBeingTracked) {
        double dblDistance =
            distanceBetweenPoints(currentFrameBlob.centerPositions.back(), existingBlobs[i].predictedNextPosition);
        if (dblDistance < dblLeastDistance) {
          dblLeastDistance = dblDistance;
          intIndexOfLeastDistance = i;
        }
      }
    }

    if (dblLeastDistance < currentFrameBlob.dblCurrentDiagonalSize * 0.5) {
      addBlobToExistingBlobs(currentFrameBlob, existingBlobs, intIndexOfLeastDistance);
    } else {
      addNewBlob(currentFrameBlob, existingBlobs);
    }
  }

  for (Blob &existingBlob : existingBlobs) {
    if (!existingBlob.blnCurrentMatchFoundOrNewBlob) {
      existingBlob.intNumOfConsecutiveFramesWithoutAMatch++;
    }
    if (existingBlob.intNumOfConsecutiveFramesWithoutAMatch >= 5) {
      existingBlob.blnStillBeingTracked = false;
    }
  }
}

void Tracker::addBlobToExistingBlobs(Blob &currentFrameBlob, std::vector<Blob> &existingBlobs, int &intIndex) {
  existingBlobs[intIndex].currentContour = currentFrameBlob.currentContour;
  existingBlobs[intIndex].currentBoundingRect = currentFrameBlob.currentBoundingRect;
  existingBlobs[intIndex].centerPositions.push_back(currentFrameBlob.centerPositions.back());
  existingBlobs[intIndex].dblCurrentDiagonalSize = currentFrameBlob.dblCurrentDiagonalSize;
  existingBlobs[intIndex].dblCurrentAspectRatio = currentFrameBlob.dblCurrentAspectRatio;
  existingBlobs[intIndex].blnStillBeingTracked = true;
  existingBlobs[intIndex].blnCurrentMatchFoundOrNewBlob = true;
}

void Tracker::addNewBlob(Blob &currentFrameBlob, std::vector<Blob> &existingBlobs) {

  currentFrameBlob.blnCurrentMatchFoundOrNewBlob = true;

  existingBlobs.push_back(currentFrameBlob);
}

double Tracker::distanceBetweenPoints(cv::Point point1, cv::Point point2) {

  int intX = abs(point1.x - point2.x);
  int intY = abs(point1.y - point2.y);

  return (sqrt(pow(intX, 2) + pow(intY, 2)));
}

void Tracker::drawAndShowContours(cv::Size imageSize,
                                  std::vector<std::vector<cv::Point> > contours,
                                  std::string strImageName) {
  cv::Mat image(imageSize, CV_8UC3, SCALAR_BLACK);

  cv::drawContours(image, contours, -1, SCALAR_WHITE, -1);

  cv::imshow(strImageName, image);
}

void Tracker::drawAndShowContours(cv::Size imageSize, std::vector<Blob> blobs, std::string strImageName) {

  cv::Mat image(imageSize, CV_8UC3, SCALAR_BLACK);

  std::vector<std::vector<cv::Point> > contours;

  for (Blob &blob : blobs) {
    if (blob.blnStillBeingTracked) {
      contours.push_back(blob.currentContour);
    }
  }

  cv::drawContours(image, contours, -1, SCALAR_WHITE, -1);

  cv::imshow(strImageName, image);
}

bool Tracker::checkIfBlobsCrossedTheLine(std::vector<Blob> &blobs,
                                         int &intHorizontalLinePosition,
                                         const int &carCount) {
  bool blnAtLeastOneBlobCrossedTheLine = false;

  for (Blob blob : blobs) {

    if (blob.blnStillBeingTracked && blob.centerPositions.size() >= 2) {
      int prevFrameIndex = (int) blob.centerPositions.size() - 2;
      int currFrameIndex = (int) blob.centerPositions.size() - 1;

      if (blob.centerPositions[prevFrameIndex].y > intHorizontalLinePosition
          && blob.centerPositions[currFrameIndex].y <= intHorizontalLinePosition) {
        set_car_count(get_car_count() + 1);
        blnAtLeastOneBlobCrossedTheLine = true;
      }
    }

  }

  return blnAtLeastOneBlobCrossedTheLine;

}

void Tracker::drawBlobInfoOnImage(std::vector<Blob> &blobs, cv::Mat &imgFrame2Copy) {

  for (unsigned int i = 0; i < blobs.size(); i++) {

    if (blobs[i].blnStillBeingTracked) {
      cv::rectangle(imgFrame2Copy, blobs[i].currentBoundingRect, SCALAR_RED, 2);

      int intFontFace = CV_FONT_HERSHEY_SIMPLEX;
      double dblFontScale = blobs[i].dblCurrentDiagonalSize / 60.0;
      int intFontThickness = (int) std::round(dblFontScale * 1.0);

      cv::putText(imgFrame2Copy,
                  std::to_string(i),
                  blobs[i].centerPositions.back(),
                  intFontFace,
                  dblFontScale,
                  SCALAR_GREEN,
                  intFontThickness);
    }
  }
}

void Tracker::drawCarCountOnImage(const int &carCount, cv::Mat &imgFrame2Copy) {

  int intFontFace = CV_FONT_HERSHEY_SIMPLEX;
  double dblFontScale = (imgFrame2Copy.rows * imgFrame2Copy.cols) / 300000.0;
  int intFontThickness = (int) std::round(dblFontScale * 1.5);

  cv::Size textSize = cv::getTextSize(std::to_string(carCount), intFontFace, dblFontScale, intFontThickness, 0);

  cv::Point ptTextBottomLeftPosition;

  ptTextBottomLeftPosition.x = imgFrame2Copy.cols - 1 - (int) ((double) textSize.width * 1.25);
  ptTextBottomLeftPosition.y = (int) ((double) textSize.height * 1.25);

  cv::putText(imgFrame2Copy,
              std::to_string(carCount),
              ptTextBottomLeftPosition,
              intFontFace,
              dblFontScale,
              SCALAR_GREEN,
              intFontThickness);

}