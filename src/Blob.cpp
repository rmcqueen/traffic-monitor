/**
 * Copyright: Chris Dahms
 * Note: Some modifications have been made to this class. These modifications are from lines 17 -> 23 and line 97.
 *
 * Blob.cpp
 *
 * This class is used to store each potential detected object within a frame. This allows for prediction of the movement
 * it will make for the next frame and enable tracking of when it was first and last seen on a frame; effetively allowing
 * for an estimation of the blob's speed to be performed.
 */
#include "Blob.hpp"

Blob::Blob(std::vector<cv::Point> _contour) {
  currentContour = _contour;
  currentBoundingRect = cv::boundingRect(currentContour);
  cv::Point currentCenter;
  currentCenter.x = (currentBoundingRect.x + currentBoundingRect.x + currentBoundingRect.width) / 2;
  currentCenter.y = (currentBoundingRect.y + currentBoundingRect.y + currentBoundingRect.height) / 2;
  centerPositions.push_back(currentCenter);
  blnStillBeingTracked = true;
  dblCurrentDiagonalSize = sqrt(pow(currentBoundingRect.width, 2) + pow(currentBoundingRect.height, 2));
  blnCurrentMatchFoundOrNewBlob = true;
  start_frame = 0;
  end_frame = 0;
  speed = 0.0;
  tracking_speed = false;
  intNumOfConsecutiveFramesWithoutAMatch = 0;
  id = 0;
  moving_left = false;
}

// This should be refactored to something like a Kalman Filter
void Blob::predict_next_position(void) {

  int numPositions = (int) centerPositions.size();

  if (numPositions == 1) {

    predictedNextPosition.x = centerPositions.back().x;
    predictedNextPosition.y = centerPositions.back().y;

  } else if (numPositions == 2) {

    int deltaX = centerPositions[1].x - centerPositions[0].x;
    int deltaY = centerPositions[1].y - centerPositions[0].y;

    predictedNextPosition.x = centerPositions.back().x + deltaX;
    predictedNextPosition.y = centerPositions.back().y + deltaY;

  } else if (numPositions == 3) {

    int sumOfXChanges = ((centerPositions[2].x - centerPositions[1].x) * 2) +
        ((centerPositions[1].x - centerPositions[0].x) * 1);

    int deltaX = (int) std::round((float) sumOfXChanges / 3.0);

    int sumOfYChanges = ((centerPositions[2].y - centerPositions[1].y) * 2) +
        ((centerPositions[1].y - centerPositions[0].y) * 1);

    int deltaY = (int) std::round((float) sumOfYChanges / 3.0);

    predictedNextPosition.x = centerPositions.back().x + deltaX;
    predictedNextPosition.y = centerPositions.back().y + deltaY;

  } else if (numPositions == 4) {

    int sumOfXChanges = ((centerPositions[3].x - centerPositions[2].x) * 3) +
        ((centerPositions[2].x - centerPositions[1].x) * 2) +
        ((centerPositions[1].x - centerPositions[0].x) * 1);

    int deltaX = (int) std::round((float) sumOfXChanges / 6.0);

    int sumOfYChanges = ((centerPositions[3].y - centerPositions[2].y) * 3) +
        ((centerPositions[2].y - centerPositions[1].y) * 2) +
        ((centerPositions[1].y - centerPositions[0].y) * 1);

    int deltaY = (int) std::round((float) sumOfYChanges / 6.0);

    predictedNextPosition.x = centerPositions.back().x + deltaX;
    predictedNextPosition.y = centerPositions.back().y + deltaY;

  } else if (numPositions >= 5) {

    int sumOfXChanges = ((centerPositions[numPositions - 1].x - centerPositions[numPositions - 2].x) * 4) +
        ((centerPositions[numPositions - 2].x - centerPositions[numPositions - 3].x) * 3) +
        ((centerPositions[numPositions - 3].x - centerPositions[numPositions - 4].x) * 2) +
        ((centerPositions[numPositions - 4].x - centerPositions[numPositions - 5].x) * 1);

    int deltaX = (int) std::round((float) sumOfXChanges / 10.0);

    int sumOfYChanges = ((centerPositions[numPositions - 1].y - centerPositions[numPositions - 2].y) * 4) +
        ((centerPositions[numPositions - 2].y - centerPositions[numPositions - 3].y) * 3) +
        ((centerPositions[numPositions - 3].y - centerPositions[numPositions - 4].y) * 2) +
        ((centerPositions[numPositions - 4].y - centerPositions[numPositions - 5].y) * 1);

    int deltaY = (int) std::round((float) sumOfYChanges / 10.0);

    predictedNextPosition.x = centerPositions.back().x + deltaX;
    predictedNextPosition.y = centerPositions.back().y + deltaY;

  }

  // Want this to be the front value otherwise incorrect predictions can be made due to noise on the blob detection.
  // Enables the correct direction to be predicted for each frame.
  moving_left = predictedNextPosition.x - centerPositions.front().x <= 0;
}
