/**
 * Tracker.cpp
 *
 * This class handles all of the functionality related to continually tracking a detected blob.
 */

#include <iostream>
#include <fstream>

#include "Tracker.hpp"

Tracker::Tracker() = default;

Tracker::Tracker(unsigned int car_count_, cv::Mat &frame1_, cv::Mat &frame2_, std::vector<Blob> &blobs_, const double fps_)
    : car_count(car_count_),
      frame1(frame1_),
      frame2(frame2_),
      blobs(blobs_),
      fps(fps_) {};

void Tracker::set_car_count(const unsigned int &car_count_) {
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

void Tracker::set_fps(const double &fps_) {
  fps = fps_;
}

const unsigned int &Tracker::get_car_count() const {
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

const double &Tracker::get_fps() {
  return fps;
};

// Copyright: Chris Dahms
/**
 * Map existing blobs to the current frame. Necessary to identify unique and reoccurring bloba in a frame.
 * @param existingBlobs std::vector<Blob>   contains all of the blobs which have currently been seen
 * @param currentFrameBlobs std::vector<Blob>   contains all of the blobs detected for the current frame
 */
void Tracker::match_current_frame_to_existing_blobs(std::vector<Blob> &existingBlobs,
                                                    std::vector<Blob> &currentFrameBlobs) {
  for (Blob &existingBlob : existingBlobs) {
    existingBlob.blnCurrentMatchFoundOrNewBlob = false;
    existingBlob.predict_next_position();
  }

  for (Blob &currentFrameBlob : currentFrameBlobs) {
    int intIndexOfLeastDistance = 0;
    double dblLeastDistance = 100000.0;
    for (unsigned int i = 0; i < existingBlobs.size(); i++) {
      if (existingBlobs[i].blnStillBeingTracked) {
        double dblDistance =
            distance_between_points(currentFrameBlob.centerPositions.back(), existingBlobs[i].predictedNextPosition);
        if (dblDistance < dblLeastDistance) {
          dblLeastDistance = dblDistance;
          intIndexOfLeastDistance = i;
        }
      }
    }

    if (dblLeastDistance < currentFrameBlob.dblCurrentDiagonalSize * 0.5) {
      add_blob_to_existing_blobs(currentFrameBlob, existingBlobs, intIndexOfLeastDistance);
    } else {
      add_new_blob(currentFrameBlob, existingBlobs);
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

// Copyright: Chris Dahms
/**
 * Map a blob on the current frame to an existing blob
 * @param currentFrameBlob Blob   blob object of the current frame to map
 * @param existingBlobs std::vector<Blob>   blobs which currently have been seen
 * @param intIndex int      the index of the existing blob
 */
void Tracker::add_blob_to_existing_blobs(Blob &currentFrameBlob, std::vector<Blob> &existingBlobs, int &intIndex) {
  existingBlobs[intIndex].currentContour = currentFrameBlob.currentContour;
  existingBlobs[intIndex].currentBoundingRect = currentFrameBlob.currentBoundingRect;
  existingBlobs[intIndex].centerPositions.push_back(currentFrameBlob.centerPositions.back());
  existingBlobs[intIndex].dblCurrentDiagonalSize = currentFrameBlob.dblCurrentDiagonalSize;
  existingBlobs[intIndex].blnStillBeingTracked = true;
  existingBlobs[intIndex].blnCurrentMatchFoundOrNewBlob = true;
}

// Copyright: Chris Dahms
/**
 * Adds a new blob to the existing blobs vector
 * @param currentFrameBlob Blob     the blob to add from the current frame
 * @param existingBlobs std::vector<Blob>       blobs which have currently been seen
 */
void Tracker::add_new_blob(Blob &currentFrameBlob, std::vector<Blob> &existingBlobs) {

  currentFrameBlob.blnCurrentMatchFoundOrNewBlob = true;

  existingBlobs.push_back(currentFrameBlob);
}

// Copyright: Chris Dahms
/**
 * Computes the euclidian distance between two coordinates
 * @param point1 cv::Point  holds (x,y) coordinates
 * @param point2 cv::Point  holds (x,y) coordinates
 * @return Euclidian distance between the two given points
 */
double Tracker::distance_between_points(cv::Point point1, cv::Point point2) {

  int intX = abs(point1.x - point2.x);
  int intY = abs(point1.y - point2.y);

  return (sqrt(pow(intX, 2) + pow(intY, 2)));
}

// Copyright: Chris Dahms
void Tracker::draw_and_show_contours(cv::Size imageSize,
                                     std::vector<std::vector<cv::Point> > contours,
                                     std::string strImageName) {
  cv::Mat image(imageSize, CV_8UC3, SCALAR_BLACK);

  cv::drawContours(image, contours, -1, SCALAR_WHITE, -1);

  cv::imshow(strImageName, image);
}

// Copyright: Chris Dahms
void Tracker::draw_and_show_contours(cv::Size imageSize, std::vector<Blob> blobs, std::string strImageName) {

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

/**
 * Computes the speed of a blob in kilometers per hour.
 * @param blob Blob     blob object to compute the speed of
 * @param conversion    ratio between pixels to real world meters. See AppConfig.cpp for how this is computed
 */
void Tracker::calculate_speed(Blob &blob, double conversion) {
  blob.end_dist = blob.currentBoundingRect.x + blob.currentBoundingRect.width;

  // Compute the absolute distance value and divide it by the pixel to meters ratio to get
  // the true distance in meters
  double dist = cv::norm(blob.end_dist - blob.start_dist) / conversion;

  // Get the time by dividing the difference in frames by the FPS of the video feed
  // Cannot use a simple clock() approach as this does not accurately represent the time
  // it truly took for the blobs to cross the calibration area.
  double time = (blob.end_frame - blob.start_frame) / get_fps();
  double speed = dist / time;

  // Convert the speed from meters per second to kilometers per hour
  blob.speed = speed * 3.6;
}

// This needs to be refactored into a more elegant solution
/**
 * Tracks a car travelling both left and right within a frame.
 * @param blobs std::vector<Blob>   vector of the blobs to track
 * @param start_point std::vector<cv::Point>    vector containing (x,y) coordinates for the start points for the
 * calibration region
 * @param end_point std::vector<cv::Point>    vector containing (x,y) coordinates for the start points for the
 * calibration region
 * @param conversion    ratio between pixels to real world meters. See AppConfig.cpp for how this is computed
 * @param frame_count int   number of frames currently viewed. Need this to calculate the time a blob has taken to pass a
 * calibration region.
 */
void Tracker::track_car_speed(std::vector<Blob> &blobs,
                              std::vector<cv::Point> &start_point,
                              std::vector<cv::Point> &end_point,
                              double conversion,
                              unsigned int frame_count) {
  int start_x;
  int finish_x;

  for (Blob &blob : blobs) {
    if (blob.blnStillBeingTracked && blob.centerPositions.size() >= 2) {
      // See if the blob is moving left or right and adjust the start/end points appropriately
      if (blob.moving_left) {
        start_x = start_point.at(0).x;
        finish_x = end_point.at(0).x;
      } else {
        start_x = end_point.at(0).x;
        finish_x = start_point.at(0).x;
      }

      // The car is beginning to reach the calibrated region
      if (blob.currentBoundingRect.x >= start_x && !blob.tracking_speed && blob.moving_left) {
        blob.start_frame = frame_count;
        blob.tracking_speed = true;
        blob.start_dist = blob.currentBoundingRect.x;
      }
      else if (!blob.moving_left && blob.currentBoundingRect.x <= start_x && !blob.tracking_speed) {
        blob.start_frame = frame_count;
        blob.tracking_speed = true;
        blob.start_dist = blob.currentBoundingRect.x;
      }


      // The car has passed the calibrated region heading left
      if (blob.tracking_speed &&
          blob.moving_left &&
          (blob.currentBoundingRect.x + blob.currentBoundingRect.width) <= finish_x) {
        blob.end_frame = frame_count;
        calculate_speed(blob, conversion);
        blob.tracking_speed = false;
        write_tracked_car_speed(blob.speed, blob.id, "data/tracked_cars/speed.log");
        write_tracked_car_image(get_frame1(), blob.currentBoundingRect, blob.id, "data/tracked_cars/");
      }
      // The car has passed the calibration region heading right
      else if (blob.tracking_speed &&
          !blob.moving_left &&
          blob.currentBoundingRect.x >= finish_x) {
        blob.end_frame = frame_count;
        calculate_speed(blob, conversion);
        blob.tracking_speed = false;
        write_tracked_car_speed(blob.speed, blob.id, "data/tracked_cars/speed.log");
        write_tracked_car_image(get_frame1(), blob.currentBoundingRect, blob.id, "data/tracked_cars/");
      }
    }
  }
}

/**
 * Writes a tracked car image to disk for later viewing.
 * @param frame cv::Mat     the frame to write
 * @param bounding_rectangle cv::Rect   the dimensions in which the vehicle is contained on the frame. This is used to
 * crop only the car in the frame as we do not need to save an entire frame to disk.
 * @param car_id int    ID of the blob to use for cross-referencing between speed/images
 * @param file_path std::string     path to save the image
 */
void Tracker::write_tracked_car_image(const cv::Mat &frame,
                                      cv::Rect bounding_rectangle,
                                      unsigned int car_id,
                                      std::string file_path) {
  cv::Mat tracked_car = frame(cv::Rect(bounding_rectangle));
  std::string path = file_path + std::to_string(car_id) + ".jpg";
  cv::imwrite(path, tracked_car);

}

/**
 * Writes a tracked car's speed to disk for later analysis
 * @param speed double  the speed at which the car has been travelling in kilometers per hour
 * @param blob_id int   ID of the blob to use for cross-referencing between speed/images
 * @param file_path std::string     path to save the image
 */
void Tracker::write_tracked_car_speed(double speed, int blob_id, std::string file_path) {
  std::ofstream speed_file(file_path, std::ios_base::app | std::ios_base::out);
  speed_file << blob_id << " " << speed << "\n";
}

/**
 * Check whether or not the blob has passed the calibration region. Used for counting how many vehicles have been detected
 * @param blobs std::vector<Blob>   vector containing all of the blobs to check
 * @param x_line_pos int    the position to use as a reference point for whether or not a vehicle has passed it
 * @return bool indicating whether or not a vehicle passed the line
 */
bool Tracker::blob_crossed_line(std::vector<Blob> &blobs,
                                int x_line_pos) {
  bool min_one_blob_passed = false;
  for (Blob &blob : blobs) {

    if (blob.blnStillBeingTracked && blob.centerPositions.size() >= 2) {
      int prevFrameIndex = (int) blob.centerPositions.size() - 2;
      int currFrameIndex = (int) blob.centerPositions.size() - 1;

      if (blob.centerPositions[prevFrameIndex].x > x_line_pos
          && blob.centerPositions[currFrameIndex].x <= x_line_pos) {
        set_car_count(get_car_count() + 1);
        blob.id = get_car_count();
        min_one_blob_passed = true;
      }
      else if (blob.centerPositions[prevFrameIndex].x <= x_line_pos
          && blob.centerPositions[currFrameIndex].x >= x_line_pos) {
        set_car_count(get_car_count() + 1);
        blob.id = get_car_count();
        min_one_blob_passed = true;
      }
    }
  }
  return min_one_blob_passed;
}

void Tracker::draw_blob_info_on_image(std::vector<Blob> &blobs, cv::Mat &imgFrame2Copy) {
  for (unsigned int i = 0; i < blobs.size(); i++) {
    if (blobs[i].blnStillBeingTracked) {
      cv::rectangle(imgFrame2Copy, blobs[i].currentBoundingRect, SCALAR_BLACK, 2);
    }
  }
}

void Tracker::draw_car_count_on_image(const int &carCount, cv::Mat &imgFrame2Copy) {
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
