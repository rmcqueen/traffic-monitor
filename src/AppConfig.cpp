/**
 * Portions of this code have been borrowed from https://github.com/MicrocontrollersAndMore/OpenCV_3_Car_Counting_Cpp
 * I hold no copyright over the original author's work.
 *
 *
 * Example usage of a bird's eye view transformed frame:
 * Define the points to use (these are typically the areas of interest as they will be warped to match the max width and
 * height of the frame)
 *   cv::Point2f(571, 16)
 *    cv::Point2f(803, 23)
 *   cv::Point2f(1266, 531)
 *   cv::Point2f(1, 667)
 *
 *  transformer.compute_birds_eye_view(transformer.get_src_vec(),
                                       transformer.get_dst_vec(),
                                       img_frame,
                                       img_warped_copy);
 *

 *  calib_rect.emplace_back(cv::Point2f(429, 194));
 *  calib_rect.emplace_back(cv::Point2f(945, 208));
 *  calib_rect.emplace_back(cv::Point2f(1206, 474));
 *  calib_rect.emplace_back(cv::Point2f(211, 436));
 *  calib_rect = transformer.transform_calibration_rectangle(calib_rect);

 */
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>


#include "Blob.hpp"
#include "Transform.hpp"
#include "AppConfig.hpp"

/**
 * Default constructor for AppConfig
 */
AppConfig::AppConfig() = default;

/**
 * Constructor for AppConfig
 * @param tracker_ Tracking.cpp class to use for tracking blobs
 * @param bgs_ BackgroundSubtractor.cpp class to use for background subtraction. Although it has costly operations, it
 * is essential to removing shadows and ignoring minor environment movements.
 * @param crossing_lines_ std::vector<cv::Point> of pre-defined points to use for identifying if a blob should be counted
 * as tracked
 * @param start_points_ std::vector<cv::Point> a vector of coordinates where the blob will begin to get tracked
 * @param end_points_ std::vector<cv::Point> a vector of coordinates where the blob will finish getting tracked
 * @param video_path_ std::string optional relative file path if running on a saved video
 * @param fps_ double of the FPS to use for the recording (max 30 FPS)
 * @param frame_width_ int width of the frame
 * @param frame_height_ int height of the frame
 * @param calibration_region_area_ how much of the frame the calibration region takes up. For example, if it occupies half
 * of the frame, this value should be set to 2.
 */
AppConfig::AppConfig(Tracker &tracker_,
                     BackgroundSubtractor &bgs_,
                     std::vector<cv::Point> &crossing_lines_,
                     std::vector<cv::Point> &start_points_,
                     std::vector<cv::Point> &end_points_,
                     std::string video_path_,
                     double fps_,
                     int frame_width_,
                     int frame_height_,
                     int calibration_region_area_)
    : tracker(tracker_),
      bgs(bgs_),
      crossing_lines(crossing_lines_),
      start_points(start_points_),
      end_points(end_points_),
      FPS(fps_),
      FRAME_WIDTH(frame_width_),
      FRAME_HEIGHT(frame_height_),
      calibration_region_area(calibration_region_area_) {
  live_capture = true;

  if (!video_path_.empty()) {
    live_capture = false;
    set_SOURCE_VIDEO_PATH(video_path_);
  }

};

AppConfig::~AppConfig() = default;

const Tracker &AppConfig::get_tracker() const {
  return tracker;
}

const BackgroundSubtractor &AppConfig::get_background_subtractor() const {
  return bgs;
}

void AppConfig::set_background_subtractor(const BackgroundSubtractor &bgs) {
  AppConfig::bgs = bgs;
}

const std::vector<cv::Point> &AppConfig::get_start_points() const {
  return start_points;
}

void AppConfig::set_start_points(const std::vector<cv::Point> &start_points) {
  AppConfig::start_points = start_points;
}

const std::vector<cv::Point> &AppConfig::get_end_points() const {
  return end_points;
}

void AppConfig::set_end_points(const std::vector<cv::Point> &end_points) {
  AppConfig::end_points = end_points;
}

const double &AppConfig::get_FPS() const {
  return FPS;
}

void AppConfig::set_FPS(const double FPS_) {
  AppConfig::FPS = FPS_;
}

const int &AppConfig::get_FRAME_WIDTH() const {
  return FRAME_WIDTH;
}

void AppConfig::set_FRAME_WIDTH(const int WIDTH_) {
  FRAME_WIDTH = WIDTH_;
}

const int &AppConfig::get_FRAME_HEIGHT() const {
  return FRAME_HEIGHT;
}
void AppConfig::set_FRAME_HEIGHT(const int HEIGHT_) {
  FRAME_HEIGHT = HEIGHT_;
}

const std::string &AppConfig::get_SOURCE_VIDEO_PATH() const {
  return SOURCE_VIDEO_PATH;
}

void AppConfig::set_SOURCE_VIDEO_PATH(const std::string &PATH_) {
  AppConfig::SOURCE_VIDEO_PATH = PATH_;
}

const int &AppConfig::get_calibration_region_area() const {
  return calibration_region_area;
}

/**
 * Start the application with all necessary configurations defined within main.cpp
 */
void AppConfig::run() {
  cv::Mat img_frame_1;
  cv::VideoCapture capVideo;
  cv::VideoWriter out_video("output.h264", CV_FOURCC('H', '2', '6', '4'), 30, cv::Size(640, 480));
  if (live_capture) {
    capVideo.open(0);
  } else {
    capVideo.open(get_SOURCE_VIDEO_PATH());
  }
  unsigned int frame_count = 0;

  std::vector<Blob> blobs;
  if (!capVideo.isOpened()) {
    std::cerr <<"Error opening the camera"<<std::endl;
    return;
  }

  capVideo.set(CV_CAP_PROP_FPS, get_FPS());
  capVideo.set(CV_CAP_PROP_FRAME_WIDTH, get_FRAME_WIDTH());
  capVideo.set(CV_CAP_PROP_FRAME_HEIGHT, get_FRAME_HEIGHT());

  tracker.set_car_count(0);
  tracker.set_fps(get_FPS());

  // Make sure the webcam was able to be opened
  if (!capVideo.isOpened()) {
    printf("Error opening video.");
    return;
  }

  char chCheckForEscKey = 0;
  bool blnFirstFrame = true;

  /*
   * These values must be measured in the real world and inputted for the region of interest. This is relied upon to
   * calculate the true distance an object travels in order to enhance the accuracy at which the speed is calculated.
   * An approximation for the distance can be made, but note the speeds detected will most likely be incorrect.
  */
  const double real_height = 8.2169;
  const double real_width = 9.8425;
  const double aspect_ratio = real_width / real_height;

  // Determine the ratio of pixels : meters to allow for accurate speed measurements.
  const double pixel_calibration_height = get_FRAME_WIDTH() / (get_calibration_region_area() * aspect_ratio);
  double pixels_to_meters = pixel_calibration_height / real_height;

  // Define the coordinates which will be used as the starting "line" for the calibration region
  std::vector<cv::Point> start_points;
  start_points.emplace_back(cv::Point(211, 436));
  start_points.emplace_back(cv::Point(442, 286));

  // Define the coordinates which will be used as the ending "line" for the calibration region
  std::vector<cv::Point> end_points;
  end_points.emplace_back(cv::Point(304, 247));
  end_points.emplace_back(cv::Point(309, 294));

  //  These points correspond to the bird's eye view start points
//  start_points.emplace_back(cv::Point2f(9, 660));
//  start_points.emplace_back(cv::Point2f(1269, 700));
  // These points correspond to the bird's eye view end points
//  end_points.emplace_back(cv::Point2f(1213, 527));
//  end_points.emplace_back(cv::Point2f(29, 502));

  capVideo.read(img_frame_1);

  while (capVideo.isOpened() && chCheckForEscKey != 27) {
    if (img_frame_1.empty()) {
      printf("WTF");
    }
    std::vector<Blob> currentFrameBlobs;
    cv::Mat img_frame_1_copy = img_frame_1.clone();
    cv::Mat img_warped;
    cv::Mat img_warped_copy;
    cv::Mat img_thresh;

    bgs.subtract(img_frame_1_copy, img_thresh);
    tracker.set_frame1(img_frame_1_copy);

    // Initialize a transformation class pointer and create the calibration rectangle
    Transform transformer(img_frame_1_copy);
    std::vector<cv::Point2f> calib_rect;

    calib_rect.emplace_back(cv::Point2f(304, 247));
    calib_rect.emplace_back(cv::Point2f(437, 237));
    calib_rect.emplace_back(cv::Point2f(442, 286));
    calib_rect.emplace_back(cv::Point2f(309, 294));
    transformer.set_calibration_rect(calib_rect);

    // Find contours (blobs) within the frame and their associated convex hull
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(img_thresh, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    std::vector<std::vector<cv::Point>> convexHulls(contours.size());

    /* Copyright: Chris Dahms
     * For each convex hull which has been detected, determine whether or not the sizes are valid for that of a vehicle
     */
    for (unsigned int i = 0; i < contours.size(); i++) {
      cv::convexHull(contours.at(i), convexHulls.at(i));
    }

    for (std::vector<cv::Point> &convexHull : convexHulls) {
      Blob possibleBlob(convexHull);
      // These area() ranges are chosen based on trial/error
      if (possibleBlob.currentBoundingRect.area() > 1000 &&
          possibleBlob.currentBoundingRect.area() < 25000 &&
          possibleBlob.currentBoundingRect.width > 50 &&
          possibleBlob.currentBoundingRect.height > 50 &&
          (cv::contourArea(possibleBlob.currentContour) / (double) possibleBlob.currentBoundingRect.area()) > 0.50) {
        currentFrameBlobs.push_back(possibleBlob);
      }
    }

    /* Copyright: Chris Dahms
     * If this is the first frame of the video, then push all blobs to the back of the blob vector, otherwise determine
     * if they have been seen before
     */
    if (blnFirstFrame) {
      for (Blob &currentFrameBlob : currentFrameBlobs) {
        blobs.push_back(currentFrameBlob);
      }
      blnFirstFrame = false;
    } else {
      tracker.match_current_frame_to_existing_blobs(blobs, currentFrameBlobs);
    }

    tracker.blob_crossed_line(blobs, start_points.at(0).x);
    tracker.track_car_speed(blobs, start_points, end_points, pixels_to_meters, frame_count);
    tracker.draw_blob_info_on_image(blobs, img_frame_1_copy);
    tracker.draw_car_count_on_image(tracker.get_car_count(), img_frame_1_copy);
    transformer.draw_calibration_rectangle(img_frame_1_copy);

    cv::imshow("Car Tracker", img_frame_1_copy);

    // Prepare for next iteration
    currentFrameBlobs.clear();
    capVideo.read(img_frame_1);

    // If escape is pressed, close the program
    chCheckForEscKey = cv::waitKey(1);

    // Write the modified frame to disk to view later
    out_video.write(img_frame_1_copy);

    // If running a pre-existing video, make sure the while loop terminates once the video is over
    if (!live_capture && capVideo.get(CV_CAP_PROP_POS_FRAMES) == capVideo.get(CV_CAP_PROP_FRAME_COUNT)) {
      break;
    }

    frame_count++;
  }

  // Close input/output streams
  capVideo.release();
  out_video.release();
}
