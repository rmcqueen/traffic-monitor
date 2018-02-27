/*
 * This code has been borrowed and modified from https://github.com/MicrocontrollersAndMore/OpenCV_3_Car_Counting_Cpp
 * I hold no copyright over the original authors work.
 *
 */
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "Blob.hpp"
#include "Transform.hpp"
#include "AppConfig.hpp"

using namespace cv;

void AppConfig::setup(Tracker &tracker, std::string video_path) {
  cv::VideoCapture capVideo;

  cv::Mat imgFrame1;
  cv::Mat imgFrame2;

  std::vector<Blob> blobs;
  cv::Point crossingLine[2];
  tracker.set_car_count(0);

  capVideo.open(video_path);

  if (!capVideo.isOpened()) {
    std::cout << "error reading video file" << std::endl << std::endl;
    // _getch();                   // it may be necessary to change or remove this line if not using Windows
    return;
  }

  if (capVideo.get(CV_CAP_PROP_FRAME_COUNT) < 2) {
    std::cout << "error: video file must have at least two frames";
    // _getch();                   // it may be necessary to change or remove this line if not using Windows
    return;
  }

  capVideo.read(imgFrame1);
  capVideo.read(imgFrame2);

  int intHorizontalLinePosition = (int) std::round((double) imgFrame1.rows * 0.35);

  crossingLine[0].x = 0;
  crossingLine[0].y = intHorizontalLinePosition;

  crossingLine[1].x = imgFrame1.cols - 1;
  crossingLine[1].y = intHorizontalLinePosition;

  char chCheckForEscKey = 0;

  bool blnFirstFrame = true;

  int frameCount = 2;

  while (capVideo.isOpened() && chCheckForEscKey != 27) {
    std::vector<Blob> currentFrameBlobs;

    cv::Mat imgFrame1Copy = imgFrame1.clone();
    cv::Mat imgFrame2Copy = imgFrame2.clone();
    cv::Mat imgWarped;

    cv::Mat imgDifference;
    cv::Mat imgThresh;

    cv::cvtColor(imgFrame1Copy, imgFrame1Copy, CV_BGR2GRAY);
    cv::cvtColor(imgFrame2Copy, imgFrame2Copy, CV_BGR2GRAY);

    // Apply a 5x5 Gaussian blur window to the two frames
    cv::GaussianBlur(imgFrame1Copy, imgFrame1Copy, cv::Size(5, 5), 0);
    cv::GaussianBlur(imgFrame2Copy, imgFrame2Copy, cv::Size(5, 5), 0);

    // Find the differences between frame 1 and frame two in order to identify objects which have moved since the last
    // frame
    // TODO: Replace this with BackgroundSubtractor
    cv::absdiff(imgFrame1Copy, imgFrame2Copy, imgDifference);

    // Remove some noise by using a threshold: pixel < 30 -> 0, pixel > 30 -> 1
    cv::threshold(imgDifference, imgThresh, 30, 255.0, CV_THRESH_BINARY);

    cv::Mat structuringElement3x3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::Mat structuringElement5x5 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::Mat structuringElement7x7 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
    cv::Mat structuringElement15x15 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));

    for (unsigned int i = 0; i < 2; i++) {
      cv::dilate(imgThresh, imgThresh, structuringElement5x5);
      cv::dilate(imgThresh, imgThresh, structuringElement5x5);
      cv::erode(imgThresh, imgThresh, structuringElement5x5);
    }

    cv::Mat imgThreshCopy = imgThresh.clone();

    // Pre-selected points from testing video
    Transform transformer(imgFrame1Copy,
                          cv::Point2f(571, 16),
                          cv::Point2f(803, 23),
                          cv::Point2f(1266, 531),
                          cv::Point2f(1, 667));

    transformer.compute_birds_eye_view(transformer.get_src_vec(),
                                       transformer.get_dst_vec(),
                                       imgFrame1,
                                       imgWarped
    );

    std::vector<std::vector<cv::Point> > contours;

    cv::findContours(imgThreshCopy, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point> > convexHulls(contours.size());

    for (unsigned int i = 0; i < contours.size(); i++) {
      cv::convexHull(contours[i], convexHulls[i]);
    }

    for (std::vector<Point> &convexHull : convexHulls) {
      Blob possibleBlob(convexHull);
      if (possibleBlob.currentBoundingRect.area() > 400 &&
          possibleBlob.currentBoundingRect.width > 30 &&
          possibleBlob.currentBoundingRect.height > 30 &&
          (cv::contourArea(possibleBlob.currentContour) / (double) possibleBlob.currentBoundingRect.area()) > 0.50) {
        currentFrameBlobs.push_back(possibleBlob);
      }
    }

    if (blnFirstFrame) {
      for (Blob &currentFrameBlob : currentFrameBlobs) {
        blobs.push_back(currentFrameBlob);
      }
    } else {
      tracker.matchCurrentFrameBlobsToExistingBlobs(blobs, currentFrameBlobs);
    }

    imgFrame2Copy =
        imgFrame2.clone(); // get another copy of frame 2 since we changed the previous frame 2 copy in the processing above


    bool blnAtLeastOneBlobCrossedTheLine =
        tracker.checkIfBlobsCrossedTheLine(blobs, intHorizontalLinePosition, tracker.get_car_count());

    if (blnAtLeastOneBlobCrossedTheLine) {
      cv::line(imgFrame2Copy, crossingLine[0], crossingLine[1], tracker.SCALAR_GREEN, 2);
    } else {
      cv::line(imgFrame2Copy, crossingLine[0], crossingLine[1], tracker.SCALAR_RED, 2);
    }

    cv::imshow("stuff", imgWarped);

    // cv::waitKey(0);                 // uncomment this line to go frame by frame for debugging

    // now we prepare for the next iteration

    currentFrameBlobs.clear();

    imgFrame1 = imgFrame2.clone();           // move frame 1 up to where frame 2 is

    if ((capVideo.get(CV_CAP_PROP_POS_FRAMES) + 1) < capVideo.get(CV_CAP_PROP_FRAME_COUNT)) {
      capVideo.read(imgFrame2);
    } else {
      std::cout << "end of video\n";
      break;
    }

    blnFirstFrame = false;
    frameCount++;
    chCheckForEscKey = cv::waitKey(1);
  }

  // Hold video window open when reaching the end of the video
//    if (chCheckForEscKey != 27) {
//      cv::waitKey(0);
//    }
}