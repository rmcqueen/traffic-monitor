/**
 * Transform.cpp
 *
 * This class is used to apply a geometric transformation in order to convert 4 given coordinates to a perspective which
 * mimics a birds eye view on a frame. This transformation is useful in order to more accurately estimate the speed at
 * which an object is moving due to the process of 2-dimensional -> real world 3-dimensional -> real world 2-dimensional
 * estimation.
 */

#include "Transform.hpp"

/**
 * Constructor for just the current frame
 * @param frame a matrix composed of the relevant RGB values to make up the frame
 */
Transform::Transform(const cv::Mat &frame)
    : frame(frame) {

}

/**
 * Constructor for just the frame coordinate points
 * @param top_left_ (x,y) coordinate to the top left point in a frame
 * @param top_right_ (x,y) coordinate to the top rightpoint in a frame
 * @param bottom_right_ (x,y) coordinate to the bottom right point in a frame
 * @param bottom_left_ (x,y) coordinate to the bottom left point in a frame
 */
Transform::Transform(cv::Point2f top_left_, cv::Point2f top_right_, cv::Point2f bottom_right_, cv::Point2f bottom_left_)
    : top_left(top_left_),
      top_right(top_right_),
      bottom_right(bottom_right_),
      bottom_left(bottom_left_) {
  set_src_vec(top_left_, top_right_, bottom_right_, bottom_left_);

  // Point2f expects (col, row) instead of traditional (row, col). See OpenCV documentation for more information
  // https://docs.opencv.org/3.3.0/db/d4e/classcv_1_1Point__.html
  set_dst_vec(cv::Point2f(0, 0),
              cv::Point2f(frame.cols - 1, 0),
              cv::Point2f(frame.cols, frame.rows - 1),
              cv::Point2f(0, frame.rows - 1));
}

Transform::Transform(cv::Mat frame,
                     cv::Point2f top_left_,
                     cv::Point2f top_right_,
                     cv::Point2f bottom_right_,
                     cv::Point2f bottom_left_)
    : frame(frame),
      top_left(top_left_),
      top_right(top_right_),
      bottom_right(bottom_right_),
      bottom_left(bottom_left_) {
  set_src_vec(top_left_, top_right_, bottom_right_, bottom_left_);

  // Point2f expects (col, row) instead of traditional (row, col). See OpenCV documentation for more information
  // https://docs.opencv.org/3.3.0/db/d4e/classcv_1_1Point__.html
  set_dst_vec(cv::Point2f(0, 0),
              cv::Point2f(frame.cols - 1, 0),
              cv::Point2f(frame.cols, frame.rows - 1),
              cv::Point2f(0, frame.rows - 1));

}

/**
 * Returns the geometric point origins from a frame used for perspective warping
 * @return src_vec
 */
const std::vector<cv::Point2f> &Transform::get_src_vec() const {
  return src_vec;
}

/**
 * Sets the geometric point origins from a frame to be used for perspective warping
 * @param top_left_ (x,y) coordinate to the top left point in a frame
 * @param top_right_ (x,y) coordinate to the top right point in a frame
 * @param bottom_right_ (x,y) coordinate to the bottom right point in a frame
 * @param bottom_left_ (x,y) coordinate to the bottom left point in a frame
 */
void Transform::set_src_vec(cv::Point2f top_left_,
                            cv::Point2f top_right_,
                            cv::Point2f bottom_right_,
                            cv::Point2f bottom_left_) {
  src_vec.emplace_back(top_left_);
  src_vec.emplace_back(top_right_);
  src_vec.emplace_back(bottom_right_);
  src_vec.emplace_back(bottom_left_);
}

/**
 * Sets the geometric point origins from a frame to be used for perspective warping
 * @param src_vec
 */
void Transform::set_src_vec(const std::vector<cv::Point2f> &src_vec) {
  Transform::src_vec = src_vec;
}

/**
 * Gets the geometric point destinations which were warped
 * @return dst_vec
 */
const std::vector<cv::Point2f> &Transform::get_dst_vec() const {
  return dst_vec;
}

/**
 * Sets the geometric point destinations such that the given frame coordinates will be warped to the new, given location
 * @param dst_vec
 */
void Transform::set_dst_vec(const std::vector<cv::Point2f> &dst_vec) {
  Transform::dst_vec = dst_vec;
}

/**
 * Sets the geometric point destinations such that the given frame coordinates will be warped to the new, given location
 * i.e, top left point at (50,50) => (0,0) etc.
 * @param top_left_ (x,y) coordinate to the top left point in a new frame
 * @param top_right_ (x,y) coordinate to the top right point in a new frame
 * @param bottom_right_ (x,y) coordinate to the bottom right point in a new frame
 * @param bottom_left_ (x,y) coordinate to the bottom left point  in a new frame
 */
void Transform::set_dst_vec(cv::Point2f top_left_,
                            cv::Point2f top_right_,
                            cv::Point2f bottom_right_,
                            cv::Point2f bottom_left_) {
  dst_vec.emplace_back(top_left_);
  dst_vec.emplace_back(top_right_);
  dst_vec.emplace_back(bottom_right_);
  dst_vec.emplace_back(bottom_left_);
}

/**
 * Gets the calibration rectangle used for the transformation
 * @return calibration_rect
 */
const std::vector<cv::Point2f> &Transform::get_calibration_rect() const {
  return calibration_rect;
}

/**
 * Sets the calibration rectangle to be used for the transformation. This rectangle enables speed estimation as the true
 * distance is known between each point
 * @param calibration_rect
 */
void Transform::set_calibration_rect(const std::vector<cv::Point2f> &calibration_rect) {
  Transform::calibration_rect = calibration_rect;
}

/**
 * Computes a "bird eye view" perspective based on the selected coordinates and given frame.
 * Assumes that the dst_ vector has been initialized prior to calling this function and checks to ensure this is the case.
 * @param src_ input vector of 4 origin points
 * @param dst_ input vector of 4 destination points to be used for the transformation
 * @param frame the frame to apply the transformation to
 * @param OutputArray a cv::Matrix containing the warped frame
 */
void Transform::compute_birds_eye_view(std::vector<cv::Point2f> src_,
                                       std::vector<cv::Point2f> dst_,
                                       cv::Mat frame,
                                       cv::Mat &OutputArray) {
  assert(!src_.empty());
  assert(!dst_.empty());

  cv::Mat hom_matrix = cv::findHomography(src_, dst_);

  cv::warpPerspective(frame, OutputArray, hom_matrix, OutputArray.size());
}

/**
 * Computes a "bird eye view" perspective based on the selected coordinates and given frame.
 * Assumes that the dst_ vector has been initialized prior to calling this function and checks to ensure this is the case.
 * @param src_ input vector of 4 origin points
 * @param dst_ input vector of 4 destination points to be used for the transformation
 * @param frame the frame to apply the transformation to
 * @return cv::Matrix containss the warped frame
 */
cv::Mat Transform::compute_birds_eye_view(std::vector<cv::Point2f> src_, std::vector<cv::Point2f> dst_, cv::Mat frame) {
  assert(!src_.empty());
  assert(!dst_.empty());

  cv::Mat hom_matrix = cv::findHomography(src_, dst_);
  cv::Mat output_mat;

  cv::warpPerspective(frame, output_mat, hom_matrix, output_mat.size());

  return output_mat;
}

/**
 * Computes the tranformation points which correspond to the original calibration points
 */
const std::vector<cv::Point2f> Transform::transform_calibration_rectangle() {
  assert(!get_calibration_rect().empty());

  cv::Mat hom_matrix = cv::findHomography(get_src_vec(), get_dst_vec());
  std::vector<cv::Point2f> warped_calibration_rect;

  cv::perspectiveTransform(get_calibration_rect(), warped_calibration_rect, hom_matrix);
  return warped_calibration_rect;
}

/**
 * Computes the tranformation points which correspond to the original calibration points
 * @param calibration_rect_     a vector of cv::Point2f in the order of: [top_left, top_right, bottom_right, bottom_left]
 */
const std::vector<cv::Point2f> Transform::transform_calibration_rectangle(std::vector<cv::Point2f> &calibration_rect_) {
  assert(!calibration_rect_.empty());

  cv::Mat hom_matrix = cv::findHomography(get_src_vec(), get_dst_vec());
  std::vector<cv::Point2f> warped_calibration_rect;

  cv::perspectiveTransform(calibration_rect_, warped_calibration_rect, hom_matrix);
  return warped_calibration_rect;
}

/**
 * Draws a calibration rectangle over a given frame
 * @param frame
 */
void Transform::draw_calibration_rectangle(cv::Mat &frame) {
  cv::Scalar col = cv::Scalar(255, 0, 0);

  // Draw lines to connect each point and form a rectangle
  cv::line(frame, calibration_rect[0], calibration_rect[1], col, 1);
  cv::line(frame, calibration_rect[1], calibration_rect[2], col, 1);
  cv::line(frame, calibration_rect[2], calibration_rect[3], col, 1);
  cv::line(frame, calibration_rect[3], calibration_rect[0], col, 1);
}

/**
 * Draws a calibration rectangle over the transformed frame. See frame from transform_calibration_rectangle().
 * @param frame transformed frame to draw on
 */
void Transform::draw_transformed_calibration_rectangle(cv::Mat &frame, std::vector<cv::Point2f> &transformed_calib_rect) {
  cv::Scalar col = cv::Scalar(255, 0, 0);

  // Draw lines to connect each point and form a rectangle
  cv::line(frame, transformed_calib_rect[0], transformed_calib_rect[1], col);
  cv::line(frame, transformed_calib_rect[1], transformed_calib_rect[2], col);
  cv::line(frame, transformed_calib_rect[2], transformed_calib_rect[3], col);
  cv::line(frame, transformed_calib_rect[3], transformed_calib_rect[0], col);
}
