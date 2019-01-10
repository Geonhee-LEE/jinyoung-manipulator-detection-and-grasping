#ifndef CAMERA_NODE_HPP
#define CAMERA_NODE_HPP
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/video/tracking.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "iostream"
#include "vector"
#include "thread"
#include <boost/thread/thread.hpp>
#include "QFile"
#include "QTextStream"
#include "QString"
#include "QDateTime"

class camera{

private:


public:

  int Object_u, Object_v;
  int previous_u, previous_v;
  int NKalman_Object_u, NKalman_Object_v;
  float Object_angle;
  float previous_angle;
  int distance_u, distance_v;
  double dist_pixel_u, dist_pixel_v, calculate_x, calculate_y;
  double _current_x, _current_y, _current_angle, enable_gripper, enable_depth;
  double NKalman_calculate_x, NKalman_calculate_y;
  cv::Point statePt;
  cv::Point measPt;
  void multi_publish();







};
#endif // CAMERA_NODE_HPP

