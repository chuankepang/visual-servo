#include <chrono>
#include <functional>
#include <memory>
#include <string>
    
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_geometry/pinhole_camera_model.h>
#include <camera_info_manager/camera_info_manager.h>

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco/charuco.hpp>


int encoding2mat_type(const std::string & encoding)
{
  if (encoding == "mono8") {
    return CV_8UC1;
  } else if (encoding == "bgr8") {
    return CV_8UC3;
  } else if (encoding == "mono16") {
    return CV_16SC1;
  } else if (encoding == "rgba8") {
    return CV_8UC4;
  }
  throw std::runtime_error("Unsupported mat type");
}

std::string mat_type2encoding(int mat_type)
{
  switch (mat_type) {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("Unsupported encoding type");
  }
}

void set_now(builtin_interfaces::msg::Time & time)
{
  std::chrono::nanoseconds now = std::chrono::high_resolution_clock::now().time_since_epoch();
  if (now <= std::chrono::nanoseconds(0)) {
    time.sec = time.nanosec = 0;
  } else {
    time.sec = static_cast<builtin_interfaces::msg::Time::_sec_type>(now.count() / 1000000000);
    time.nanosec = now.count() % 1000000000;
  }
}

void draw_on_image(cv::Mat & image, const std::string & text, int height)
{
  cv::Mat c_mat = image;
  cv::putText(
    c_mat,
    text.c_str(),
    cv::Point(10, height),
    cv::FONT_HERSHEY_SIMPLEX,
    0.3,
    cv::Scalar(0, 255, 0));
}