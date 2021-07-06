// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#include "cv_camera/capture.h"
#include <sstream>
#include <string>

#define COUNT 2

void loadCalibrationFiles(std::string& calib_path, cv::Mat& cameraMatrix, cv::Mat& distortion, double scale)
{

  cv::FileStorage fs;

  cv::Mat cameraMatrix_origin;

  calib_path = calib_path + "/calib_color.yaml";
  std::cout << calib_path << std::endl;
  if(fs.open(calib_path, cv::FileStorage::READ))
  {
    std::cout << "open!" << std::endl;
    fs["cameraMatrix"] >> cameraMatrix_origin;
    std::cout << "color matrix load success"<< std::endl;
    cameraMatrix = cameraMatrix_origin.clone();
    cameraMatrix.at<double>(0, 0) *= scale;
    cameraMatrix.at<double>(1, 1) *= scale;
    cameraMatrix.at<double>(0, 2) *= scale;
    cameraMatrix.at<double>(1, 2) *= scale;

    distortion= cv::Mat::zeros(1, 5, CV_64F);

    fs["distortionCoefficients"] >> distortion;
    std::cout << "color matrix load success"<< std::endl;
    fs.release();

  }
}

namespace cv_camera
{

namespace enc = sensor_msgs::image_encodings;

Capture::Capture(ros::NodeHandle &node, const std::string &topic_name,
                 int32_t buffer_size, const std::string &frame_id)
    : node_(node),
      it_(node_),
      topic_name_(topic_name),
      buffer_size_(buffer_size),
      frame_id_(frame_id),
      info_manager_(node_, frame_id),
      capture_delay_(ros::Duration(node_.param("capture_delay", 0.0)))
{
      status.resize(COUNT);
      cam_pubs.resize(COUNT);
      bridges.resize(COUNT);
      size_image.resize(COUNT);
      map1.resize(COUNT);
      map2.resize(COUNT);
      size_image[0] = cv::Size(1920,1080);
      size_image[1] = cv::Size(960,540);
      std::string calib_path = "/home/agent/catkin_ws/src/cv_camera/calibration_data";
      loadCalibrationFiles(calib_path, cameraMatrix, distortion, 1);
      cameraMatrix_qhd = cameraMatrix.clone();
      cameraMatrix_qhd.at<double>(0, 0) /= 2;
      cameraMatrix_qhd.at<double>(1, 1) /= 2;
      cameraMatrix_qhd.at<double>(0, 2) /= 2;
      cameraMatrix_qhd.at<double>(1, 2) /= 2;
      const int mapType = CV_16SC2;
      cv::initUndistortRectifyMap(cameraMatrix, distortion, cv::Mat(), cameraMatrix, size_image[0], mapType, map1[0], map2[0]);
      cv::initUndistortRectifyMap(cameraMatrix, distortion, cv::Mat(), cameraMatrix_qhd, size_image[1], mapType, map1[1], map2[1]);
      const double *itC = cameraMatrix.ptr<double>(0, 0);
      for(size_t i = 0; i < 9; ++i, ++itC)
      {
        double k = *itC;
        std::cout << k << std::endl;
      }
}

void Capture::updateStatus()
{
  for(int i = 0; i < COUNT; ++i)
  {
    status[i] = 0;
    if(cam_pubs[i].getNumSubscribers() > 0)
    {
      status[i] = 1;
    }
  }

}

void Capture::loadCameraInfo()
{
  std::string url;
  if (node_.getParam("camera_info_url", url))
  {
    if (info_manager_.validateURL(url))
    {
      info_manager_.loadCameraInfo(url);
    }
  }

  rescale_camera_info_ = node_.param<bool>("rescale_camera_info", false);

  for (int i = 0;; ++i)
  {
    int code = 0;
    double value = 0.0;
    std::stringstream stream;
    stream << "property_" << i << "_code";
    const std::string param_for_code = stream.str();
    stream.str("");
    stream << "property_" << i << "_value";
    const std::string param_for_value = stream.str();
    if (!node_.getParam(param_for_code, code) || !node_.getParam(param_for_value, value))
    {
      break;
    }
    if (!cap_.set(code, value))
    {
      ROS_ERROR_STREAM("Setting with code " << code << " and value " << value << " failed"
                                            << std::endl);
    }
  }
}

void Capture::rescaleCameraInfo(int width, int height)
{
  double width_coeff = static_cast<double>(width) / info_.width;
  double height_coeff = static_cast<double>(height) / info_.height;
  info_.width = width;
  info_.height = height;

  // See http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html for clarification
  info_.K[0] *= width_coeff;
  info_.K[2] *= width_coeff;
  info_.K[4] *= height_coeff;
  info_.K[5] *= height_coeff;

  info_.P[0] *= width_coeff;
  info_.P[2] *= width_coeff;
  info_.P[5] *= height_coeff;
  info_.P[6] *= height_coeff;
}

void Capture::open(int32_t device_id)
{
  cap_.open(device_id);
  if (!cap_.isOpened())
  {
    std::stringstream stream;
    stream << "device_id" << device_id << " cannot be opened";
    throw DeviceError(stream.str());
  }
  //pub_ = it_.advertiseCamera(topic_name_, buffer_size_);
  cam_pubs[0] = it_.advertiseCamera("hd/" + topic_name_, buffer_size_);
  cam_pubs[1] = it_.advertiseCamera("qhd/" + topic_name_, buffer_size_);

  loadCameraInfo();
}
/**
void Capture::open(const std::string &device_path)
{
  cap_.open(device_path, cv::CAP_V4L);
  if (!cap_.isOpened())
  {
    throw DeviceError("device_path " + device_path + " cannot be opened");
  }
  pub_ = it_.advertiseCamera(topic_name_, buffer_size_);

  loadCameraInfo();
}
**/
/**
void Capture::open()
{
  open(0);
}
**/
/**
void Capture::openFile(const std::string &file_path)
{
  cap_.open(file_path);
  if (!cap_.isOpened())
  {
    std::stringstream stream;
    stream << "file " << file_path << " cannot be opened";
    throw DeviceError(stream.str());
  }
  pub_ = it_.advertiseCamera(topic_name_, buffer_size_);

  std::string url;
  if (node_.getParam("camera_info_url", url))
  {
    if (info_manager_.validateURL(url))
    {
      info_manager_.loadCameraInfo(url);
    }
  }
}
**/
bool Capture::capture()
{
  cv::Mat image_raw;
  cv::Mat image_raw_rect;

  if (cap_.read(image_raw))
  {
      //cv::remap(image_raw, image_raw, map1[0], map2[0],  cv::INTER_AREA);
      for(int i = 0; i < COUNT; ++i)
      {
        if(status[i])
        {
          //cv::resize(image_raw, bridges[i].image, size_image[i], 0, 0, cv::INTER_AREA);
          cv::remap(image_raw, bridges[i].image, map1[i], map2[i],  cv::INTER_AREA);
        }
        ros::Time stamp = ros::Time::now() - capture_delay_;
        bridges[i].encoding = enc::BGR8;
        bridges[i].header.stamp = stamp;
        bridges[i].header.frame_id = frame_id_;


        info_ = info_manager_.getCameraInfo();
        if (info_.height == 0 && info_.width == 0)
        {
          info_.height = bridges[i].image.rows;
          info_.width = bridges[i].image.cols;
        }
        else if (info_.height != bridges[i].image.rows || info_.width != bridges[i].image.cols)
        {
          if (rescale_camera_info_)
          {
            int old_width = info_.width;
            int old_height = info_.height;
            rescaleCameraInfo(bridges[i].image.cols, bridges[i].image.rows);
            ROS_INFO_ONCE("Camera calibration automatically rescaled from %dx%d to %dx%d",
                          old_width, old_height, bridges[i].image.cols, bridges[i].image.rows);
          }
          else
          {
            ROS_WARN_ONCE("Calibration resolution %dx%d does not match camera resolution %dx%d. "
                          "Use rescale_camera_info param for rescaling",
                          info_.width, info_.height, bridges[i].image.cols, bridges[i].image.rows);
          }
        }
        info_.header.stamp = stamp;
        info_.header.frame_id = frame_id_;
      }
      return true;
  }
  return false;

}

void Capture::setDecoding()
{
  cap_.set(cv::CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
}

void Capture::publish()
{
  for(int i = 0;i < COUNT; ++i)
  {
    if(status[i])
    {
      cam_pubs[i].publish(*(bridges[i].toImageMsg()), info_);
    }

  }

}

bool Capture::setPropertyFromParam(int property_id, const std::string &param_name)
{
  if (cap_.isOpened())
  {
    double value = 0.0;
    if (node_.getParam(param_name, value))
    {
      ROS_INFO("setting property %s = %lf", param_name.c_str(), value);
      return cap_.set(property_id, value);
    }
  }
  return true;
}

} // namespace cv_camera
