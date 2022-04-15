#if !defined(_VISION_STATE_TRACKING_H_)
#define _VISION_STATE_TRACKING_H_

#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_srvs/Empty.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "BallTracking.hpp"
#include "BallTrackingRos.hpp"
#include "RosVH.hpp"
#include "State.hpp"
#include "VideoHandler.hpp"
#include "utils/Utils.hpp"
#include "utils/thresholdtype.hpp"

using geometry_msgs::Pose;
using geometry_msgs::PosePtr;
using geometry_msgs::TransformStamped;
using image_geometry::PinholeCameraModel;
using sensor_msgs::CameraInfo;
using sensor_msgs::CameraInfoConstPtr;
using visualization_msgs::Marker;
using visualization_msgs::MarkerConstPtr;

class StateTracking : public State {
private:
  int _fps = 0;
  bool _isObjDetected = false;
  bool _isTrekLinePredicted = false;
  float _filterGain = 0.65f;
  const char *_winName = "Tracking";

  BallTracking *_bt = nullptr;
  RosVH *_vh = nullptr;

  std::string _confFile = "";
  std::string _cameraInfoTopic = "/camera/color/camera_info";
  std::list<geometry_msgs::Point> _objRealLine;
  std::list<geometry_msgs::Point> _objPredictedLine;

  ros::NodeHandle &_nh;
  ros::Time _loopTimer;
  ros::Time _startTrackingTimer;
  ros::Time _resetTimer;
  ros::Time _detectionTimer;
  ros::Time _buildRealTrekLineTimer;
  ros::Publisher _ballPub;
  tf2_ros::Buffer _tfBuffer;
  tf2_ros::TransformListener *_tfListener;
  tf2::Vector3 _firstObjPose;
  Pose _lastObjPose;
  image_transport::ImageTransport *_it;
  CameraInfoConstPtr _cameraInfo;
  PinholeCameraModel _cameraModel;

  cv::Mat _frame;
  cv::Mat _depth;

  void drawObjPose(Pose &p);
  void drawObjPose(Pose &p, std_msgs::ColorRGBA &c);
  void drawObjPose(geometry_msgs::Point &p, std_msgs::ColorRGBA &c);
  void drawLine(geometry_msgs::Point &p1, geometry_msgs::Point &p2, std_msgs::ColorRGBA &c);
  void drawObjPredictedLine(std::list<geometry_msgs::Point> &list);
  void drawObjRealLine(std::list<geometry_msgs::Point> &list);
  void pubMarker(Marker m);
  void transformPose(Pose &pose);
  void conceptOne(cv::Mat &mask, cv::Point2i &center, uint16_t &radius);
  void conceptTwo(cv::Mat &mask, cv::Point2i &center, uint16_t &radius);
  float getDistToObj(cv::Mat &mask, uint16_t &radius);

  /**
   * Builds a 3D point of object from 2D Image via camera model
   *
   * @param[in] point2d cv::Point2i - 2d point
   * @param[in] distToObj distance to interest object
   * @param[out] point3d geometry_msgs::Pose - 3d point
   **/
  void getObjPoseFromCameraModel(cv::Point2i &point2d, float distToObj, Pose &objPose);

public:
  StateTracking(BallTrackingRos &context, ros::NodeHandle &nh) : State(context, "Tracking"), _nh(nh) {}
  ~StateTracking();

  bool loadParam();
  bool setup();

  void tracking() override {}
  void wait() override;
  void execute() override;

  float getVelocity(float x, float y, float angle) {
    float g = 9.8f;
    float v2 = g * pow(x, 2) / (2.0f * (y - tan(angle) * x) * pow(cos(angle), 2));
    return sqrt(abs(v2));
  }

  std_msgs::ColorRGBA getColorMsg(float r, float g, float b) {
    std_msgs::ColorRGBA color;
    color.a = 1.0f;
    color.r = r;
    color.g = g;
    color.b = b;
    return color;
  }

  float getContactProbobility(tf2::Vector3 &currentPosition) {
    if (_objPredictedLine.size() == 0)
      return -1;

    std::vector<std_msgs::ColorRGBA> colors;
    colors.push_back(getColorMsg(1, 0, 0));
    colors.push_back(getColorMsg(0, 1, 0));
    colors.push_back(getColorMsg(0, 0, 1));
    colors.push_back(getColorMsg(0, 0.5, 0.5));

    uint16_t realTrekLen = _objRealLine.size();
    uint16_t predictedTrekLen = _objPredictedLine.size();
    uint16_t realTrekMiddle = realTrekLen / 2;
    uint16_t predictedTrekMiddle = predictedTrekLen / 2;

    // ROS_DEBUG("RealTrekLine size: %i\nMiddle: %i", _objRealLine.size(), middle);
    // geometry_msgs::Point curRealPoint;
    uint16_t realTrekPosCounter = 0;
    static uint8_t colorCounter = 0;
    // for (auto realPoint : _objRealLine) {
    // tf2::Vector3 vecRealPoint;
    // tf2::fromMsg(realPoint, vecRealPoint);

    tf2::Vector3 vecPredPoint;
    tf2::fromMsg(_objPredictedLine.back(), vecPredPoint);

    float minDist1 = tf2::tf2Distance2(currentPosition, vecPredPoint);
    float minDist2 = minDist1;
    // curRealPoint = realPoint;
    geometry_msgs::Point nearPoint1;
    geometry_msgs::Point nearPoint2;
    for (auto predPoint : _objPredictedLine) {
      // tf2::Vector3 vecPredPoint;
      tf2::fromMsg(predPoint, vecPredPoint);
      float dist = tf2::tf2Distance2(currentPosition, vecPredPoint);
      if (dist < minDist1) {
        minDist2 = minDist1;
        nearPoint2 = nearPoint1;
        minDist1 = dist;
        nearPoint1 = predPoint;
      }
    }

    if (nearPoint1 != geometry_msgs::Point() && nearPoint2 != geometry_msgs::Point()) {
      if (colorCounter == colors.size() - 1)
        colorCounter = 0;
      geometry_msgs::Point tmpPoint;
      tf2::toMsg(currentPosition, tmpPoint);
      drawLine(nearPoint1, nearPoint2, colors[colorCounter]);
      drawObjPose(tmpPoint, colors[colorCounter]);
      ++colorCounter;
    }
    // }

    return 1;
  }
};

#endif // _VISION_STATE_TRACKING_H_
