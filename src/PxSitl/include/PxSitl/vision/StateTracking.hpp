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
};

#endif // _VISION_STATE_TRACKING_H_
