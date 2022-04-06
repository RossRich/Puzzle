#if !defined(_VISION_STATE_TRACKING_H_)
#define _VISION_STATE_TRACKING_H_

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_srvs/Empty.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "State.hpp"
#include "RosVH.hpp"
#include "VideoHandler.hpp"
#include "BallTrackingRos.hpp"
#include "BallTracking.hpp"
#include "utils/thresholdtype.hpp"
#include "utils/Utils.hpp"

using sensor_msgs::CameraInfo;
using sensor_msgs::CameraInfoConstPtr;
using visualization_msgs::Marker;
using visualization_msgs::MarkerConstPtr;
using geometry_msgs::TransformStamped;
using image_geometry::PinholeCameraModel;

class StateTracking : public State {
private:
  int _fps = 0;
  float _filterGain = 0.65f;
  const char *_winName = "Tracking";

  BallTracking *_bt = nullptr;
  RosVH *_vh = nullptr;
  
  std::string _confFile = "";
  std::string _cameraInfoTopic = "/camera/color/camera_info";
  std::list<cv::Point3d> _ballTragectory;
  std::list<geometry_msgs::Point> _ballPredictedTraj;
  std::list<std::list<geometry_msgs::Point>> _ballPredictedTrajs;

  ros::Time _timer;
  ros::Time _lastDetection;
  ros::Time startTime;
  ros::Time resetTimer;
  ros::NodeHandle &_nh;
  ros::Publisher _ballPub;
  tf2_ros::Buffer _tfBuffer;
  tf2_ros::TransformListener *_tfListener;
  CameraInfoConstPtr _cameraInfo;
  PinholeCameraModel _cameraModel;
  image_transport::ImageTransport *_it;
  
  cv::Mat _frame;
  cv::Mat _depth;
  cv::Point3d _ballPos;
  cv::Point3d startPoint;
  
public:
  StateTracking(BallTrackingRos &context, ros::NodeHandle &nh): State(context, "Tracking"), _nh(nh) {}
  ~StateTracking();

  bool loadParam();
  bool setup();
 
  void drawBallPos(geometry_msgs::Pose p);
  void drawBallDiract(geometry_msgs::Pose p);
  void drawPredicted(std::list<geometry_msgs::Point> list);
  void draBallTrajectory(std::list<cv::Point3d> &bt);
  void pubMarker(Marker m);

  void tracking() override {}
  void wait() override;
  void execute() override;
};

#endif // _VISION_STATE_TRACKING_H_
