#if !defined(_BALL_TRACKING_ROS_H_)
#define _BALL_TRACKING_ROS_H_

#include "State.hpp"
#include "StateTracking.hpp"
#include "StateWait.hpp"
#include "Strategy.hpp"
#include "StrategyTracking.hpp"
#include "StrategyWait.hpp"
#include "VideoHandler.hpp"
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

using sensor_msgs::CameraInfoConstPtr;
using visualization_msgs::Marker;
using visualization_msgs::MarkerConstPtr;

class BallTrackingRos {

private:
  std::string _confFile = "/workspaces/Puzzle/src/PxSitl/data/config.yaml";
  float _filterGain;

  ros::NodeHandle &_nh;
  ros::ServiceServer _strategySrv;
  ros::Publisher _ballPub;

  Strategy *_strategy = nullptr;
  State *_state = nullptr;
  CameraInfoConstPtr _cameraInfo;
  VideoHandler &_vh;

public:
  BallTrackingRos(ros::NodeHandle &nh, VideoHandler &vh);
  ~BallTrackingRos();

  bool runSetupSrv(std_srvs::EmptyRequest &request, std_srvs::EmptyResponse &response);
  bool loadParam();
  void drawBallPos(geometry_msgs::Pose p);
  void drawBallDiract(geometry_msgs::Pose p);
  void drawPredicted(std::list<geometry_msgs::Point> list);
  void draBallTrajectory(std::list<cv::Point3d> &bt);
  void pubMarker(Marker m);

  void setState(State *state);
  void setStrategy(Strategy *strategy);

  VideoHandler &getVideoHandler() const { return _vh; }
  const char *getConfFile() const { return _confFile.c_str(); }
  CameraInfoConstPtr &getCameraInfo() { return _cameraInfo; }
  float getFilterGain() { return _filterGain; }

  void tracking();
  void wait();
  void loop();
  void shutdown();
};

#include "StateTracking.hpp"
#include "StateWait.hpp"

#endif // _BALL_TRACKING_ROS_H_
