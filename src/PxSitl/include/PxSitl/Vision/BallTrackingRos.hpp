#if !defined(_BALL_TRACKING_ROS_H_)
#define _BALL_TRACKING_ROS_H_

#include "State.hpp"
#include "StateTracking.hpp"
#include "StateWait.hpp"
#include "Strategy.hpp"
#include "StrategyTracking.hpp"
#include "StrategyWait.hpp"
#include "VideoHandler.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/CameraInfo.h>

using sensor_msgs::CameraInfoConstPtr;

class BallTrackingRos {

private:
  Strategy *_strategy = nullptr;
  State *_state = nullptr;
  ros::NodeHandle &_nh;
  CameraInfoConstPtr _cameraInfo;
  VideoHandler &_vh;
  ros::ServiceServer _strategySrv;
  std::string _confFile = "/workspaces/Puzzle/src/PxSitl/data/config.yaml";

public:
  BallTrackingRos(ros::NodeHandle &nh, VideoHandler &vh);
  ~BallTrackingRos();

  bool runSetupSrv(std_srvs::EmptyRequest &request,
                   std_srvs::EmptyResponse &response);
  bool loadParam();

  void setState(State *state);
  void setStrategy(Strategy *strategy);

  VideoHandler &getVideoHandler() const { return _vh; }
  const char *getConfFile() const { return _confFile.c_str(); }
  CameraInfoConstPtr& getCameraInfo() { return _cameraInfo; }

  void tracking();
  void wait();
  void loop();
  void shutdown();
};

#include "StateTracking.hpp"
#include "StateWait.hpp"

#endif // _BALL_TRACKING_ROS_H_
