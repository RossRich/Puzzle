#if !defined(_BALL_TRACKING_ROS_H_)
#define _BALL_TRACKING_ROS_H_

#include "State/State.hpp"
#include "State/StateTracking.hpp"
#include "State/StateWait.hpp"
#include "Strategy/Strategy.hpp"
#include "Strategy/StrategyTracking.hpp"
#include "Strategy/StrategyWait.hpp"
#include "VideoHandler.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

class BallTrackingRos {

private:
  Strategy *_strategy = nullptr;
  State *_state = nullptr;
  ros::NodeHandle &_nh;
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

  void tracking();
  void wait();
  void loop();
};

#endif // _BALL_TRACKING_ROS_H_
