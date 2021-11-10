#if !defined(_BALL_TRACKING_ROS_H_)
#define _BALL_TRACKING_ROS_H_

#include "BallTracking.hpp"
#include "State.hpp"
#include "StateWait.hpp"
#include "StateTracking.hpp"
#include "Strategy.hpp"
#include "StrategyTracking.hpp"
#include "StrategyWait.hpp"
#include "TrackingParam.hpp"
#include "VideoHandler.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

class BallTrackingRos {

private:
  ros::NodeHandle &_nh;
  VideoHandler &_vh;
  State *_state = nullptr;
  Strategy *_strategy = nullptr;
  std::string _confFile = "/workspaces/Puzzle/src/PxSitl/data/config.yaml";

public:
  BallTrackingRos(ros::NodeHandle &nh, VideoHandler &vh);
  ~BallTrackingRos();

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
