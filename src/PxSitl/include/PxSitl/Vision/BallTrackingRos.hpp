#if !defined(_BALL_TRACKING_ROS_H_)
#define _BALL_TRACKING_ROS_H_

#include "VideoHandler.hpp"
#include "BallTracking.hpp"
#include "TrackingParam.hpp"
#include "State/State.hpp"
#include "Strategy/Strategy.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

class BallTrackingRos {
  
private:
  Strategy *_strategy = nullptr;
  State *_state = nullptr;
  ros::NodeHandle &_nh;
  VideoHandler &_vh;
  // BallTracking _bt;
  ros::ServiceServer _strategySrv;

public:
  BallTrackingRos(ros::NodeHandle &nh, VideoHandler &vh);
  ~BallTrackingRos();

  void transitionTo(State *state);

  void setStrategy(Strategy *strategy);

  bool runSetupSrv(std_srvs::EmptyRequest &request, std_srvs::EmptyResponse &response);
  // void toTrackingStrategy();
  // void toSetupStrategy();
  bool loadParam();
  void tracking();
  void setup();
  void run();

  void test1();
  void test2();

  inline VideoHandler& getVideoHandler() { return _vh; }
};

#include "State/SetupState.hpp"
#include "State/TrackingState.hpp"

#endif // _BALL_TRACKING_ROS_H_
