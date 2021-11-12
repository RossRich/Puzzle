#if !defined(_BALL_TRACKING_ROS_H_)
#define _BALL_TRACKING_ROS_H_

#include "BallTracking.hpp"
<<<<<<< HEAD
#include "State.hpp"
#include "StateWait.hpp"
#include "StateTracking.hpp"
#include "Strategy.hpp"
#include "StrategyTracking.hpp"
#include "StrategyWait.hpp"
#include "TrackingParam.hpp"
#include "VideoHandler.hpp"
=======
#include "TrackingParam.hpp"
#include "State/State.hpp"
#include "Strategy/Strategy.hpp"
>>>>>>> 8ad21a8c05e4c999f491118b7574d61db3ade669
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

class BallTrackingRos {

private:
  Strategy *_strategy = nullptr;
  State *_state = nullptr;
  ros::NodeHandle &_nh;
  VideoHandler &_vh;
<<<<<<< HEAD
  State *_state = nullptr;
  Strategy *_strategy = nullptr;
  std::string _confFile = "/workspaces/Puzzle/src/PxSitl/data/config.yaml";
=======
  // BallTracking _bt;
  ros::ServiceServer _strategySrv;
>>>>>>> 8ad21a8c05e4c999f491118b7574d61db3ade669

public:
  BallTrackingRos(ros::NodeHandle &nh, VideoHandler &vh);
  ~BallTrackingRos();

  void transitionTo(State *state);

  void setStrategy(Strategy *strategy);

  bool runSetupSrv(std_srvs::EmptyRequest &request, std_srvs::EmptyResponse &response);
  // void toTrackingStrategy();
  // void toSetupStrategy();
  bool loadParam();

  void setState(State *state);
  void setStrategy(Strategy *strategy);

  VideoHandler &getVideoHandler() const { return _vh; }
  const char *getConfFile() const { return _confFile.c_str(); }

  void tracking();
<<<<<<< HEAD
  void wait();
  void loop();
=======
  void setup();
  void run();

  void test1();
  void test2();

  inline VideoHandler& getVideoHandler() { return _vh; }
>>>>>>> 8ad21a8c05e4c999f491118b7574d61db3ade669
};

#include "State/SetupState.hpp"
#include "State/TrackingState.hpp"

#endif // _BALL_TRACKING_ROS_H_
