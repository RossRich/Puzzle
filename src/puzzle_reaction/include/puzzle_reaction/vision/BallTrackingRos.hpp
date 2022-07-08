#if !defined(_BALL_TRACKING_ROS_H_)
#define _BALL_TRACKING_ROS_H_

#include <ros/ros.h>
#include <std_srvs/Empty.h>

class State;
class StateWait;
class StateTracking;

class BallTrackingRos {

private:
  State *_state = nullptr;
  StateWait *_stateWait = nullptr;
  StateTracking *_stateTracking = nullptr;

  ros::NodeHandle &_nh;
  // ros::ServiceServer _strategySrv;
public:
  BallTrackingRos(ros::NodeHandle &nh);
  ~BallTrackingRos();
  // static BallTrackingRos* getInstanse(ros::NodeHandle &nh);

  void shutdown();
  void setState(State *state);
  void loop();

  // bool runSetupSrv(std_srvs::EmptyRequest &request, std_srvs::EmptyResponse &response);

  inline ros::NodeHandle& getNodeHandler() { return _nh; }
  inline StateWait *getStateWait() { return _stateWait; }
  inline StateTracking *getStateTracking() { return _stateTracking; }

  friend bool operator==(State &state, BallTrackingRos &btr) {
    return &state == btr._state;
  }
};

#include "State.hpp"
#include "StateWait.hpp"
#include "StateTracking.hpp"

#include "../../../src/vision/StateWait.cpp"
#include "../../../src/vision/StateTracking.cpp"

#endif // _BALL_TRACKING_ROS_H_
