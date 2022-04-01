#if !defined(_BALL_TRACKING_ROS_H_)
#define _BALL_TRACKING_ROS_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <std_srvs/Empty.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

#include "VideoHandler.hpp"
#include "RosVH.hpp"

using sensor_msgs::CameraInfo;
using sensor_msgs::CameraInfoConstPtr;
using visualization_msgs::Marker;
using visualization_msgs::MarkerConstPtr;

class State;
class StateWait;
class StateInit;

class BallTrackingRos {

private:
  static BallTrackingRos *_instance;

  State *_state = nullptr;
  StateInit *_stateInit = nullptr;
  StateWait *_stateWait = nullptr;
  // StateTracking *_stateTracking = nullptr;

  ros::NodeHandle &_nh;
  ros::ServiceServer _strategySrv;

  BallTrackingRos(ros::NodeHandle &nh);

public:
  ~BallTrackingRos();
  static BallTrackingRos* getInstanse(ros::NodeHandle &nh);

  bool runSetupSrv(std_srvs::EmptyRequest &request, std_srvs::EmptyResponse &response);
  // bool loadParam();
  void shutdown();

  inline StateWait *getStateWait() { return _stateWait; }
  inline StateInit *getStateInit() { return _stateInit; }
  // inline StateTracking *getStateTracking() { return _stateTracking; }

  void setState(State *state);          ///< class StateContext;
  // void tracking();                      ///< class StateContext;
  void loop();                          ///< calss StateContext;
  void wait();                          ///< class StateContext;
  void init();                          ///< class StateContext;
};

BallTrackingRos *BallTrackingRos::_instance = nullptr;

#include "State.hpp"
#include "StateWait.hpp"
#include "StateInit.hpp"

#include "StateInit.cpp"
#include "StateWait.cpp"

#endif // _BALL_TRACKING_ROS_H_
