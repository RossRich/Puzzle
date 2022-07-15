#include "puzzle_reaction/vision/BallTrackingRos.hpp"

BallTrackingRos::BallTrackingRos(ros::NodeHandle &nh) : _nh(nh) {

  // _strategySrv = _nh.advertiseService("strategy_srv", &BallTrackingRos::runSetupSrv, this);
  // ROS_INFO("Change strategy server ready");

  _stateWait = new StateWait(*this);
  _stateTracking = new StateTracking(*this, _nh);

  setState(_stateWait);
}

BallTrackingRos::~BallTrackingRos() {
  delete _stateWait;
  delete _stateTracking;
}

/* bool BallTrackingRos::runSetupSrv(std_srvs::EmptyRequest &request, std_srvs::EmptyResponse &response) {
  return true;
} */

void BallTrackingRos::shutdown() {
  _nh.shutdown();
}

void BallTrackingRos::loop() {
  _state->execute();
}

void BallTrackingRos::setState(State *state) {
  _state = state;
}