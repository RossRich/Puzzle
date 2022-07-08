#include "puzzle_reaction/vision/StateWait.hpp"

void StateWait::tracking() {
  ROS_INFO("[StateWait] Transition from %s state to Tracking state", toString().c_str());

  if(!_context.getStateTracking()->loadParam())
    return;
  
  if(!_context.getStateTracking()->setup())
    return;

  _context.setState(_context.getStateTracking());
}

void StateWait::execute() {
  if (ros::Time::now() - _waitTimer >= ros::Duration(1.0)) {
    tracking();
    _waitTimer = ros::Time::now();
  }
}