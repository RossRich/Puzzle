#include "StrategyWait.hpp"

void StrategyWait::execute() {

  if (ros::Duration(ros::Time::now() - _timer) >= _timeOut) {

    threshold_t thresh;
    if (Utils::readThresholds(_confFile.c_str(), thresh)) {
      ROS_INFO("Data available");
      // _context->tracking();
      _state->tracking();
    } else
      ROS_WARN("No data");

    _timer = ros::Time::now();
  }
}