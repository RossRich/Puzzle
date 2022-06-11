#include "puzzle_transform/PuzzleTransform.hpp"

void PuzzleTransform::getLocalPosition(const PoseStampedConstPtr &localPosition) {
  _dronePose = *localPosition;
  transform();
}

bool PuzzleTransform::loadParam() {
  if (!_nh.getParam(ros::this_node::getName() + "/mavros_local_position", _mavrosLocalPositionTopic)) {
    ROS_ERROR("[PuzzleTransform::loadParam] no parameter \'mavros_local_position\'");
    return false;
  }

  if (!_nh.getParam(ros::this_node::getName() + "/puzzle_vehicle_name", _vehicleName)) {
    ROS_ERROR("[PuzzleTransform::loadParam] no parameter \'puzzle_vehicle_name\'");
    return false;
  }

  return true;
}

void PuzzleTransform::connect() {
  _localPositionSubs = _nh.subscribe<PoseStamped>("/" + _vehicleName + _mavrosLocalPositionTopic, 100, &PuzzleTransform::getLocalPosition, this);
}

void PuzzleTransform::transform() {
  _transformMsg.header.stamp = ros::Time::now();
  _transformMsg.transform.rotation = _dronePose.pose.orientation;
  _transformMsg.transform.translation.x = _dronePose.pose.position.x;
  _transformMsg.transform.translation.y = _dronePose.pose.position.y;
  _transformMsg.transform.translation.z = _dronePose.pose.position.z;
  _transformBroadcaster.sendTransform(_transformMsg);
}

PuzzleTransform::PuzzleTransform(ros::NodeHandle &nh) : _nh(nh) {
  if (!loadParam())
    throw std::runtime_error("Invalid parameters");

  connect();

  _transformMsg.header.frame_id = "map";
  _transformMsg.child_frame_id = "base_link";
}