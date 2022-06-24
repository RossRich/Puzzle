#include "puzzle_transform/PuzzleTransform.hpp"

void PuzzleTransform::getLocalPosition(const PoseStampedConstPtr &localPosition) { transform(localPosition); }

bool PuzzleTransform::loadParam() {
  if (!_nh.getParam(ros::this_node::getName() + "/mavros_local_position", _mavrosLocalPositionTopic)) {
    ROS_ERROR("[PuzzleTransform::loadParam] no parameter \'mavros_local_position\'");
    return false;
  }

  if (!_nh.getParam(ros::this_node::getName() + "/drone_name", _vehicleName)) {
    ROS_ERROR("[PuzzleTransform::loadParam] no parameter \'drone_name\'");
    return false;
  }

  return true;
}

void PuzzleTransform::connect() {
  _localPositionSubs = _nh.subscribe<PoseStamped>("/" + _vehicleName + _mavrosLocalPositionTopic, 10,
                                                  &PuzzleTransform::getLocalPosition, this);
}

void PuzzleTransform::transform(const PoseStampedConstPtr &ps) {
  _transformMsg.header.stamp = ros::Time::now();
  _transformMsg.transform.rotation = ps->pose.orientation;
  _transformMsg.transform.translation.x = ps->pose.position.x;
  _transformMsg.transform.translation.y = ps->pose.position.y;
  _transformMsg.transform.translation.z = ps->pose.position.z;
  _transformBroadcaster.sendTransform(_transformMsg);
}

PuzzleTransform::PuzzleTransform(ros::NodeHandle &nh) : _nh(nh) {
  if (!loadParam())
    throw std::runtime_error("Invalid parameters");

  connect();

  _transformMsg.header.frame_id = "map";
  _transformMsg.child_frame_id = "base_link";
}