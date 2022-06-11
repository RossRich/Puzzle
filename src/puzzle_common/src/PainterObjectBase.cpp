
#include "puzzle_common/PainterObjectBase.hpp"

PainterObjectBase::PainterObjectBase(const char *name, const char *frameId) {
  _marker.ns = name;
  _marker.id = _id;
  _marker.header.frame_id = frameId;
  setPosition(tf2::Vector3(0, 0, 0));
  setOrientation(tf2::Quaternion::getIdentity());
}

Marker &PainterObjectBase::getMarker() {
  _marker.header.stamp = ros::Time::now();
  return _marker;
}

Marker &PainterObjectBase::drawMarker() {
  _marker.header.stamp = ros::Time::now();
  _marker.action = Marker::ADD;
  _marker.id = _id++;
  return _marker;
}

Marker &PainterObjectBase::updateMarker() {
  _marker.header.stamp = ros::Time::now();
  _marker.action = Marker::MODIFY;
  _marker.id = _id - 1;
  return _marker;
}

void PainterObjectBase::setColor(float r, float g, float b, float a) {
  _marker.color = RvizVisually::getColorMsg(r, g, b, a);
}

void PainterObjectBase::setColor(const ColorRGBA &color) {
  _marker.color = color;
}

void PainterObjectBase::setScale(float x, float y, float z) {
  _marker.scale.x = x;
  _marker.scale.y = y;
  _marker.scale.z = z;
}

void PainterObjectBase::setScale(float scale) {
  setScale(scale, scale, scale);
}

void PainterObjectBase::setPose(const Pose &pose) {
  _marker.pose = pose;
}

void PainterObjectBase::setPosition(const tf2::Vector3 &position) {
  tf2::toMsg(position, _marker.pose.position);
}

void PainterObjectBase::setOrientation(const tf2::Quaternion &orientation) {
  _marker.pose.orientation = tf2::toMsg(orientation);
}