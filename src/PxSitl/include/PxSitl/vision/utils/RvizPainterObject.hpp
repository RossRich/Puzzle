#if !defined(_PUZZLE_RVIZ_PAINTER_OBJECT_H_)
#define _PUZZLE_RVIZ_PAINTER_OBJECT_H_

#include "RvizVisually.hpp"
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <visualization_msgs/Marker.h>

using geometry_msgs::Point;
using geometry_msgs::Pose;
using std_msgs::ColorRGBA;
using visualization_msgs::Marker;

class PainterObjectBase {
protected:
  uint16_t _id = 1;
  Marker _marker;

public:
  PainterObjectBase(const char *name, const char *frameId) {
    _marker.ns = name;
    _marker.id = _id;
    _marker.header.frame_id = frameId;
    setPosition(tf2::Vector3(0, 0, 0));
    setOrientation(tf2::Quaternion::getIdentity());
  }

  ~PainterObjectBase() {}

  Marker &getMarker() {
    _marker.header.stamp = ros::Time::now();
    return _marker;
  }

  Marker &drawMarker() {
    _marker.header.stamp = ros::Time::now();
    _marker.action = Marker::ADD;
    _marker.id = _id++;
    return _marker;
  }

  Marker &updateMarker() {
    _marker.header.stamp = ros::Time::now();
    _marker.action = Marker::MODIFY;
    _marker.id = _id - 1;
    return _marker;
  }

  void setColor(float r, float g, float b, float a = 1.0f) {
    _marker.color = RvizVisually::getColorMsg(r, g, b, a);
  }

  void setColor(const ColorRGBA &color) {
    _marker.color = color;
  }

  void setScale(float x, float y, float z) {
    _marker.scale.x = x;
    _marker.scale.y = y;
    _marker.scale.z = z;
  }

  void setScale(float scale) {
    setScale(scale, scale, scale);
  }

  void setPose(const Pose &pose) {
    _marker.pose = pose;
  }

  void setPosition(const tf2::Vector3 &position) {
    tf2::toMsg(position, _marker.pose.position);
  }

  void setOrientation(const tf2::Quaternion &orientation) {
    _marker.pose.orientation = tf2::toMsg(orientation);
  }
};

class RvizArrow : public PainterObjectBase {
public:
  RvizArrow(const char *name, const char *frameId) : PainterObjectBase(name, frameId) {
    _marker.type = Marker::ARROW;
    setScale(.5, .02, .02);
    setColor(RvizVisually::Colors().at(RvizVisually::Color::Green));
  }
  ~RvizArrow() {}
};

class RvizLine : public PainterObjectBase {
public:
  RvizLine(const char *name, const char *frameId) : PainterObjectBase(name, frameId) {
    _marker.type = Marker::LINE_LIST;
    setScale(.04f);
    setColor(RvizVisually::Colors().at(RvizVisually::Color::Green));
  }
  ~RvizLine() {}
};

class RvizLineStrip : public PainterObjectBase {
public:
  RvizLineStrip(const char *name, const char *frameId) : PainterObjectBase(name, frameId) {
    _marker.type = Marker::LINE_STRIP;
    setScale(.025f);
    setColor(RvizVisually::Colors().at(RvizVisually::Color::Green));
  }
  ~RvizLineStrip() {}
};

class RvizPosition : public PainterObjectBase {
public:
  RvizPosition(const char *name, const char *frameId) : PainterObjectBase(name, frameId) {
    _marker.type = Marker::SPHERE;
    setScale(.08f);
    setColor(RvizVisually::Colors().at(RvizVisually::Color::Green));
  }
  ~RvizPosition() {}
};

class RivzPoints : public PainterObjectBase {
public:
  RivzPoints(const char *name, const char *frameId) : PainterObjectBase(name, frameId) {
    _marker.type = Marker::POINTS;
    setScale(.05f);
    setColor(RvizVisually::Colors().at(RvizVisually::Color::Green));
  }
  ~RivzPoints() {}
};

class RvizPainterObject {
private:
  RvizLine _shortAlfaBlueLine = {"short_ablue_line", "map"};
  RvizArrow _redArrow = {"red_arrow", "map"};
  RvizArrow _yellowArrow = {"yellow_arrow", "map"};
  RivzPoints _predTrajLine = {"pred_traj", "map"};
  RvizPosition _objFirstPosition = {"obj_first_position", "map"};
  RvizPosition _pointOnTraj = {"point_on_traj", "map"};
  RvizLineStrip _realTrajLine = {"real_traj", "map"};

public:
  RvizPainterObject() {
    _redArrow.setColor(RvizVisually::Colors().at(RvizVisually::Color::Red));
    _yellowArrow.setColor(RvizVisually::Colors().at(RvizVisually::Color::Yellow));
    _predTrajLine.setColor(RvizVisually::Colors().at(RvizVisually::Color::CyanProcess));
    _objFirstPosition.setColor(RvizVisually::Colors().at(RvizVisually::Color::Marigold));
    _shortAlfaBlueLine.setColor(RvizVisually::Colors().at(RvizVisually::Color::AlfaBlue));
    _shortAlfaBlueLine.setScale(0.02f);
    _pointOnTraj.setColor(RvizVisually::Colors().at(RvizVisually::Color::SonicSilver));
    _pointOnTraj.setScale(.05f);
  }

  ~RvizPainterObject() {}

  RvizArrow &getRegArrow() {
    return _redArrow;
  }

  RvizArrow &getYellowArrow() {
    return _yellowArrow;
  }

  RvizLineStrip &getRealTrajLine() {
    return _realTrajLine;
  }

  RivzPoints &getPredTrajLine() {
    return _predTrajLine;
  }

  RvizPosition &getObjFirstPosition() {
    return _objFirstPosition;
  }

  RvizLine &getShortABlueLine() {
    return _shortAlfaBlueLine;
  }

  RvizPosition &getPointOnTraj() {
    return _pointOnTraj;
  }
};

#endif // _PUZZLE_RVIZ_PAINTER_OBJECT_H_
