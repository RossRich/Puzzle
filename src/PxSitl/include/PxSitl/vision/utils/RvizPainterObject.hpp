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
  uint16_t _id = 0;
  Marker _marker;

public:
  PainterObjectBase(const char *name, const char *frameId) {
    _marker.ns = name;
    _marker.header.frame_id = frameId;
  }

  ~PainterObjectBase() {}

  Marker &getMarker() {
    _marker.header.stamp = ros::Time::now();
    _marker.id = _id++;
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

class RvizPainterObject {
private:
  RvizArrow redArrow = RvizArrow("red_arrow", "map");
  RvizArrow yellowArrow = RvizArrow("yellow_arrow", "map");

public:
  RvizPainterObject() {
    redArrow.setColor(RvizVisually::Colors().at(RvizVisually::Color::Red));
    yellowArrow.setColor(RvizVisually::Colors().at(RvizVisually::Color::Yellow));
  }
  ~RvizPainterObject() {}

  PainterObjectBase &getRegArrow() {
    return redArrow;
  }

  PainterObjectBase &getYellowArrow() {
    return yellowArrow;
  }
};

#endif // _PUZZLE_RVIZ_PAINTER_OBJECT_H_
