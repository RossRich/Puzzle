#if !defined(_PUZZLE_COMMON_RVIZ_PAINTER_BASE_H_)
#define _PUZZLE_COMMON_RVIZ_PAINTER_BASE_H_

#include "RvizVisually.hpp"
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2/convert.h>
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
  PainterObjectBase(const char *name, const char *frameId);

  ~PainterObjectBase() {}

  Marker &getMarker();

  Marker &drawMarker();

  Marker &updateMarker();

  Marker &clear();

  void setColor(float r, float g, float b, float a = 1.0f);

  void setColor(const ColorRGBA &color);

  void setScale(float x, float y, float z);

  void setScale(float scale);

  void setPose(const Pose &pose);

  void setPosition(const tf2::Vector3 &position);

  void setOrientation(const tf2::Quaternion &orientation);
};

#endif // _PUZZLE_COMMON_RVIZ_PAINTER_BASE_H_
