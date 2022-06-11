#if !defined(_PUZZLE_COMMON_PAINTER_OBJECTS_H_)
#define _PUZZLE_COMMON_PAINTER_OBJECTS_H_

#include "RvizVisually.hpp"
#include "PainterObjectBase.hpp"

using geometry_msgs::Point;
using geometry_msgs::Pose;
using std_msgs::ColorRGBA;
using visualization_msgs::Marker;

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

class RvizPoints : public PainterObjectBase {
public:
  RvizPoints(const char *name, const char *frameId) : PainterObjectBase(name, frameId) {
    _marker.type = Marker::POINTS;
    setScale(.05f);
    setColor(RvizVisually::Colors().at(RvizVisually::Color::Green));
  }
  ~RvizPoints() {}
};

class RvizMesh : public PainterObjectBase {
public:
  RvizMesh(const char *name, const char *frameId) : PainterObjectBase(name, frameId) {
    _marker.type = Marker::MESH_RESOURCE;
    setScale(1.f);
    setColor(RvizVisually::Colors().at(RvizVisually::Color::Green));
  }

  inline void setResource(const std::string &resource) {
    _marker.mesh_resource = resource;
  }
  ~RvizMesh() {}
};

#endif // _PUZZLE_COMMON_PAINTER_OBJECTS_H_
