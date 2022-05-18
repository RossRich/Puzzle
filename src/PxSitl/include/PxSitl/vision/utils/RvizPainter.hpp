#if !defined(_PUZLLE_ROS_RVIZ_PAINTER_H_)
#define _PUZLLE_ROS_RVIZ_PAINTER_H_

#include "RvizPainterObject.hpp"
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <visualization_msgs/Marker.h>

using geometry_msgs::Point;
using geometry_msgs::Pose;
using std_msgs::ColorRGBA;
using visualization_msgs::Marker;

class RvizPainter {
private:
  const char *_pubName;
  ros::NodeHandle &_nh;
  ros::Publisher _markersPublisher;

public:
  explicit RvizPainter(ros::NodeHandle &nh, const char *pubName = "rviz_painter") : _nh(nh), _pubName(pubName) {
    _markersPublisher = _nh.advertise<Marker>(_pubName, 100);
  }

  ~RvizPainter() {
    _markersPublisher.shutdown();
  }

  //TODO: void update(Marker) { Marker.update() };

  void draw(PainterObjectBase &obj, const Pose &pose) {
    Marker &m = obj.drawMarker();
    m.pose = pose;
    _markersPublisher.publish(m);
  }

  void draw(RvizArrow &obj, const Pose &pose) {
    Marker &m = obj.drawMarker();
    m.pose = pose;
    _markersPublisher.publish(m);
  }

  void draw(RvizArrow &obj, const tf2::Vector3 &position, const tf2::Quaternion &orientation) {
    Pose pose;
    tf2::toMsg(position, pose.position);
    pose.orientation = tf2::toMsg(orientation);

    draw(obj, pose);
  }

  void draw(RvizLine &obj, const Point &p1, const Point &p2) {
    Marker &m = obj.drawMarker();
    m.points.push_back(p1);
    m.points.push_back(p2);
    
    _markersPublisher.publish(m);
  }

  void draw(RvizLine &obj, const tf2::Vector3 &p1, const tf2::Vector3 &p2) {
    Point p1Msg, p2Msg;
    tf2::toMsg(p1, p1Msg);
    tf2::toMsg(p2, p2Msg);

    draw(obj, p1Msg, p2Msg);
  }

  void draw(RvizLineStrip &obj, const std::list<tf2::Vector3> &points) {
    Marker &m = obj.drawMarker();
    Point pMsg;
    m.points.clear();
    for (auto &point : points) {
      tf2::toMsg(point, pMsg);
      m.points.push_back(pMsg);
    }
    
    _markersPublisher.publish(m);
  }

  void draw(RivzPoints &obj, const std::list<tf2::Vector3> &points) {
    Marker &m = obj.drawMarker();
    Point pMsg;
    m.points.clear();
    for (auto &point : points)
    {
      tf2::toMsg(point, pMsg);
      m.points.push_back(pMsg);
    }
    
    _markersPublisher.publish(m);
  }

  void draw(RvizPosition &obj, const tf2::Vector3 &position) {
    Marker &m = obj.drawMarker();
    tf2::toMsg(position, m.pose.position);

    _markersPublisher.publish(m);
  }

  /* void drawArrow(const Pose &pose) {
   Marker m;
   m.header.frame_id = "map";
   m.header.stamp = ros::Time::now();
   m.ns = name;
   m.id = 500;
   m.type = Marker::ARROW;
   m.action = Marker::ADD;

   m.pose = pose;

   m.scale.x = .5;
   m.scale.y = .02;
   m.scale.z = .02;

   m.color = c;

   pubMarker(m);
 }

 void drawArrow(const tf2::Vector3 &position, const tf2::Quaternion &orientation) {
   Pose p;
   tf2::toMsg(position, p.position);
   p.orientation = tf2::toMsg(orientation);

   drawArrow(p, c, name);
 }

 void drawPosition(Pose &p) {
   Marker m;
   m.header.frame_id = "map";
   m.header.stamp = ros::Time::now();
   m.ns = "obj_position";
   m.id = 0;
   m.type = Marker::SPHERE;
   m.pose = p;
   m.scale.x = .1;
   m.scale.y = .1;
   m.scale.z = .1;
   m.color = Utils::Colors.at(Utils::Color::SonicSilver);

   pubMarker(m);
 }

 void drawPosition(Pose &p, const ColorRGBA &c) {
   static uint16_t id = 200;
   Marker m;

   m.header.frame_id = "map";
   m.header.stamp = ros::Time::now();
   m.ns = "near_position";
   m.id = ++id;
   m.pose = p;
   m.scale.x = .09;
   m.scale.y = .09;
   m.scale.z = .09;

   m.color = c;

   m.type = Marker::CUBE;

   pubMarker(m);
 }

 void drawPosition(Point &p, const ColorRGBA &c) {
   Pose tmpPose;
   tmpPose.position = p;
   tf2::Quaternion tmpQat(tf2::Quaternion::getIdentity());
   tmpPose.orientation = tf2::toMsg(tmpQat);
   drawObjPose(tmpPose, c);
 }

 void drawPosition(const tf2::Vector3 &position, const ColorRGBA &c) {
   geometry_msgs::Point tmpPointMsg;
   tf2::toMsg(position, tmpPointMsg);
   drawObjPose(tmpPointMsg, c);
 } */

  /* void drawLine(Point &p1, Point &p2, const ColorRGBA &c) {
    static uint16_t id = 400;
    Marker m;

    m.header.frame_id = "map";
    m.header.stamp = ros::Time::now();
    m.id = ++id;
    m.ns = std::string("line ").append(std::to_string(id));
    m.type = Marker::LINE_LIST;
    m.action = Marker::ADD;

    m.points.push_back(p1);
    m.points.push_back(p2);
    m.pose.orientation.w = 1;

    m.scale.x = 0.02;

    m.color = c;

    pubMarker(m);
  }

  void drawLine(const tf2::Vector3 &p1, const tf2::Vector3 &p2, const ColorRGBA &c) {
    geometry_msgs::Point p1Msg;
    geometry_msgs::Point p2Msg;

    tf2::toMsg(p1, p1Msg);
    tf2::toMsg(p2, p2Msg);

    drawLine(p1Msg, p2Msg, c);
  } */

  /* void StateTracking::drawObjPredictedLine(std::list<geometry_msgs::Point> &list) {
    Marker m;
    static int i = 1;

    m.header.frame_id = "map";
    m.header.stamp = ros::Time::now();
    m.ns = "obj_predicted_trajectory";
    m.id = i++;
    m.action = Marker::ADD;
    m.type = Marker::SPHERE_LIST;
    m.color = Utils::getColorMsg(1, 1, 1);
    m.scale.x = .04;
    m.scale.y = .04;
    m.scale.z = .04;

    for (auto &&p : list)
      m.points.push_back(p);

    m.pose.orientation.w = 1;

    pubMarker(m);
  } */

  /* void StateTracking::drawObjRealLine(std::list<geometry_msgs::Point> &list) {
    Marker line;

    line.header.frame_id = "map";
    line.header.stamp = ros::Time::now();
    line.id = 3;
    line.ns = "obj_real_trajectory";
    line.type = Marker::LINE_STRIP;
    line.color = Utils::getColorMsg(0, 1, 0);
    line.scale.x = .02f;
    line.scale.y = .02f;

    for (auto &&i : list)
      line.points.push_back(i);

    line.pose.orientation.w = 1;

    pubMarker(line);
  } */
};

#endif // _PUZLLE_ROS_RVIZ_PAINTER_H_
