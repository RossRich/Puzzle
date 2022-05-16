#if !defined(_VISION_RVIZ_VISUALLY)
#define _VISION_RVIZ_VISUALLY

#include <visualization_msgs/Marker.h>

class RvizVisually {
private:
  RvizVisually() {}

public:
  ~RvizVisually() {}
  static std::vector<std_msgs::ColorRGBA> &Colors;
  enum Color { Black = 0, SonicSilver, White, CyanProcess, Blue, AlfaBlue, Green, DarkGreen, Red, Marigold, Yellow };

  /**
   * Wrapper function for get "color msg" of ros  as "one shot"
   * 
   * @param r red[0 - 1.0f]
   * @param g green[0 - 1.0f]
   * @param b blue[0 - 1.0f]
   * @param a opacity faktor[0 - 1.0f]
   * 
   * @return ROS ColorRGBA
   */
  static std_msgs::ColorRGBA getColorMsg(float r, float g, float b, float a = 1.0f) {
    std_msgs::ColorRGBA color;
    color.a = a;
    color.r = r;
    color.g = g;
    color.b = b;
    return color;
  }

  /**
   * Create vector of ROS ColorRGBA
   *
   * @return colors link
   */
  static std::vector<std_msgs::ColorRGBA> &createColors() {
    static std::vector<std_msgs::ColorRGBA> colors;
    ///< Black(0) - 33, 33, 33
    colors.push_back(RvizVisually::getColorMsg(33.f / 255.f, 33.f / 255.f, 33.f / 255.f, 0.8f));
    ///< SonicSilver(1) - 117, 117, 117
    colors.push_back(RvizVisually::getColorMsg(117.f / 255.f, 117.f / 255.f, 117.f / 255.f, 1.0f));
    ///< White(2) - 251, 251, 255
    colors.push_back(RvizVisually::getColorMsg(251.f / 255.f, 251.f / 255.f, 255.f / 255.f, 1.0f));
    ///< CyanProcess(3) - 1, 186, 239
    colors.push_back(RvizVisually::getColorMsg(1.f / 255.f, 186.f / 255.f, 239.f / 255.f, 1.0f));
    ///< Blue(4) - 21, 229, 244
    colors.push_back(RvizVisually::getColorMsg(21.f / 255.f, 229.f / 255.f, 244.f / 255.f, 1.0f));
    ///< AlfaBlue(5)
    colors.push_back(RvizVisually::getColorMsg(21.f / 255.f, 229.f / 255.f, 244.f / 255.f, 0.65f));
    ///< Green(6) - 19, 205, 177
    colors.push_back(RvizVisually::getColorMsg(19.f / 255.f, 205.f / 255.f, 177.f / 255.f, 1.0f));
    ///< DarkGreen(7) - 32, 191, 85
    colors.push_back(RvizVisually::getColorMsg(32.f / 255.f, 191.f / 255.f, 85.f / 255.f, 1.0f));
    ///< Red(8) - 238, 104, 59
    colors.push_back(RvizVisually::getColorMsg(238.f / 255.f, 104.f / 255.f, 59.f / 255.f, 1.0f));
    ///< Marigold(9) - 229, 165, 36
    colors.push_back(RvizVisually::getColorMsg(229.f / 255.f, 165.f / 255.f, 36.f / 255.f, 1.0f));
    ///< Yellow(10) - 226, 245, 60
    colors.push_back(RvizVisually::getColorMsg(226.f / 255.f, 245.f / 255.f, 60.f / 255.f, 1.0f));

    return colors;
  }

  

/* visualization_msgs::Marker arrowMid(const Pose &pose, const std_msgs::ColorRGBA &c, const char *name = "arrow") {
  Marker m;
  m.header.frame_id = "map";
  m.header.stamp = ros::Time::now();
  m.ns = name;
  m.id = 500 + tmpMarkerIndex;
  m.type = Marker::ARROW;
  m.action = Marker::ADD;

  m.pose = pose;

  m.scale.x = .5;
  m.scale.y = .02;
  m.scale.z = .02;

  m.color = c;

  pubMarker(m);
}

void StateTracking::drawArrow(const tf2::Vector3 &position, const tf2::Quaternion &orientation, const std_msgs::ColorRGBA &c,
                              const char *name) {
  Pose p;
  tf2::toMsg(position, p.position);
  p.orientation = tf2::toMsg(orientation);

  drawArrow(p, c, name);
}

void StateTracking::drawObjPose(Pose &p) {
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

void StateTracking::drawObjPose(Pose &p, const std_msgs::ColorRGBA &c) {
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

void StateTracking::drawObjPose(geometry_msgs::Point &p, const std_msgs::ColorRGBA &c) {
  Pose tmpPose;
  tmpPose.position = p;
  tf2::Quaternion tmpQat(tf2::Quaternion::getIdentity());
  tmpPose.orientation = tf2::toMsg(tmpQat);
  drawObjPose(tmpPose, c);
}

void StateTracking::drawObjPose(const tf2::Vector3 &position, const std_msgs::ColorRGBA &c) {
  geometry_msgs::Point tmpPointMsg;
  tf2::toMsg(position, tmpPointMsg);
  drawObjPose(tmpPointMsg, c);
}

void StateTracking::drawLine(geometry_msgs::Point &p1, geometry_msgs::Point &p2, const std_msgs::ColorRGBA &c) {
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

void StateTracking::drawLine(const tf2::Vector3 &p1, const tf2::Vector3 &p2, const std_msgs::ColorRGBA &c) {
  geometry_msgs::Point p1Msg;
  geometry_msgs::Point p2Msg;

  tf2::toMsg(p1, p1Msg);
  tf2::toMsg(p2, p2Msg);

  drawLine(p1Msg, p2Msg, c);
}

void StateTracking::drawObjPredictedLine(std::list<geometry_msgs::Point> &list) {
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

#endif // _VISION_RVIZ_VISUALLY
