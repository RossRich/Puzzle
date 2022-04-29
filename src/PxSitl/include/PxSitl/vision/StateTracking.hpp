#if !defined(_VISION_STATE_TRACKING_H_)
#define _VISION_STATE_TRACKING_H_

#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_srvs/Empty.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "BallTracking.hpp"
#include "BallTrackingRos.hpp"
#include "RosVH.hpp"
#include "State.hpp"
#include "VideoHandler.hpp"
#include "utils/Utils.hpp"
#include "utils/thresholdtype.hpp"

using geometry_msgs::Pose;
using geometry_msgs::PosePtr;
using geometry_msgs::TransformStamped;
using image_geometry::PinholeCameraModel;
using sensor_msgs::CameraInfo;
using sensor_msgs::CameraInfoConstPtr;
using visualization_msgs::Marker;
using visualization_msgs::MarkerConstPtr;

class Line {
private:
  tf2::Vector3 _point1;
  tf2::Vector3 _point2;
  tf2::Vector3 _midpoint;

public:
  Line(const tf2::Vector3 &point3d1, const tf2::Vector3 &point3d2) : _point1(point3d1), _point2(point3d2) {
    _midpoint = tf2::lerp(_point1, _point2, .5);
  }
  ~Line() {}

  bool operator==(Line &line) {
    return _point1 == line._point1 && _point2 == line._point2;
  }

  inline const tf2::Vector3 &getP1() {
    return _point1;
  }

  inline const tf2::Vector3 &getP2() {
    return _point2;
  }

  inline const tf2::Vector3 &getMidpoint() {
    return _midpoint;
  }

  inline float length2() {
    return tf2::tf2Distance2(_point2, _point1);
  }

  inline float length() {
    return tf2::tf2Distance(_point2, _point1);
  }

  // S = ah
  float distToPoint2(const tf2::Vector3 &point) {
    float dist2ToP1 = tf2::tf2Distance2(point, _point1);
    float dist2ToP2 = tf2::tf2Distance2(point, _point2);

    auto nearPoint = std::ref(_point1);
    if (dist2ToP2 < dist2ToP1)
      nearPoint = std::ref(_point2);

    debug();
    // ROS_DEBUG("\nDistToP1 %f\nDistToP2 %f", dist2ToP1, dist2ToP2);
    tf2::Vector3 lineDir = _point2 - _point1;
    ROS_DEBUG("Line length2 %f", lineDir.length2());
    if (lineDir.length2() == 0.f)
      return -1.f;

    tf2::Vector3 numerator = tf2::tf2Cross(point - nearPoint, lineDir); // vector length equals area of parallelogram
    // ROS_DEBUG("\nNum %f\nDiv %f", numerator.length2(), lineDir.length2());

    return numerator.length2() / lineDir.length2();
  }

  float distToPoint(tf2::Vector3 &point) {
    float res = distToPoint2(point);
    return res == -1.f ? -1.f : sqrt(res);
  }

  /**
   * https://gamedev.stackexchange.com/questions/72528/how-can-i-project-a-3d-point-onto-a-3d-line
   *
   * A + dot(AP,AB) / dot(AB,AB) * AB
   * A = line point1
   * B = line point2
   * P = incoming point
   */
  tf2::Vector3 porjectPoint(const tf2::Vector3 &point) {
    tf2::Vector3 AB = _point2 - _point1;
    tf2::Vector3 AP = point - _point1;

    tf2::Vector3 res = _point1;

    float numerator = tf2::tf2Dot(AP, AB);
    float denominator = tf2::tf2Dot(AB, AB);

    res = res + numerator / denominator * AB;

    return res;
  }

  void debug() {
    ROS_DEBUG("P1 %f %f %f", _point1.x(), _point1.y(), _point1.z());
    ROS_DEBUG("P2 %f %f %f", _point2.x(), _point2.y(), _point2.z());
  }
};

class StateTracking : public State {
private:
  int _fps = 0;
  bool _isObjDetected = false;
  bool _isTrekLinePredicted = false;
  float _filterGain = 0.65f;
  const char *_winName = "Tracking";
  float _prevDist = 0.f;

  int tmpMarkerIndex = 1;

  BallTracking *_bt = nullptr;
  RosVH *_vh = nullptr;

  std::string _confFile = "";
  std::string _cameraInfoTopic = "/camera/color/camera_info";
  std::list<geometry_msgs::Point> _objRealLine;
  std::list<geometry_msgs::Point> _objPredictedLine;
  std::vector<std_msgs::ColorRGBA> _rosColors;
  std::vector<Line> _predictedSigments;

  ros::NodeHandle &_nh;
  ros::Time _loopTimer;
  ros::Time _startTrackingTimer;
  ros::Time _resetTimer;
  ros::Time _detectionTimer;
  ros::Time _buildRealTrekLineTimer;
  ros::Time _aTimer;
  ros::Publisher _ballPub;
  tf2_ros::Buffer _tfBuffer;
  tf2_ros::TransformListener *_tfListener;
  tf2::Vector3 _firstObjPose;
  Pose _lastObjPose;
  image_transport::ImageTransport *_it;
  CameraInfoConstPtr _cameraInfo;
  PinholeCameraModel _cameraModel;

  cv::Mat _frame;
  cv::Mat _depth;

  void drawLine(const tf2::Vector3 &p1, const tf2::Vector3 &p2, std_msgs::ColorRGBA &c);
  void drawLine(geometry_msgs::Point &p1, geometry_msgs::Point &p2, std_msgs::ColorRGBA &c);
  void drawObjPose(Pose &p);
  void drawObjPose(Pose &p, std_msgs::ColorRGBA &c);
  void drawObjPose(geometry_msgs::Point &p, std_msgs::ColorRGBA &c);
  void drawObjPose(const tf2::Vector3 &position, std_msgs::ColorRGBA &c);
  void drawObjRealLine(std::list<geometry_msgs::Point> &list);
  void drawObjRealLine(const tf2::Vector3 &position, std_msgs::ColorRGBA &c);
  void drawObjPredictedLine(std::list<geometry_msgs::Point> &list);
  void pubMarker(Marker m);
  void transformPose(Pose &pose);
  void conceptOne(cv::Mat &mask, cv::Point2i &center, uint16_t &radius);
  void conceptTwo(cv::Mat &mask, cv::Point2i &center, uint16_t &radius);
  float getDistToObj(cv::Mat &mask, uint16_t &radius);

  /**
   * Builds a 3D point of object from 2D Image via camera model
   *
   * @param[in] point2d cv::Point2i - 2d point
   * @param[in] distToObj distance to interest object
   * @param[out] point3d geometry_msgs::Pose - 3d point
   **/
  void getObjPoseFromCameraModel(cv::Point2i &point2d, float distToObj, Pose &objPose);

public:
  StateTracking(BallTrackingRos &context, ros::NodeHandle &nh) : State(context, "Tracking"), _nh(nh) {
    _rosColors.push_back(Utils::getColorMsg(0, 0, 0));
    _rosColors.push_back(Utils::getColorMsg(1, 0, 0));
    _rosColors.push_back(Utils::getColorMsg(0, 1, 0));
    _rosColors.push_back(Utils::getColorMsg(0, 0, 1));
    _rosColors.push_back(Utils::getColorMsg(0, 0.2, 0.5));
    _rosColors.push_back(Utils::getColorMsg(1, 1, 1));
  }
  ~StateTracking();

  bool loadParam();
  bool setup();

  void tracking() override {}
  void wait() override;
  void execute() override;

  float getVelocity(float x, float y, float angle) {
    float g = 9.8f;
    float v2 = g * pow(x, 2) / (2.0f * (y - tan(angle) * x) * pow(cos(angle), 2));
    return sqrt(abs(v2));
  }

  float getContactProbobility(tf2::Vector3 &currentPosition, tf2::Vector3 &cameraPosition) {
    if (_predictedSigments.size() == 0)
      return -1;

    uint16_t realTrekLen = _objRealLine.size();
    uint16_t predictedTrekLen = _objPredictedLine.size();
    uint16_t realTrekMiddle = realTrekLen / 2;
    uint16_t predictedTrekMiddle = predictedTrekLen / 2;

    // ROS_DEBUG("RealTrekLine size: %i\nMiddle: %i", _objRealLine.size(), middle);
    // geometry_msgs::Point curRealPoint;
    uint16_t realTrekPosCounter = 0;
    static uint8_t colorCounter = 0;

    float minDist = 100.0f;
    auto nearLine = std::ref(_predictedSigments[0]);
    auto middle = std::cref(nearLine.get().getMidpoint());

    for (auto &&line : _predictedSigments) {
      const tf2::Vector3 &tmpMiddle = line.getMidpoint();
      float tmpDist = tf2::tf2Distance2(tmpMiddle, currentPosition);

      if (tmpDist < minDist) {
        nearLine = std::ref(line);
        middle = std::cref(tmpMiddle);
        minDist = tmpDist;
      }
    }

    float distToPoint = nearLine.get().distToPoint2(currentPosition);

    if (distToPoint == -1.f)
      return -1.f;

    drawLine(currentPosition, middle, _rosColors[0]);
    tf2::Vector3 pointOnLine = nearLine.get().porjectPoint(currentPosition);

    drawObjPose(pointOnLine, _rosColors[4]);

    tf2::Vector3 fromPointToLine = pointOnLine - currentPosition;

    float euclidian = fromPointToLine.length();
    float chebyshev = std::max(std::max(fromPointToLine.x(), fromPointToLine.y()), fromPointToLine.z());

    float c = (currentPosition - cameraPosition).length2();
    float aCath = distToPoint;
    float sinMetric = aCath / c;
    float bCath = (cameraPosition - pointOnLine).length2();
    float cosMetric = bCath / c;

    float totalDistToObject = (cameraPosition - _firstObjPose).length2();
    float passDist = (_firstObjPose - pointOnLine).length2();
    float passDistMetric = passDist / totalDistToObject;

    /* if (_prevDist != 0) {
      float deltaDist = euclidian - _prevDist;
      double deltaTime = (ros::Time::now() - _aTimer).toSec();
      ROS_INFO("a %lf", deltaDist / deltaTime);
    }

    _prevDist = euclidian;
    _aTimer = ros::Time::now(); */

    float avrMetric = (cosMetric + passDistMetric) / 2.f;

    ROS_DEBUG_NAMED("dist_to_line", "DistToLine: %f", sqrt(distToPoint));
    ROS_DEBUG_NAMED("euclidian", "Euclidian %f", euclidian);
    ROS_DEBUG_NAMED("chebyshev", "Chebyshev %f", chebyshev);
    ROS_DEBUG_NAMED("sin", "Sin %f", 1.f - sinMetric);
    ROS_DEBUG_NAMED("cos", "Cos %f", cosMetric);
    ROS_DEBUG_NAMED("pass_dist", "PossDist %f", passDistMetric);
    ROS_DEBUG_NAMED("avr", "Avr %f", avrMetric);

    return 1;
  }
};

#endif // _VISION_STATE_TRACKING_H_
