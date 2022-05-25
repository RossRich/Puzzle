#if !defined(_VISION_STATE_TRACKING_H_)
#define _VISION_STATE_TRACKING_H_
#define PZ_GRAVITY 9.8f

#include <puzzle_msgs/Metrics.h>
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
#include <tf2/utils.h>

#include "../utils/Line.hpp"
#include "BallTracking.hpp"
#include "BallTrackingRos.hpp"
#include "RosVH.hpp"
#include "State.hpp"
#include "VideoHandler.hpp"
#include "utils/RvizPainter.hpp"
#include "utils/RvizPainterObject.hpp"
#include "utils/Utils.hpp"
#include "utils/thresholdtype.hpp"

using geometry_msgs::Pose;
using geometry_msgs::TransformStamped;
using image_geometry::PinholeCameraModel;
using sensor_msgs::CameraInfo;
using sensor_msgs::CameraInfoConstPtr;
using puzzle_msgs::Metrics;


class StateTracking : public State {
private:
  int _fps = 0;
  bool _isObjDetected = false;
  bool _isTrekLinePredicted = false;
  float _filterGain = 0.65f;
  const char *_winName = "Tracking";
  float _prevDist = 0.f;
  float _dt4prediction = 0.01f;

  BallTracking *_bt = nullptr;
  RosVH *_vh = nullptr;

  std::string _confFile = "";
  std::string _cameraInfoTopic = "/camera/color/camera_info";
  std::list<tf2::Vector3> _realTrajPoints;
  std::list<tf2::Vector3> _predTrajectory;
  std::vector<Line> _predictedSigments;

  ros::NodeHandle &_nh;
  ros::Publisher _metricsPublisher;
  ros::Time _loopTimer;
  ros::Time _startTrackingTimer;
  ros::Time _resetTimer;
  ros::Time _detectionTimer;
  ros::Time _aTimer;

  tf2_ros::Buffer _tfBuffer;
  tf2_ros::TransformListener *_tfListener;
  tf2::Vector3 _firstObjPosition;
  tf2::Vector3 _lastObjPosition;
  image_transport::ImageTransport *_it;
  CameraInfoConstPtr _cameraInfo;
  PinholeCameraModel _cameraModel;

  RvizPainter *_rvizPainter; //< TODO: smart pointer
  RvizPainterObject _rvizPainterObject;

  cv::Mat _frame;
  cv::Mat _depth;

  void transformPose(tf2::Vector3 &position);
  Pose transformPose2(tf2::Vector3 &position, const tf2::Quaternion &orientation);
  void conceptOne(cv::Mat &mask, cv::Point2i &center, uint16_t &radius);
  void conceptTwo(cv::Mat &mask, cv::Point2i &point2d, uint16_t &radius);

  /**
   * Builds a 3D object position from 2D Image via camera model
   *
   * @param[in] point2d cv::Point2i - 2d point
   * @param[in] distToObj distance to interest object
   * @param[out] objPos tf2::Vector3 - 3d point
   **/
  void getObjPosFromImg(cv::Point2i &point2d, float distToObj, tf2::Vector3 &objPos);
  
  float getDistToObj(cv::Mat &mask, uint16_t &radius);
  float getVelocity(float x, float y, float angle);
  float getContactProbobility(const tf2::Vector3 &currentObjPosition, const tf2::Vector3 &lastObjPosition, const tf2::Vector3 &cameraPosition);

public:
  StateTracking(BallTrackingRos &context, ros::NodeHandle &nh) : State(context, "Tracking"), _nh(nh) {}
  ~StateTracking();

  bool loadParam();
  bool setup();

  void tracking() override {}
  void wait() override;
  void execute() override;
};

#endif // _VISION_STATE_TRACKING_H_
