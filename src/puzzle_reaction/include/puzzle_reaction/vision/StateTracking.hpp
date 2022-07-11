#if !defined(_VISION_STATE_TRACKING_H_)
#define _VISION_STATE_TRACKING_H_
#define PZ_GRAVITY 9.8f

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <puzzle_common/GyverFilters/src/filters/RingAverage.h>
#include <puzzle_common/GyverFilters/src/filters/median3.h>
#include <puzzle_common/RvizPainter.hpp>
#include <puzzle_msgs/Metrics.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_srvs/Empty.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "BallTracking.hpp"
#include "BallTrackingRos.hpp"
#include "RosVH.hpp"
#include "State.hpp"
#include "VideoHandler.hpp"
#include "puzzle_reaction/utils/Line.hpp"
#include "utils/RvizPainterObject.hpp"
#include "utils/Utils.hpp"
#include "utils/thresholdtype.hpp"

using cv_bridge::CvImage;
using geometry_msgs::Pose;
using geometry_msgs::TransformStamped;
using image_geometry::PinholeCameraModel;
using puzzle_msgs::Metrics;
using sensor_msgs::CameraInfo;
using sensor_msgs::CameraInfoConstPtr;
using geometry_msgs::Vector3StampedConstPtr;
using geometry_msgs::Vector3StampedPtr;
using geometry_msgs::Vector3Stamped;

class StateTracking : public State {
private:
  int _fps = 0;
  bool _isObjDetected = false; ///< TODO: rename to _isStartedPredict
  bool _isTrekLinePredicted = false;
  bool _isSoftFiltringEnabled = false;
  bool _isConturFinded = false; ///< TODO: rename to _isObjDetected
  float _filterGain = 0.65f;
  float _prevDist = 0.f;
  float _dt4prediction = 0.01f;
  float _lastDist = 0.f;

  int _maxDist = 5000; ///< max dist in mm
  int _minDist = 250;  ///< min dist in mm
  uint8_t _filterSize = 3;

  std::unique_ptr<BallTracking> _bt;
  std::unique_ptr<RosVH> _vh;

  std::string _confFile = "";
  std::string _cameraInfoTopic = "";
  std::list<tf2::Vector3> _realTrajPoints;
  std::list<tf2::Vector3> _predTrajectory;
  std::vector<Line> _predictedSigments;

  ros::NodeHandle &_nh;
  ros::Publisher _metricsPublisher;
  ros::Subscriber _arucoTopic; 
  ros::Time _loopTimer;
  ros::Time _startTrackingTimer;
  ros::Time _resetTimer;
  ros::Time _detectionTimer;
  ros::Time _aTimer;

  tf2_ros::Buffer _tfBuffer;
  std::unique_ptr<tf2_ros::TransformListener> _tfListener;
  tf2::Vector3 _firstObjPosition;
  tf2::Vector3 _test2222;
  tf2::Vector3 _lastObjPosition;
  std::shared_ptr<image_transport::ImageTransport> _it;
  CameraInfoConstPtr _cameraInfo;
  PinholeCameraModel _cameraModel;
  image_transport::Publisher _debugPub;
  image_transport::Publisher _resultPub;

  std::unique_ptr<RvizPainter> _rvizPainter;
  RvizPainterObject _rvizPainterObject;
  RingAverage<uint16_t, 10> _distanseFilter;
  GMedian3<uint16_t> _medianFilter;

  // cv::Mat _frame;
  // cv::Mat _depth;

  ///< TMP section
  ros::Time _objMoveTimer;
  tf2::Vector3 cameraPosition;
  tf2::Quaternion cameraOrientation;
  uint8_t safePoints, totalPoints;
  float pointDist2 = 0.0225f;
  Vector3Stamped _arucoPosition;

  void arucoPosCallback(const Vector3StampedConstPtr &arucoPosition);

  void transformPose(tf2::Vector3 &position);
  Pose transformPose2(const tf2::Vector3 &position, const tf2::Quaternion &orientation);
  Pose transformPose3(const tf2::Vector3 &position, const tf2::Quaternion &orientation);
  void conceptThree(tf2::Vector3 &objPosition, uint16_t &radius);

  /**
   * Builds a 3D object position from 2D Image via camera model
   *
   * @param[in] point2d cv::Point2i - 2d point
   * @param[in] distToObj distance to interest object
   * @param[out] objPos tf2::Vector3 - 3d point
   **/
  void getObjPosFromImg(cv::Point2f &point2d, float distToObj, tf2::Vector3 &objPos);

  float getDistToObj(cv::Mat &depth, cv::Mat &mask, uint16_t &radius);
  float getVelocity(float x, float y, float angle);
  float getContactProbobility(const tf2::Vector3 &currentObjPosition, const tf2::Vector3 &lastObjPosition, const tf2::Vector3 &cameraPosition);
  void trajectoryPrediction(const tf2::Vector3 &cameraPosition, const tf2::Quaternion &cameraOrientation, uint8_t safePoints, uint8_t totalPoints, float pointDist2);
  void approxQuadratic(std::vector<tf2::Vector3> &in, std::vector<float> &out, uint8_t newPoints, float dist);
  void approxLinear(std::vector<tf2::Vector3> &in, std::vector<float> &out, uint8_t newPoints);
  float calcPlane(const tf2::Vector3 &p1, const tf2::Vector3 &p2, const tf2::Vector3 &p3, const tf2::Vector3 &targetPosition);
  void reset();

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
