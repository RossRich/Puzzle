#include "puzzle_reaction/vision/StateTracking.hpp"

StateTracking::~StateTracking() { std::cout << "Delete state tracking\n"; }

bool StateTracking::loadParam() {

  if (!_nh.getParam("data_config", _confFile)) {
    ROS_ERROR("[StateTracking] No data_config param");
    return false;
  }

  if (!_nh.getParam("camera_info_topic", _cameraInfoTopic)) {
    ROS_ERROR("[StateTracking] Topic with camera info not set");
    return false;
  }

  ROS_INFO("[StateTracking] Reads camera info from topic \'%s\'  ...", _cameraInfoTopic.c_str());
  try {
    _cameraInfo = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(_cameraInfoTopic, _nh);
  } catch (const ros::Exception &e) {
    ROS_ERROR("[StateTracking] %s", e.what());
    ROS_ERROR("[StateTracking] Failed to get camera info. Exit.");
    return false;
  }

  if (!_cameraInfo || _cameraInfo->width <= 0 || _cameraInfo->height <= 0) {
    ROS_ERROR("[StateTracking] Failed to get camera info. Exit.");
    return false;
  }

  if (!_nh.getParam("filter_gain", _filterGain)) {
    ROS_INFO("[StateTracking] No filter_gain param. Use default value %f", _filterGain);
  }

#if defined(BALL_TRACKING_FILTER) && BALL_TRACKING_FILTER == KALMAN
  if (!_nh.getParam("dt_4_kalman", _dt4kalman)) {
    ROS_INFO("[StateTracking] No dt_4_kalman param. Use default value %f", _dt4kalman);
  }
#endif // BALL_TRACKING_FILTER

  if (!_nh.getParam("max_dist", _maxDist)) {
    ROS_INFO("[StateTracking] No max_dist param. Use default value %i", _maxDist);
  }

  if (!_nh.getParam("min_dist", _minDist)) {
    ROS_INFO("[StateTracking] No _minDist param. Use default value %i", _minDist);
  }

  return true;
}

bool StateTracking::setup() {
  threshold_t threshold;
  if (!Utils::readThresholds(_confFile.c_str(), threshold)) {
    ROS_ERROR("[StateTracking] Failed to read color treshold. Exit.");
    return false;
  }

  if (!_cameraModel.initialized()) {
    if (!_cameraModel.fromCameraInfo(_cameraInfo)) {
      ROS_ERROR("[StateTracking] Failed to build model of camera. Exit.");
      return false;
    }
  }

  if (!_it)
    _it = std::make_shared<image_transport::ImageTransport>(_nh);

  if (!_vh) {
    try {
      _vh = std::make_unique<RosVH>(_nh, *_it, _cameraInfo->width, _cameraInfo->height);
    } catch (const ros::Exception &e) {
      ROS_ERROR_STREAM("[StateTracking] " << e.what());
      return false;
    }
  }

  if (_arucoTopic.getTopic().empty()) {
    _arucoTopic = _nh.subscribe("/aruco_single/position", 3, &StateTracking::arucoPosCallback, this);
  }

  if (_objPositionPub.getTopic().empty()) {
    _objPositionPub = _nh.advertise<Point>("obj_position", 1);
  }

  if (!_bt)
    _bt = std::make_unique<BallTracking>(_vh->getWidth(), _vh->getHeight(), threshold);

  if (!_tfListener)
    _tfListener = std::make_unique<tf2_ros::TransformListener>(_tfBuffer, _nh);

  if (!_rvizPainter)
    _rvizPainter = std::make_unique<RvizPainter>(_nh, "puzzle_reaction_painter");

  if (_metricsPub.getTopic().empty()) {
    _metricsPub = _nh.advertise<puzzle_msgs::Metrics>("metrics", 1500);
  }

  if (_debugPub.getTopic().empty()) {
    _debugPub = _it->advertise("ball_tracking_debug", 1, true);
  }

  if (_resultPub.getTopic().empty()) {
    _resultPub = _it->advertise("ball_tracking_result", 1, true);
  }

  _loopTimer = ros::Time::now();
  debugImgPubRate = ros::Time::now();

  reset();

#if defined(BALL_TRACKING_FILTER) && BALL_TRACKING_FILTER == KALMAN
  ROS_INFO("Kalman filter setup done! Filter gain = %f; dt = %f.", _filterGain, _dt4kalman);
  _kalmanFilter.setParameters(_filterGain, _dt4kalman);
#else
  ROS_INFO("Running average filter setup done! Filter gain = %f.", _filterGain);
  _runningAvrFilter.setCoef(_filterGain);
#endif // BALL_TRACKING_FILTER

  return true;
}

void StateTracking::wait() {
  ROS_INFO("[StateTracking] Transition from %s state to Wait state", toString().c_str());
  _context.setState(static_cast<State *>(_context.getStateWait()));
}

void StateTracking::arucoPosCallback(const Vector3StampedConstPtr &arucoPosition) {
  _arucoPosition.header = arucoPosition->header;
  _arucoPosition.vector = arucoPosition->vector;
}

float StateTracking::getDistToObj(const cv::Mat &depth, const cv::Mat &mask, const cv::Point2f &center, const uint16_t &radius) {
  uint16_t deametr = radius + radius;
  if (deametr > 25)
    deametr = 25;

  float x = center.x - radius;
  if (x < 0)
    x = 0;

  float y = center.y - radius;
  if (y < 0)
    y = 0;

  float width = deametr;
  if (x + width > depth.cols)
    width = depth.cols - x;

  float height = deametr;
  if (y + height > depth.rows)
    height = depth.rows - y;

  return getDistToObj(depth, cv::Rect2f(x, y, width, height));
}

float StateTracking::getDistToObj(const cv::Mat &depth, const cv::Rect2f &roi) {
  cv::Mat ballDists(depth, roi);
  ros::Time depthHandle = ros::Time::now();
  // ROS_DEBUG_STREAM_THROTTLE(1.0, "Depth oroigin size: " << depth.size() << " ball dist size: " << ballDists.size());

  uint16_t *bestDist1 = nullptr, *bestDist2 = nullptr, *bestDist3 = nullptr;
  uint16_t maxCount1 = 0, maxCount2 = 0, maxCount3 = 0;

  std::map<uint16_t, uint16_t> mapOfDist; ///< <Dist, NumOfTheDist>

  uint16_t *cols;
  uint16_t dist = 0;

  for (size_t i = 0; i < ballDists.rows; ++i) {
    cols = ballDists.ptr<uint16_t>(i);
    for (size_t j = 0; j < ballDists.cols; ++j) {
      dist = cols[j];

      if (dist < _minDist || dist > _maxDist) ///< min dist = StateTracking::_minDist mm, max dist = StateTracking::_maxDist mm
        continue;

      if (mapOfDist.count(dist) > 0) {
        auto distIter = mapOfDist.find(dist);
        uint16_t num = distIter->second + 1;
        distIter->second = num;

        if (num > maxCount1) {
          maxCount1 = num;
          bestDist1 = cols + j;
        }

        if (num > maxCount2 && cols + j != bestDist1 && cols + j != bestDist3) {
          maxCount2 = num;
          bestDist2 = cols + j;
        }

        if (num > maxCount3 && cols + j != bestDist1 && cols + j != bestDist2) {
          maxCount3 = num;
          bestDist3 = cols + j;
        }

      } else
        mapOfDist.insert({dist, 1});
    }
  }

  dist = 0;      ///< reuse dist variable
  maxCount1 = 0; ///< reuse dist variable

  if (bestDist1 != nullptr) {
    dist += *bestDist1;
    ++maxCount1;
  }

  if (bestDist2 != nullptr) {
    dist += *bestDist2;
    ++maxCount1;
  }

  if (bestDist3 != nullptr) {
    dist += *bestDist3;
    ++maxCount1;
  }

  if (maxCount1 != 0) {
    dist /= maxCount1;

    float filtredDist;
#if defined(BALL_TRACKING_FILTER) && BALL_TRACKING_FILTER == KALMAN
    // filtredDist = _kalmanFilter.filtered(dist);
#else
    // filtredDist = _runningAvrFilter.filtered(dist);
#endif // BALL_TRACKING_FILTER

    std::stringstream info2;
    info2 << "MapOfDistDump:";
    for (auto i : mapOfDist) {
      info2 << "\n\tcount: " << i.second << "\tdist: " << i.first;
    }

    /* info2 << "\nBsetDists:"
          << "\n\tbest_dist1: " << *bestDist1
          << "\n\tbest_dist2: " << *bestDist2
          << "\n\tbest_dist3: " << *bestDist3 << "\n";
   */
    ROS_DEBUG_STREAM_THROTTLE(0.5, info2.rdbuf());

    /*
        Point objPositionMsg;
        objPositionMsg.x = dist;
        objPositionMsg.y = filtredDist;

        if (std::fabs(filtredDist - dist) < 500.0f && dist != 0) {
          dist = filtredDist;
          _isSoftFiltringEnabled = true;
        } else {
          _isSoftFiltringEnabled = false;
        }

        objPositionMsg.z = 3000 + (500 * _isSoftFiltringEnabled);
        _objPositionPub.publish(objPositionMsg);
        */
  }

  ROS_DEBUG_STREAM_THROTTLE(.5, "Depth handle: " << ros::Duration(ros::Time::now() - depthHandle).toSec() * 1000.0);

  return dist * 0.001f;
}

void StateTracking::getObjPosFromImg(cv::Point2f &point2d, float distToObj, tf2::Vector3 &objPos) {
  cv::Point3d point3d = _cameraModel.projectPixelTo3dRay(point2d) * distToObj;
  objPos.setValue(point3d.x, point3d.y, point3d.z);
}

// for camera to map
void StateTracking::transformPose(tf2::Vector3 &position) {
  TransformStamped transformation = _tfBuffer.lookupTransform("map", _cameraModel.tfFrame(), ros::Time(0)); ///< Take the transformation from camera_frame (frame from camera_info) to map

  tf2::Transform originalTransform(tf2::Quaternion::getIdentity(), position);
  tf2::Stamped<tf2::Transform> ts(originalTransform, ros::Time::now(), _cameraModel.tfFrame());

  TransformStamped newTransform;
  tf2::doTransform(tf2::toMsg(ts), newTransform, transformation);

  tf2::convert(newTransform.transform.translation, position);
}

// from base_link_frd to map
Pose StateTracking::transformPose2(const tf2::Vector3 &position, const tf2::Quaternion &orientation) {
  TransformStamped transformation = _tfBuffer.lookupTransform("map", "base_link_frd", ros::Time(0));

  tf2::Transform originalTransform(orientation, position);
  tf2::Stamped<tf2::Transform> ts(originalTransform, ros::Time::now(), "base_link_frd");

  TransformStamped newTransform;
  tf2::doTransform(tf2::toMsg(ts), newTransform, transformation);

  Pose pose;
  pose.orientation = newTransform.transform.rotation;
  pose.position.x = newTransform.transform.translation.x;
  pose.position.y = newTransform.transform.translation.y;
  pose.position.z = newTransform.transform.translation.z;

  return pose;
}

// form map to base_link_frd
Pose StateTracking::transformPose3(const tf2::Vector3 &position, const tf2::Quaternion &orientation) {
  TransformStamped transformation = _tfBuffer.lookupTransform("base_link_frd", "map", ros::Time(0));

  tf2::Transform originalTransform(orientation, position);
  tf2::Stamped<tf2::Transform> ts(originalTransform, ros::Time::now(), "map");

  TransformStamped newTransform;
  tf2::doTransform(tf2::toMsg(ts), newTransform, transformation);

  Pose pose;
  pose.orientation = newTransform.transform.rotation;
  pose.position.x = newTransform.transform.translation.x;
  pose.position.y = newTransform.transform.translation.y;
  pose.position.z = newTransform.transform.translation.z;

  return pose;
}

float StateTracking::getVelocity(float x, float y, float angle) {
  float num = PZ_GRAVITY * powf(x, 2);
  float den = 2.0f * (y - tanf(angle) * x) * powf(cosf(angle), 2);
  if (den < 1.0e-3f && den > -1.0e-3f)
    den = 1.0f;

  ROS_DEBUG_STREAM_NAMED("[getVelocity] num", "num" << num);
  ROS_DEBUG_STREAM_NAMED("[getVelocity] den", "den" << den);
  float v2 = num / den;
  return sqrtf(abs(v2));
}

float StateTracking::getContactProbobility(const tf2::Vector3 &currentObjPosition, const tf2::Vector3 &lastObjPosition,
                                           const tf2::Vector3 &cameraPosition) {
  if (_predictedSigments.size() == 0)
    return -1;

  float minDist = 100.0f;
  auto nearLine = std::ref(_predictedSigments[0]);
  auto middle = std::cref(nearLine.get().getMidpoint());
  for (auto &line : _predictedSigments) {
    const tf2::Vector3 &tmpMiddle = line.getMidpoint();
    float tmpDist = tf2::tf2Distance2(tmpMiddle, currentObjPosition);

    if (tmpDist < minDist) {
      nearLine = std::ref(line);
      middle = std::cref(tmpMiddle);
      minDist = tmpDist;
    }
  }

  float dist2ToPoint = nearLine.get().dist2ToPoint(currentObjPosition);

  if (dist2ToPoint == -1.f)
    return -1.f;

  tf2::Vector3 pointOnLine = nearLine.get().porjectPoint(currentObjPosition);
  _rvizPainter->draw(_rvizPainterObject.getPointOnTraj(), pointOnLine);
  _rvizPainter->draw(_rvizPainterObject.getShortABlueLine(), currentObjPosition, middle);

  tf2::Vector3 lineDir;
  if (cameraPosition.z() - _firstObjPosition.z() < 0)
    lineDir = nearLine.get().getP1() - nearLine.get().getP2();
  else
    lineDir = nearLine.get().getP2() - nearLine.get().getP1();

  float yawArwOfPred = atan2f(lineDir.normalized().y(), lineDir.normalized().x());
  float pitchArwOfPred = asinf(lineDir.normalized().z());
  tf2::Quaternion vectorOrientation;
  vectorOrientation.setRPY(0.f, -pitchArwOfPred, yawArwOfPred);
  _rvizPainter->draw(_rvizPainterObject.getVecArrowOfPred(), currentObjPosition, vectorOrientation);

  tf2::Vector3 fromNewPosToLast = currentObjPosition - lastObjPosition;
  float yawArwOfReal = atan2f(fromNewPosToLast.normalized().y(), fromNewPosToLast.normalized().x());
  float pitchArwOfReal = asinf(fromNewPosToLast.normalized().z());
  tf2::Quaternion orientArwOfReal;
  orientArwOfReal.setRPY(0.f, -pitchArwOfReal, yawArwOfReal);
  _rvizPainter->draw(_rvizPainterObject.getVecArrowOfReal(), currentObjPosition, orientArwOfReal);

  // metrics

  tf2::Vector3 firstToCamera = cameraPosition - _firstObjPosition;
  tf2::Vector3 firstToCurrent = currentObjPosition - _firstObjPosition;
  tf2::Vector3 firstToPointOnLine = pointOnLine - _firstObjPosition;
  tf2::Vector3 currentToPointOnLine = pointOnLine - currentObjPosition;

  float vectorMetric = tf2::tf2Dot(lineDir, fromNewPosToLast);
  float vector2Metric = tf2::tf2Dot(firstToPointOnLine.normalized(), firstToCurrent.normalized());
  float vectorNormMetric = tf2::tf2Dot(lineDir.normalized(), fromNewPosToLast.normalized());

  float chebyshev = std::max(std::max(currentToPointOnLine.x(), currentToPointOnLine.y()), currentToPointOnLine.z());

  float c = firstToCurrent.length2();
  float sinMetric = sqrtf(dist2ToPoint / c);

  float totalDistToObject = firstToCamera.length2();
  float passDist = firstToPointOnLine.length2();
  float passDistMetric = sqrtf(passDist / totalDistToObject);
  float avrMetric = vectorMetric + passDistMetric;

  Metrics metricsMsg;
  metricsMsg.header.frame_id = "map";
  metricsMsg.header.stamp = ros::Time::now();

  metricsMsg.avr = avrMetric;
  metricsMsg.chebyshev = chebyshev;
  metricsMsg.sin = 1.f - sinMetric;
  metricsMsg.dist_to_line = sqrtf(dist2ToPoint);
  metricsMsg.vector = vectorMetric;
  metricsMsg.vector2 = vector2Metric;
  metricsMsg.vector_norm = vectorNormMetric;
  metricsMsg.pass_dist = passDistMetric;

  _metricsPub.publish(metricsMsg);
  return 1;
}

void StateTracking::trajectoryPrediction(const tf2::Vector3 &cameraPosition, const tf2::Quaternion &cameraOrientation,
                                         uint8_t safePoints, uint8_t totalPoints, float pointDist2) {
  if (_realTrajPoints.size() >= 5) {
    safePoints = 5;
  } else if (_realTrajPoints.size() < 5 && _realTrajPoints.size() > 1) {
    safePoints = _realTrajPoints.size();
  } else {
    ROS_ERROR_STREAM("Too few poinst for estimate object trajectory.");
    return;
  }

  ROS_DEBUG_STREAM("totalPoints: " << (int)totalPoints);
  ROS_DEBUG_STREAM("safePoints: " << (int)safePoints);

  std::vector<float> abc;
  std::vector<tf2::Vector3> points;
  uint8_t counter = 0;
  ROS_ASSERT(_realTrajPoints.size() >= safePoints);
  for (auto &point : _realTrajPoints) {
    switch (safePoints) {
    case 5:
      if (!(counter & 1)) ///< point: 0, 2, 4
        points.push_back(point);
      break;
    case 4:
      if (counter == 0 || counter == 1 || counter == 3)
        points.push_back(point);
      break;
    case 3:
      points.push_back(point);
      break;
    case 2: ///< TODO: Linear approximation
      points.push_back(point);
      break;
    default: ///< TODO: ???
      break;
    }
    ++counter;

    if (counter == safePoints)
      break;
  }

  if (points.empty()) {
    ROS_ERROR_STREAM("Points arr for approximation is empty");
    return;
  }

  if (points.size() > 2) {
    float safeZone = calcPlane(points[0], points[1], points[2], cameraPosition);
    ROS_DEBUG_STREAM("Plane factor " << safeZone);
    // if(abs(safeZone) < 0.25f) {
    approxQuadratic(points, abc, totalPoints, sqrtf(pointDist2));

    Pose cameraInBaseLink = transformPose3(cameraPosition, cameraOrientation);
    tf2::Vector3 cameraInBaseLinkPosition;
    tf2::fromMsg(cameraInBaseLink.position, cameraInBaseLinkPosition);
    float z = (abc[0] * powf(cameraInBaseLinkPosition.x(), 2) + abc[1] * cameraInBaseLinkPosition.x() + abc[2]) + cameraInBaseLinkPosition.z();
    cameraInBaseLinkPosition.setZ(z);
    cameraInBaseLink = transformPose2(cameraInBaseLinkPosition, tf2::Quaternion::getIdentity());
    tf2::fromMsg(cameraInBaseLink.position, cameraInBaseLinkPosition);
    tf2::Vector3 objDir = (points[2] - points[0]).normalize();
    cameraInBaseLinkPosition = objDir * tf2::tf2Distance(points[0], cameraInBaseLinkPosition) + points[0];
    cameraInBaseLinkPosition.setZ(cameraInBaseLink.position.z);
    _rvizPainter->draw(_rvizPainterObject.getIntersectPosition(), cameraInBaseLinkPosition);
    float distToDrone = tf2::tf2Distance(cameraInBaseLinkPosition, cameraPosition);
    ROS_DEBUG_STREAM("DistToDrone " << distToDrone);
    ROS_DEBUG("Intersect state %i", distToDrone <= abs(safeZone));
    // }

  } else
    approxLinear(points, abc, totalPoints);

  for (auto &point : points) {
    ROS_DEBUG_STREAM("point:\n"
                     << point.x() << "\n"
                     << point.y() << "\n"
                     << point.z());
  }

  for (auto &k : abc) {
    ROS_DEBUG("%f", k);
  }

  _rvizPainter->draw(_rvizPainterObject.getPredTrajLine(), _predTrajectory);
}

void StateTracking::approxQuadratic(std::vector<tf2::Vector3> &in, std::vector<float> &out, uint8_t newPoints, float dist) {
  if (in.size() < 3) {
    ROS_ERROR_STREAM(
        "[StateTracking::approxQuadratic] Input points number not enother for quadratic approximation. Total points = "
        << in.size());
    return;
  }

  Pose pp1 = transformPose3(in[0], tf2::Quaternion::getIdentity());
  Pose pp2 = transformPose3(in[1], tf2::Quaternion::getIdentity());
  Pose pp3 = transformPose3(in[2], tf2::Quaternion::getIdentity());

  tf2::Vector3 p1, p2, p3;
  tf2::fromMsg(pp1.position, p1);
  tf2::fromMsg(pp2.position, p2);
  tf2::fromMsg(pp3.position, p3);

  float num = p3.z() - (((p3.x() * (p2.z() - p1.z())) + (p2.x() * p1.z()) - (p1.x() * p2.z())) / (p2.x() - p1.x()));
  float den = (p3.x() * (p3.x() - p1.x() - p2.x())) + (p1.x() * p2.x());
  out.push_back(num / den); ///< TODO: division by zero
  out.push_back(((p2.z() - p1.z()) / (p2.x() - p1.x())) - (out[0] * (p1.x() + p2.x())));
  out.push_back((((p2.x() * p1.z()) - (p1.x() * p2.z())) / (p2.x() - p1.x())) + (out[0] * p1.x() * p2.x()));

  tf2::Vector3 dir = (p3 - p1).normalize();
  // Pose testPose = transformPose3(in[0], tf2::Quaternion::getIdentity());
  // tf2::Vector3 trFirstPoint;
  // tf2::fromMsg(testPose.position, trFirstPoint);
  float xx = p1.x();
  float distance;
  tf2::Vector3 approxQuat = p1;
  for (size_t i = 0; i < newPoints; i++) {
    float z = out[0] * powf(xx, 2) + out[1] * xx + out[2];
    approxQuat.setX(xx);
    distance = tf2::tf2Distance(p1, approxQuat);
    approxQuat = dir * distance + p1;
    approxQuat.setZ(z);
    xx -= dist;

    tf2::Vector3 poseInMap;
    Pose msgPoseInMap = transformPose2(approxQuat, tf2::Quaternion::getIdentity());
    tf2::fromMsg(msgPoseInMap.position, poseInMap);
    _predTrajectory.push_back(poseInMap);
  }
}

void StateTracking::approxLinear(std::vector<tf2::Vector3> &in, std::vector<float> &out, uint8_t newPoints) {
  if (in.size() < 2) {
    ROS_ERROR_STREAM("[StateTracking::approxLinear] Input points number not enother for linear approximation. Total points = " << in.size());
    return;
  }

  Pose pp1 = transformPose3(in[0], tf2::Quaternion::getIdentity());
  Pose pp2 = transformPose3(in[1], tf2::Quaternion::getIdentity());

  tf2::Vector3 p1, p2;
  tf2::fromMsg(pp1.position, p1);
  tf2::fromMsg(pp2.position, p2);

  out.push_back((p1.z() - p2.z()) / (p1.x() - p2.x())); ///< TODO: division by zero
  out.push_back(p2.z() - out[0] * p2.x());

  float xx = _firstObjPosition.x();
  tf2::Vector3 approxQuat = _firstObjPosition;
  for (size_t i = 0; i < newPoints; i++) {
    float z = out[0] * xx + out[1];
    approxQuat.setX(xx);
    approxQuat.setZ(z);
    xx += sqrtf(sqrtf(tf2::tf2Distance2(p1, p2)));

    Pose msgPoseInMap = transformPose2(approxQuat, tf2::Quaternion::getIdentity());
    tf2::Vector3 poseInMap;
    tf2::fromMsg(msgPoseInMap.position, poseInMap);
    _predTrajectory.push_back(poseInMap);
  }
}

float StateTracking::calcPlane(const tf2::Vector3 &p1, const tf2::Vector3 &p2, const tf2::Vector3 &p3,
                               const tf2::Vector3 &targetPosition) {
  tf2::Vector3 planeNormal = tf2::tf2Cross(p2 - p1, p3 - p1).normalize();
  tf2::Vector3 planeNormalUp = tf2::tf2Cross(p2 - p1, planeNormal).normalize();

  float gridSize = 0.25f;
  int distToDrone = std::roundf(tf2::tf2Distance(targetPosition, p1) / gridSize) + 1;
  uint16_t row = 10;

  tf2::Vector3 startPoint = (row / 2.f * gridSize) * (-planeNormalUp) + p1;

  std::list<tf2::Vector3> planePoints;
  for (uint16_t i = 0; i < distToDrone; i++) {
    tf2::Vector3 pp = (p2 - p1).normalized() * (i * gridSize) + startPoint;
    for (uint16_t j = 0; j < row; j++) {
      planePoints.push_back(planeNormalUp * (j * gridSize) + pp);
    }
  }
  _rvizPainter->draw(_rvizPainterObject.getTrajectoryPlane(), planePoints);
  return tf2::tf2Dot(planeNormal, targetPosition) - tf2::tf2Dot(planeNormal, p3);
}

void StateTracking::conceptThree(tf2::Vector3 &objPosition, uint16_t &radius) {

  /* if (radius == 0 && _realTrajPoints.empty())
    return;

  if (radius == 0 && !_realTrajPoints.empty()) {
    double timeOut = ros::Duration(ros::Time::now() - _resetTimer).toSec();
    ROS_DEBUG_THROTTLE(.5, "timeOut %lf", timeOut);
    if (timeOut >= 0.1) {
      if (_isObjDetected && !_isTrekLinePredicted) {
        ROS_DEBUG("predict");
        trajectoryPrediction(cameraPosition, cameraOrientation, safePoints, totalPoints, pointDist2);
        _isTrekLinePredicted = true;
      }
    }
    return;
  } */

  if (!_isObjDetected) {
    _firstObjPosition = objPosition;
    _isObjDetected = true;
    _realTrajPoints.push_back(objPosition);
    _rvizPainter->draw(_rvizPainterObject.getObjFirstPosition(), _firstObjPosition);
    _startTrackingTimer = ros::Time::now();
    return;
  }

  if (tf2::tf2Distance2(_firstObjPosition, objPosition) > 0.0225f && _objMoveTimer.is_zero()) {
    _objMoveTimer = ros::Time::now();
  }

  tf2::Vector3 lastPointInRealTrajectory = _realTrajPoints.back();
  if (tf2::tf2Distance2(lastPointInRealTrajectory, objPosition) < 0.0225f)
    return;

  if (_realTrajPoints.size() > 20) ///< TODO: add to launch param
    _realTrajPoints.pop_front();

  _realTrajPoints.push_back(objPosition);
  _rvizPainter->update(_rvizPainterObject.getRealTrajLine(), _realTrajPoints); ///< TODO: delete from rviz old point

  // if (_realTrajPoints.size() == 2)
  // _test2222 = objPosition;

  // if (_realTrajPoints.size() < 3)
  // return;

  TransformStamped transform = _tfBuffer.lookupTransform("map", "base_link_frd", ros::Time(0));
  // tf2::Quaternion cameraOrientation; ///< camera orientation in map
  // tf2::Vector3 cameraPosition;       ///< camera position in map
  tf2::fromMsg(transform.transform.rotation, cameraOrientation);
  tf2::fromMsg(transform.transform.translation, cameraPosition);

  float totalDist2 = tf2::tf2Distance2(_firstObjPosition, cameraPosition);
  // pointDist2 = 0.15f;
  // float pointDist2 = tf2::tf2Distance2(_firstObjPosition, _test2222);
  totalPoints = std::floor(sqrtf(totalDist2 / pointDist2));
  // uint8_t totalPoints = std::floor(sqrtf(totalDist2 / pointDist2));
  safePoints = std::floor(totalPoints / 2.f);
  // uint8_t safePoints = std::floor(totalPoints / 2.f);
  if (safePoints > 5)
    safePoints = 5; ///< TODO: not good idea

  float safeDist = tf2::tf2Distance2(cameraPosition, objPosition) / totalDist2;
  ROS_DEBUG_STREAM("safeDist: " << safeDist);
  ROS_DEBUG_STREAM("pointsInTraj: " << _realTrajPoints.size());
  ROS_DEBUG_STREAM("objMoveTimer: " << ros::Duration(ros::Time::now() - _objMoveTimer).toSec());

  if (safeDist > .5f && ros::Duration(ros::Time::now() - _objMoveTimer).toSec() < 0.1) {
    return;
  }

  // _rvizPainter->draw(_rvizPainterObject.getRegArrow(), cameraPosition, cameraOrientation);

  // tf2::Vector3 camToObjDirection = _firstObjPosition - cameraPosition; ///< vector from camera to detected obj
  // tf2::Vector3 camToObjDirXY = (cameraPosition + tf2::Vector3(camToObjDirection.x(), camToObjDirection.y(), 0)) -
  // cameraPosition; ///< projection camToObjDirection on plane XY float relatedHeight = (camToObjDirection.normalized()
  // - camToObjDirXY.normalized()).z(); ///< droneHeight - detectedObjHeight ROS_DEBUG_STREAM("relaitedHeight " <<
  // relatedHeight);

  // tf2::Vector3 lastToCurrent = objPosition - _firstObjPosition;
  // tf2::Vector3 lastToCurrentXY(lastToCurrent.x(), lastToCurrent.y(), 0.f);

  // float camToObjAngle = acosf(tf2::tf2Dot(lastToCurrent.normalized(), lastToCurrentXY.normalized()));
  // float camToObjAngle = acosf(tf2::tf2Dot(camToObjDirection.normalized(), camToObjDirXY.normalized()));
  // ROS_DEBUG_STREAM_NAMED("cam_to_obj_angle", "InRad " << camToObjAngle << " InDeg " << tf2Degrees(camToObjAngle));

  // just for rviz
  // tf2::Quaternion dirOrientation;
  // dirOrientation.setRotation(tf2::tf2Cross(camToObjDirXY.normalized(), camToObjDirection.normalized()),
  // camToObjAngle); _rvizPainter->draw(_rvizPainterObject.getYellowArrow(), cameraPosition, (dirOrientation *
  // cameraOrientation).normalized());

  if (_isObjDetected && !_isTrekLinePredicted) {
    trajectoryPrediction(cameraPosition, cameraOrientation, safePoints, totalPoints, pointDist2);
    _isTrekLinePredicted = true;
  }

  // getContactProbobility(newObjPosition, lastPointInRealTrajectory, cameraPosition);

  /* tf2::Vector3 currToLast = lastPointInRealTrajectory - newObjPosition;
  float m = currToLast.y() / currToLast.x(); */
}

void StateTracking::reset() {
  _resetTimer = ros::Time::now();
  _predTrajectory.clear();
  _realTrajPoints.clear();
  _predictedSigments.clear();
  _isObjDetected = false;
  _isTrekLinePredicted = false;
  _firstObjPosition.setZero();
  _lastObjPosition.setZero();
  _startTrackingTimer = ros::Time::now();
  _objMoveTimer = ros::Time(0);
  _detectionTimer = ros::Time::now();
  ROS_DEBUG_THROTTLE(1.0, "[StateTracking] Reset done");
}

void StateTracking::execute() {
  cv::Mat frame;
  cv::Mat depth;
  try {
    _vh->readColor(frame);
    _vh->readDepth(depth);
  } catch (...) {
    ROS_ERROR("Read fatal error");
    ros::Duration(5.0).sleep();
    wait();
  }

  if (frame.empty() || depth.empty()) {
    ROS_WARN_THROTTLE(1.0, "Frame is empty");
    return;
  }

  cv::Mat mask;
  cv::Mat tmpFrame = frame.clone();
  cv::Point2f center = {0, 0};
  uint16_t radius = 0;
  ros::Time imgProcTime = ros::Time::now();
  try {
    _bt->process(tmpFrame, mask, &center, &radius);
  } catch (const cv::Exception &e) {
    ROS_ERROR("process fatal error: %s", e.what());
    ros::Duration(5.0).sleep();
    wait();
  }

  ROS_DEBUG_STREAM_THROTTLE(.5, "Img proc time: " << ros::Duration(ros::Time::now() - imgProcTime).toSec() * 1000.0);

  if (radius != 0)
    _isConturFinded = true;
  else
    _isConturFinded = false;

  if (_isConturFinded && !_isTrekLinePredicted) {
    float distToObj = getDistToObj(depth, mask, center, radius);
    ROS_DEBUG_STREAM_THROTTLE(1.0, "distToObj " << distToObj);
    ROS_DEBUG_THROTTLE(1.0, "objRadius: %i", radius);

    if (distToObj == 0) {
      ROS_WARN_THROTTLE(5.0, "Distance to object out of range [%f, %f]", _minDist * 0.001f, _maxDist * 0.001f);
      // reset(); ///< !!!!!
      return;
    }

    ///< !!! Duplicated from conceptThree
    TransformStamped droneFrd;
    tf2::Quaternion droneOrientation; ///< drone orientation in map
    tf2::Vector3 dronePosition;       ///< drone position in map
    try {
      droneFrd = _tfBuffer.lookupTransform("map", "base_link_frd", ros::Time(0));
    } catch (const tf2::TransformException &e) {
      ROS_ERROR("[StateTracking] %s", e.what());
      ros::Duration(1.0).sleep();
      return;
    }
    Pose calcZonePose;
    calcZonePose.position.x = droneFrd.transform.translation.x;
    calcZonePose.position.y = droneFrd.transform.translation.y;
    calcZonePose.position.z = droneFrd.transform.translation.z;
    calcZonePose.orientation = tf2::toMsg(tf2::Quaternion::getIdentity());
    // calcZonePose.orientation = droneFrd.transform.rotation;
    _rvizPainter->update(_rvizPainterObject.getCalcZone(), calcZonePose);

    try {
      tf2::Vector3 newObjPosition;
      getObjPosFromImg(center, distToObj, newObjPosition);
      transformPose(newObjPosition);
      _rvizPainter->update(_rvizPainterObject.getObjPosition(), newObjPosition);

      if (_arucoTopic.getNumPublishers() != 0 && !_arucoPosition.header.frame_id.empty()) {
        TransformStamped arucoTransfMap;
        geometry_msgs::Vector3Stamped arucoPositionInMap;

        try {
          arucoTransfMap = _tfBuffer.lookupTransform("map", _arucoPosition.header.frame_id, ros::Time(0));
          tf2::doTransform(_arucoPosition, arucoPositionInMap, arucoTransfMap);
        } catch (const tf2::TransformException &e) {
          ROS_ERROR("[StateTracking] %s", e.what());
          ros::Duration(1.0).sleep();
        }

        tf2::Vector3 arucoPosition;
        tf2::fromMsg(arucoPositionInMap.vector, arucoPosition);
        ROS_DEBUG_STREAM_THROTTLE(1.0, "Aruco_to_obj_dist: " << tf2::tf2Distance(newObjPosition, arucoPosition));
        if (tf2::tf2Distance2(newObjPosition, arucoPosition) < 0.01) {
          reset();
          _rvizPainter->clear(_rvizPainterObject.getObjPosition());
          ros::Duration(1.0).sleep();
          return;
        }
      }

      ros::Time approxTime = ros::Time::now();
      if (distToObj <= 3.2)
        conceptThree(newObjPosition, radius);
      ROS_DEBUG_STREAM_THROTTLE(.5, "Approx time: " << ros::Duration(ros::Time::now() - approxTime).toSec() * 1000.0);
    } catch (const tf2::TransformException &e) {
      ROS_ERROR("[StateTracking] %s", e.what());
      ros::Duration(1.0).sleep();
      wait();
    } catch (const cv::Exception &e) {
      ROS_ERROR("[StateTracking] %s", e.what());
      ros::Duration(1.0).sleep();
      wait();
    } catch (...) {
      ros::Duration(1.0).sleep();
      wait();
    }

    ros::Duration pubPrd(1.0 / 15.0);
    if (ros::Time::now() - debugImgPubRate > pubPrd) {
      cv::circle(tmpFrame, center, radius + 3.0, cv::Scalar::all(128), 2, cv::LINE_4);
      cv::circle(tmpFrame, center, 3, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);

      // _fps = static_cast<int>(std::ceil(1.0 / ros::Duration(ros::Time::now() - _loopTimer).toSec()));

      // cv::Size textSize = cv::getTextSize(std::to_string(_fps), cv::FONT_HERSHEY_SIMPLEX, 0.4, 2, nullptr);
      // cv::putText(tmpFrame, std::to_string(_fps), cv::Point(5, textSize.height * 2), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar::all(0));

      sensor_msgs::ImagePtr debugImg = CvImage(std_msgs::Header(), enc::MONO8, mask).toImageMsg();
      sensor_msgs::ImagePtr resultImg = CvImage(std_msgs::Header(), enc::BGR8, tmpFrame).toImageMsg();
      _debugPub.publish(debugImg);
      _resultPub.publish(resultImg);
      debugImgPubRate += pubPrd;
    }

    _resetTimer = ros::Time::now();
  } else {
    ROS_DEBUG_STREAM_THROTTLE(1.0, "Obj lost");

    /* if (!_realTrajPoints.empty() && !_isTrekLinePredicted) {
      double timeOut = ros::Duration(ros::Time::now() - _resetTimer).toSec();
      ROS_DEBUG_THROTTLE(.5, "timeOut %lf", timeOut);
      if (timeOut >= 0.1) {
        if (_isObjDetected && !_isTrekLinePredicted) {
          ROS_DEBUG("predict");
          trajectoryPrediction(cameraPosition, cameraOrientation, safePoints, totalPoints, pointDist2);
          _isTrekLinePredicted = true;
        }
      }
    } */

    if (ros::Time::now() - _resetTimer >= ros::Duration(2.0)) {
      reset();
    }
  }

  _loopTimer = ros::Time::now();
}