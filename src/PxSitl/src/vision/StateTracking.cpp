#include "PxSitl/vision/StateTracking.hpp"

StateTracking::~StateTracking() {
  std::cout << "Delete state tracking\n";
  // cv::destroyAllWindows();
}

bool StateTracking::loadParam() {

  if (!_nh.getParam("data_config", _confFile)) {
    ROS_ERROR("[StateTracking] No data_config param");
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

  if (!_nh.getParam("dt_for_prediction", _dt4prediction)) {
    ROS_INFO("[StateTracking] No dt_for_prediction param. Use default value %f", _dt4prediction);
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

  if (!_bt)
    _bt = std::make_unique<BallTracking>(_vh->getWidth(), _vh->getHeight(), threshold);

  if (!_tfListener)
    _tfListener = std::make_unique<tf2_ros::TransformListener>(_tfBuffer, _nh);

  if (!_rvizPainter)
    _rvizPainter = std::make_unique<RvizPainter>(_nh, "tracking_painter");

  if (_metricsPublisher.getTopic().empty()) {
    _metricsPublisher = _nh.advertise<puzzle_msgs::Metrics>("metrics", 1500);
  }

  // cv::namedWindow(_winName, cv::WINDOW_AUTOSIZE);
  // cv::namedWindow("test", cv::WINDOW_AUTOSIZE);

  _loopTimer = ros::Time::now();
  _detectionTimer = ros::Time::now();
  _resetTimer = ros::Time::now();

  return true;
}

void StateTracking::wait() {
  ROS_INFO("[StateTracking] Transition from %s state to Wait state", toString().c_str());
  // cv::destroyAllWindows();
  _context.setState(static_cast<State *>(_context.getStateWait()));
}

float StateTracking::getDistToObj(cv::Mat &mask, uint16_t &radius) {
  uint16_t deametr = radius + radius;
  if (deametr > 30)
    deametr = 30;
  cv::Mat ballDist(cv::Size2i(deametr, deametr), CV_16UC1, cv::Scalar::all(0));
  _depth.copyTo(ballDist, mask);

  uint16_t maxCount = 0;
  uint16_t bestDist = 0;
  std::map<uint16_t, uint16_t> mapOfDist; ///< <Dist, NumOfTheDist>
  for (auto &&d : cv::Mat_<uint16_t>(ballDist)) {
    if (d <= 100 || d > 3000) ///< min dist = 10 cm, max dist = 3 m
      continue;
    if (mapOfDist.count(d) > 0) {
      mapOfDist.at(d) += 1;
      uint16_t num = mapOfDist.at(d);
      if (num > maxCount) {
        maxCount = num;
        bestDist = d;
      }

    } else
      mapOfDist.insert(std::pair<uint16_t, uint16_t>(d, 1));
  }

  /* ROS_DEBUG_STREAM("maxCount " << maxCount << " bestDist " << bestDist);

  std::multimap<uint16_t, uint16_t> inv;
  for (auto &&i : mapOfDist) {
    inv.insert(std::pair<uint16_t, uint16_t>(i.second, i.first));
    ROS_DEBUG_STREAM("count " << i.second << " dist " << i.first);
  }

  uint16_t distToBall;
  for (std::multimap<uint16_t, uint16_t>::const_iterator i = inv.cend(); i != inv.cbegin(); i--) {
    if (i->second <= 100)
      continue;

    distToBall = i->second;
    break;
  } */

  return bestDist * 0.001f;
}

void StateTracking::getObjPosFromImg(cv::Point2i &point2d, float distToObj, tf2::Vector3 &objPos) {
  cv::Point3d point3d = _cameraModel.projectPixelTo3dRay(point2d);
  objPos.setValue(point3d.x, point3d.y, distToObj);
}

void StateTracking::transformPose(tf2::Vector3 &position) {
  TransformStamped transform = _tfBuffer.lookupTransform("map", _cameraModel.tfFrame(), ros::Time(0));

  tf2::Transform originalTransform(tf2::Quaternion::getIdentity(), position);
  TransformStamped ts;
  ts.header.frame_id = "map";
  ts.header.stamp = ros::Time::now();
  ts.child_frame_id = _cameraModel.tfFrame();
  tf2::convert(originalTransform, ts.transform);

  TransformStamped newTransform;
  tf2::doTransform(ts, newTransform, transform);
  tf2::convert(newTransform.transform.translation, position);
}

Pose StateTracking::transformPose2(const tf2::Vector3 &position, const tf2::Quaternion &orientation) {
  tf2::Transform originalTransform(orientation, position);
  tf2::Stamped<tf2::Transform> ts4Transform(originalTransform, ros::Time::now(), "base_link_frd");
  TransformStamped ts(tf2::toMsg(ts4Transform));
  ts.child_frame_id = _cameraModel.tfFrame();
  TransformStamped transform = _tfBuffer.lookupTransform("map", "base_link_frd", ros::Time(0));
  TransformStamped newTransform;
  tf2::doTransform(ts, newTransform, transform);

  Pose pose;
  pose.orientation = newTransform.transform.rotation;
  pose.position.x = newTransform.transform.translation.x;
  pose.position.y = newTransform.transform.translation.y;
  pose.position.z = newTransform.transform.translation.z;

  return pose;
}

Pose StateTracking::transformPose3(const tf2::Vector3 &position, const tf2::Quaternion &orientation) {
  tf2::Transform originalTransform(orientation, position);
  tf2::Stamped<tf2::Transform> ts4Transform(originalTransform, ros::Time::now(), "map");
  TransformStamped ts(tf2::toMsg(ts4Transform));
  ts.child_frame_id = "base_link";
  TransformStamped transform = _tfBuffer.lookupTransform("base_link_frd", "map", ros::Time(0));
  TransformStamped newTransform;
  tf2::doTransform(ts, newTransform, transform);

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

  _metricsPublisher.publish(metricsMsg);
  return 1;
}

void StateTracking::trajectoryPrediction(const tf2::Vector3 &cameraPosition, const tf2::Quaternion &cameraOrientation,
                                         uint8_t safePoints, uint8_t totalPoints, float pointDist2) {
  if (_realTrajPoints.size() >= 5)
    safePoints = 5;
  else if (_realTrajPoints.size() < 5 && _realTrajPoints.size() > 1) {
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
    float z = (abc[0] * powf(cameraInBaseLinkPosition.x(), 2) + abc[1] * cameraInBaseLinkPosition.x() + abc[2]) +
              cameraInBaseLinkPosition.z();
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

void StateTracking::conceptOne(cv::Mat &mask, cv::Point2i &center, uint16_t &radius) {
  float distToObj = getDistToObj(mask, radius);
  tf2::Vector3 newObjPosition;
  getObjPosFromImg(center, distToObj, newObjPosition);
  transformPose(newObjPosition);

  if (_firstObjPosition.isZero()) {
    _lastObjPosition = newObjPosition;
    // tf2::fromMsg(newObjPosition.position, _firstObjPosition);
    _firstObjPosition = newObjPosition;
    _rvizPainter->draw(_rvizPainterObject.getObjFirstPosition(), _firstObjPosition);
    _startTrackingTimer = ros::Time::now();
    return;
  }

  tf2::Vector3 vecLastObjPosition;
  tf2::Vector3 vecNewObjPosition;
  // tf2::fromMsg(_lastObjPosition.position, vecLastObjPosition);
  // tf2::fromMsg(newObjPosition.position, vecNewObjPosition);

  // double dt = 0.01; ///< 100Hz
  ros::Rate rate(250);
  if (ros::Time::now() - _detectionTimer >= ros::Duration(rate)) {
    // ROS_INFO("New timer cycle");
    // double velocity = dist / ros::Duration(0.1).toSec();
    // float dist = tf2::tf2Distance2(vecNewObjPosition, vecLastObjPosition);

    // double vX = objDirection.x() / ros::Duration(dt).toSec();
    // double vY = objDirection.y() / ros::Duration(dt).toSec();
    // double vZ = objDirection.z() / ros::Duration(dt).toSec();

    // ROS_INFO("vX: %lf, xY: %lf vZ: %lf", vX, vY, vZ);

    // ROS_INFO("objDirection.len = %f", objDirection.length());
    // ROS_INFO("objDirection.len2 = %f", objDirection.length2());
    tf2::Vector3 objDirection = _lastObjPosition - newObjPosition;
    // ROS_INFO("objDirection x %f; y %f; z %f;", objDirection.x(), objDirection.y(), objDirection.z());
    if (objDirection.length2() != 0) {
      tf2::Vector3 totalLength = newObjPosition - _firstObjPosition;
      ros::Duration totalTime = ros::Time::now() - _startTrackingTimer;

      double v = totalLength.length() / totalTime.toSec(); ///< velocity m/s
      double vX = totalLength.x() / totalTime.toSec();     ///< x axsis velocity
      double vY = totalLength.y() / totalTime.toSec();     ///< y axsis velocity
      double vZ = totalLength.z() / totalTime.toSec();     ///< z axsis velocity

      // ROS_INFO("Speed: %f", v);
      // ROS_INFO("Len for speed: %f", totalLength.length());
      // ROS_INFO("vecNewObjPosition x %f; y %f; z %f;", vecNewObjPosition.x(), vecNewObjPosition.y(),
      // vecNewObjPosition.z()); ROS_INFO("vecLastObjPosition x %f; y %f; z %f;", vecLastObjPosition.x(),
      // vecLastObjPosition.y(), vecLastObjPosition.z()); double angle = 0.05; ///< angle in radian
      double angle = asin(totalLength.z() / objDirection.length()); ///< angle in radian
      // double angle = _medianFilter.filtered(tf2Degrees(asin(objDirection.z() / objDirection.length())));
      // ROS_INFO("Angle: %f", angle);

      tf2::Vector3 tmpVecObjPos(newObjPosition);
      tf2::Vector3 tmpVecLastObjPos(_lastObjPosition);
      double timePred = 0.1;

      for (size_t i = 0; i < 5; i++) {
        tf2::Vector3 tmpDirection = tmpVecLastObjPos - tmpVecObjPos;
        // ROS_INFO("tmpDirection x %f; y %f; z %f;", tmpDirection.x(), tmpDirection.y(), tmpDirection.z());
        // ROS_INFO("tmpDirection.len: %f", tmpDirection.length());
        // angle = _medianFilter.filtered(tf2Degrees(asin(tmpDirection.z() / tmpDirection.length())));
        // angle = asin(tmpDirection.z() / tmpDirection.length()); ///< in radian
        // ROS_INFO("angle2: %f", angle);

        double dx = vX * timePred;
        double gt = (9.8f * powl(timePred, 2)) / 2.0f;
        double dy = (vZ * timePred) - gt;

        // ROS_INFO("dx %f, dy %f", dx, dy);

        tmpVecLastObjPos = tmpVecObjPos;
        tmpVecObjPos.setX(tmpVecObjPos.getX() + dx);
        tmpVecObjPos.setZ(tmpVecObjPos.getZ() + dy);

        // geometry_msgs::Point p;
        // tf2::toMsg(tmpVecObjPos, p);
        if (_predTrajectory.size() > 50)
          _predTrajectory.pop_front();
        _predTrajectory.push_back(tmpVecObjPos);
      }
      // fixTragectory(_ballPredictedTrajs);

      // drawObjPredictedLine(_predTrajectory);
    }
    // geometry_msgs::Point pointPath;
    // tf2::toMsg(newObjPosition, pointPath);
    _realTrajPoints.push_back(newObjPosition);
    _detectionTimer = ros::Time::now();
  }
  // drawObjRealLine(_realTrajPoints);
  // ROS_INFO("Ball move %f", dist);

  Pose pose;
  tf2::Quaternion q = tf2::shortestArcQuatNormalize2(vecLastObjPosition, vecNewObjPosition);
  tf2::toMsg(tf2::Transform(q, vecNewObjPosition), pose);
  // drawObjPose(pose);

  _lastObjPosition = newObjPosition;
}

void StateTracking::conceptTwo(cv::Mat &mask, cv::Point2i &point2d, uint16_t &radius) {
  float distToObj = getDistToObj(mask, radius);
  tf2::Vector3 newObjPosition;
  getObjPosFromImg(point2d, distToObj, newObjPosition);
  transformPose(newObjPosition);

  if (!_isObjDetected) {
    _firstObjPosition = newObjPosition;
    _isObjDetected = true;
    _realTrajPoints.push_back(newObjPosition);
    _rvizPainter->draw(_rvizPainterObject.getObjFirstPosition(), _firstObjPosition);
    _startTrackingTimer = ros::Time::now();
    return;
  }

  tf2::Vector3 lastPointInRealTrajectory = _realTrajPoints.back();
  if (tf2::tf2Distance2(lastPointInRealTrajectory, newObjPosition) < 0.01)
    return;

  if (_realTrajPoints.size() > 30) ///< TODO: add to launch param
    _realTrajPoints.pop_front();

  _realTrajPoints.push_back(newObjPosition);
  _rvizPainter->draw(_rvizPainterObject.getRealTrajLine(), _realTrajPoints);

  TransformStamped transform = _tfBuffer.lookupTransform("map", "base_link_frd", ros::Time(0));
  tf2::Quaternion cameraOrientation; ///< camera orientation in map
  tf2::Vector3 cameraPosition;       ///< camera position in map
  tf2::fromMsg(transform.transform.rotation, cameraOrientation);
  tf2::fromMsg(transform.transform.translation, cameraPosition);
  _rvizPainter->draw(_rvizPainterObject.getRegArrow(), cameraPosition, cameraOrientation);

  tf2::Vector3 camToObjDirection = _firstObjPosition - cameraPosition; ///< vector from camera to detected obj
  tf2::Vector3 camToObjDirXY = (cameraPosition + tf2::Vector3(camToObjDirection.x(), camToObjDirection.y(), 0)) -
                               cameraPosition; ///< projection camToObjDirection on plane XY
  float relatedHeight =
      (camToObjDirection.normalized() - camToObjDirXY.normalized()).z(); ///< droneHeight - detectedObjHeight
  // ROS_DEBUG_STREAM("relaitedHeight " << relatedHeight);

  float camToObjAngle = acosf(tf2::tf2Dot(camToObjDirection.normalized(), camToObjDirXY.normalized()));
  ROS_DEBUG_STREAM_NAMED("cam_to_obj_angle", "InRad " << camToObjAngle << " InDeg " << tf2Degrees(camToObjAngle));

  // just for rviz
  tf2::Quaternion dirOrientation;
  dirOrientation.setRotation(tf2::tf2Cross(camToObjDirXY.normalized(), camToObjDirection.normalized()), camToObjAngle);
  _rvizPainter->draw(_rvizPainterObject.getYellowArrow(), cameraPosition,
                     (dirOrientation * cameraOrientation).normalized());

  if (_isObjDetected && !_isTrekLinePredicted) {
    float dt = _dt4prediction;                      ///< TODO: move to launch param
    float gt = (PZ_GRAVITY * powf(dt, 2)) / 2.0f;   ///< 4.9 * pow(dt, 2)?
    tf2::Vector3 startPosition = _firstObjPosition; ///< trek line start from first detected obj
    if (relatedHeight > 0)
      startPosition = cameraPosition; ///< trek line start from camera position

    tf2::Vector3 prevPosition = startPosition;
    tf2::Vector3 fromTo = newObjPosition - cameraPosition;
    tf2::Vector3 fromToXY(fromTo.x(), fromTo.y(), 0);
    float v0 = getVelocity(fromToXY.length(), fromTo.getZ(), camToObjAngle);
    dt = fromToXY.length() / 25.f / v0; ///< TODO: auto abjust dt for pretty rviz line
    gt = (PZ_GRAVITY * powf(dt, 2)) / 2.0f;
    ROS_DEBUG_STREAM_NAMED("vel", "Vel " << v0);
    ROS_DEBUG_STREAM_NAMED("total_time", "TotalTime " << fromToXY.length() / v0);
    float x = v0 * cosf(camToObjAngle) * dt;
    float z = (v0 * sinf(camToObjAngle) * dt) - gt;

    for (uint8_t i = 0; i < 25; i++) { ///< TODO: move to launch param
      if (relatedHeight > 0)
        fromTo = _firstObjPosition - startPosition; ///< startPosition has a new value for every iteration
      else
        fromTo = startPosition - cameraPosition;

      if (fromTo.length2() <= 0.09f)
        break;

      tf2::Vector3 velDirection = fromTo.normalized() * x;
      velDirection.setZ(fromTo.normalized().z() + z);

      if (relatedHeight > 0) {
        velDirection = startPosition + velDirection;
      } else {
        velDirection = startPosition - velDirection;
      }

      // if (!prevPosition.isZero() && prevPosition != startPosition) {
      if (i != 0) { ///< ignore first line
        Line line(startPosition, velDirection);
        _predictedSigments.push_back(line);
      }

      /* ROS_DEBUG_STREAM(line);

      if (_predictedSigments.size() & 1)
        _rvizPainter->draw(_rvizPainterObject.getRedLine(), line.getP1(), line.getP2());
      else
        _rvizPainter->draw(_rvizPainterObject.getYellowLine(), line.getP1(), line.getP2());

      _rvizPainter->draw(_rvizPainterObject.getMidPoint(), line.getMidpoint()); */
      // }

      // prevPosition = startPosition;
      startPosition = velDirection;

      _predTrajectory.push_back(velDirection);
    }

    _rvizPainter->draw(_rvizPainterObject.getPredTrajLine(), _predTrajectory);
    _isTrekLinePredicted = true;
  }

  getContactProbobility(newObjPosition, lastPointInRealTrajectory, cameraPosition);

  /* tf2::Vector3 currToLast = lastPointInRealTrajectory - newObjPosition;
  float m = currToLast.y() / currToLast.x(); */
}

void StateTracking::approxQuadratic(std::vector<tf2::Vector3> &in, std::vector<float> &out, uint8_t newPoints,
                                    float dist) {
  if (in.size() < 3) {
    ROS_ERROR_STREAM(
        "[StateTracking::approxQuadratic] Input points number not enother for quadratic approximation. Total points = "
        << in.size());
    return;
  }

  tf2::Vector3 p1 = in[0];
  tf2::Vector3 p2 = in[1];
  tf2::Vector3 p3 = in[2];

  Pose pp1, pp2, pp3;
  pp1 = transformPose3(in[0], tf2::Quaternion::getIdentity());
  pp2 = transformPose3(in[1], tf2::Quaternion::getIdentity());
  pp3 = transformPose3(in[2], tf2::Quaternion::getIdentity());

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
    tf2::Vector3 tttt;
    Pose p = transformPose2(approxQuat, tf2::Quaternion::getIdentity());
    tf2::fromMsg(p.position, tttt);
    _predTrajectory.push_back(tttt);
  }
}

void StateTracking::approxLinear(std::vector<tf2::Vector3> &in, std::vector<float> &out, uint8_t newPoints) {
  if (in.size() < 2) {
    ROS_ERROR_STREAM(
        "[StateTracking::approxLinear] Input points number not enother for linear approximation. Total points = "
        << in.size());
    return;
  }

  tf2::Vector3 &p1 = in[0];
  tf2::Vector3 &p2 = in[1];

  out.push_back((p1.z() - p2.z()) / (p1.x() - p2.x())); ///< TODO: division by zero
  out.push_back(p2.z() - out[0] * p2.x());

  float xx = _firstObjPosition.x();
  tf2::Vector3 approxQuat = _firstObjPosition;
  for (size_t i = 0; i < newPoints; i++) {
    float z = out[0] * xx + out[1];
    approxQuat.setX(xx);
    approxQuat.setZ(z);
    xx += sqrtf(sqrtf(tf2::tf2Distance2(p1, p2)));

    _predTrajectory.push_back(approxQuat);
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

void StateTracking::conceptThree(cv::Mat &mask, cv::Point2i &point2d, uint16_t &radius) {

  if (radius == 0 && _realTrajPoints.empty())
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
  }

  float distToObj = getDistToObj(mask, radius);
  ROS_DEBUG_STREAM_THROTTLE(1.0, "distToObj " << distToObj);

  if (distToObj == 0) {
    ROS_WARN_STREAM_THROTTLE(5.0, "Distance to object out of range [0.1, 5.0]");
    return;
  }

  tf2::Vector3 newObjPosition;
  getObjPosFromImg(point2d, distToObj, newObjPosition);
  transformPose(newObjPosition);

  if (!_isObjDetected) {
    _firstObjPosition = newObjPosition;
    _isObjDetected = true;
    _realTrajPoints.push_back(newObjPosition);
    _rvizPainter->draw(_rvizPainterObject.getObjFirstPosition(), _firstObjPosition);
    _startTrackingTimer = ros::Time::now();
    return;
  }

  if (tf2::tf2Distance2(_firstObjPosition, newObjPosition) > 0.0225 && _objMoveTimer.is_zero()) {
    _objMoveTimer = ros::Time::now();
  }

  tf2::Vector3 lastPointInRealTrajectory = _realTrajPoints.back();
  if (tf2::tf2Distance2(lastPointInRealTrajectory, newObjPosition) < 0.0225)
    return;

  if (_realTrajPoints.size() > 30) ///< TODO: add to launch param
    _realTrajPoints.pop_front();

  _realTrajPoints.push_back(newObjPosition);
  _rvizPainter->draw(_rvizPainterObject.getRealTrajLine(), _realTrajPoints);

  if (_realTrajPoints.size() == 2)
    _test2222 = newObjPosition;

  // if (_realTrajPoints.size() < 3)
  // return;

  TransformStamped transform = _tfBuffer.lookupTransform("map", "base_link_frd", ros::Time(0));
  // tf2::Quaternion cameraOrientation; ///< camera orientation in map
  // tf2::Vector3 cameraPosition;       ///< camera position in map
  tf2::fromMsg(transform.transform.rotation, cameraOrientation);
  tf2::fromMsg(transform.transform.translation, cameraPosition);

  float totalDist2 = tf2::tf2Distance2(_firstObjPosition, cameraPosition);
  pointDist2 = tf2::tf2Distance2(_firstObjPosition, _test2222);
  // float pointDist2 = tf2::tf2Distance2(_firstObjPosition, _test2222);
  totalPoints = std::floor(sqrtf(totalDist2 / pointDist2));
  // uint8_t totalPoints = std::floor(sqrtf(totalDist2 / pointDist2));
  safePoints = std::floor(totalPoints / 2.f);
  // uint8_t safePoints = std::floor(totalPoints / 2.f);
  if (safePoints > 5)
    safePoints = 5; ///< TODO: not good idea

  float safeDist = tf2::tf2Distance2(cameraPosition, newObjPosition) / totalDist2;
  ROS_DEBUG_STREAM("safeDist: " << safeDist);
  ROS_DEBUG_STREAM("pointsInTraj: " << _realTrajPoints.size());
  ROS_DEBUG_STREAM("objMoveTimer: " << ros::Duration(ros::Time::now() - _objMoveTimer).toSec());

  if (safeDist > .5f && ros::Duration(ros::Time::now() - _objMoveTimer).toSec() < 0.1) {
    return;
  }

  _rvizPainter->draw(_rvizPainterObject.getRegArrow(), cameraPosition, cameraOrientation);

  tf2::Vector3 camToObjDirection = _firstObjPosition - cameraPosition; ///< vector from camera to detected obj
  tf2::Vector3 camToObjDirXY = (cameraPosition + tf2::Vector3(camToObjDirection.x(), camToObjDirection.y(), 0)) -
                               cameraPosition; ///< projection camToObjDirection on plane XY
  float relatedHeight =
      (camToObjDirection.normalized() - camToObjDirXY.normalized()).z(); ///< droneHeight - detectedObjHeight
  // ROS_DEBUG_STREAM("relaitedHeight " << relatedHeight);

  tf2::Vector3 lastToCurrent = newObjPosition - _firstObjPosition;
  tf2::Vector3 lastToCurrentXY(lastToCurrent.x(), lastToCurrent.y(), 0.f);

  float camToObjAngle = acosf(tf2::tf2Dot(lastToCurrent.normalized(), lastToCurrentXY.normalized()));
  // float camToObjAngle = acosf(tf2::tf2Dot(camToObjDirection.normalized(), camToObjDirXY.normalized()));
  // ROS_DEBUG_STREAM_NAMED("cam_to_obj_angle", "InRad " << camToObjAngle << " InDeg " << tf2Degrees(camToObjAngle));

  // just for rviz
  tf2::Quaternion dirOrientation;
  dirOrientation.setRotation(tf2::tf2Cross(camToObjDirXY.normalized(), camToObjDirection.normalized()), camToObjAngle);
  _rvizPainter->draw(_rvizPainterObject.getYellowArrow(), cameraPosition,
                     (dirOrientation * cameraOrientation).normalized());

  if (_isObjDetected && !_isTrekLinePredicted) {
    trajectoryPrediction(cameraPosition, cameraOrientation, safePoints, totalPoints, pointDist2);
    _isTrekLinePredicted = true;
  }

  // getContactProbobility(newObjPosition, lastPointInRealTrajectory, cameraPosition);

  /* tf2::Vector3 currToLast = lastPointInRealTrajectory - newObjPosition;
  float m = currToLast.y() / currToLast.x(); */
}

void StateTracking::execute() {
  _vh->readColor(_frame);
  _vh->readDepth(_depth);

  if (_frame.empty() || _depth.empty()) {
    ROS_WARN_THROTTLE(1.0, "Frame is empty");
    return;
  }

  cv::Mat tmpFrame = _frame.clone();
  cv::Mat mask;
  cv::Point2i center = {0, 0};
  uint16_t radius = 0;

  _bt->process(_frame, mask, &center, &radius);

  // ROS_DEBUG("RADIUS: %i", radius);

  if (radius != 0) {

    /* try {
      conceptOne(mask, center, radius);
      conceptTwo(mask, center, radius);
      conceptThree(mask, center, radius);
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
    } */

    cv::circle(tmpFrame, center, radius + 7.0, cv::Scalar::all(128), 1, cv::LINE_4);
    cv::circle(tmpFrame, center, 3, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);

    _fps = static_cast<int>(std::ceil(1.0 / ros::Duration(ros::Time::now() - _loopTimer).toSec()));

    cv::Size textSize = cv::getTextSize(std::to_string(_fps), cv::FONT_HERSHEY_SIMPLEX, 0.4, 2, nullptr);
    cv::putText(tmpFrame, std::to_string(_fps), cv::Point(5, textSize.height * 2), cv::FONT_HERSHEY_SIMPLEX, 0.4,
                cv::Scalar::all(0));
    /* cv::putText(tmpFrame, info.str(),
                cv::Point(5, (textSize.height * 4) + textSize.height),
                cv::FONT_HERSHEY_SIMPLEX, .4, cv::Scalar::all(0)); */
    _resetTimer = ros::Time::now();
  } else {
    ROS_DEBUG_STREAM_THROTTLE(0.25, "Obj lost");
    if (ros::Time::now() - _resetTimer >= ros::Duration(3.0)) {
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
      ROS_DEBUG("Clean lines");
    }
  }

  try {
    // conceptOne(mask, center, radius);
    // conceptTwo(mask, center, radius);
    conceptThree(mask, center, radius);
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

  try {
    // cv::imshow("test", mask);
    // cv::imshow(_winName, tmpFrame);
    // cv::waitKey(1);
  } catch (const cv::Exception &e) {
    ROS_ERROR("[StateTracking] The video in current environment not available.\n%s", e.what());
    wait();
  }

  _loopTimer = ros::Time::now();
}