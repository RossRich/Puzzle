#include "PxSitl/vision/StateTracking.hpp"

void StateTracking::conceptOne(tf2::Vector3 &objPosition, uint16_t &radius) {
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

void StateTracking::conceptTwo(tf2::Vector3 &objPosition, uint16_t &radius) {
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