#include "../../include/PxSitl/vision/StateTracking.hpp"

StateTracking::~StateTracking() {
  std::cout << "Delete state tracking\n";
  cv::destroyAllWindows();
  delete _tfListener;
  delete _bt;
  delete _vh;
  delete _it;
}

bool StateTracking::loadParam() {

  if (!_nh.getParam("data_config", _confFile)) {
    ROS_ERROR("No data_config param");
    return false;
  }

  ROS_INFO("Reads camera info from topic \'%s\'  ...", _cameraInfoTopic.c_str());
  try {
    _cameraInfo = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(_cameraInfoTopic, _nh);
  } catch (const ros::Exception &e) {
    ROS_ERROR("%s", e.what());
    ROS_ERROR("Failed to get camera info. Exit.");
    return false;
  }

  if (!_cameraInfo || _cameraInfo->width <= 0 || _cameraInfo->height <= 0) {
    ROS_ERROR("Failed to get camera info. Exit.");
    return false;
  }

  if (!_nh.getParam("filter_gain", _filterGain)) {
    ROS_INFO("No filter_gain param. Use default value %f", _filterGain);
  }

  if (!_nh.getParam("dt_for_prediction", _dt4prediction)) {
    ROS_INFO("No dt_for_prediction param. Use default value %f", _dt4prediction);
  }

  return true;
}

bool StateTracking::setup() {
  threshold_t threshold;
  if (!Utils::readThresholds(_confFile.c_str(), threshold)) {
    ROS_ERROR("Failed to read color treshold. Exit.");
    return false;
  }

  if (!_cameraModel.initialized()) {
    if (!_cameraModel.fromCameraInfo(_cameraInfo)) {
      ROS_ERROR("Failed to build model of camera. Exit.");
      return false;
    }
  }

  if (_it == nullptr)
    _it = new image_transport::ImageTransport(_nh);

  if (_vh == nullptr)
    _vh = new RosVH(_nh, *_it, _cameraInfo->width, _cameraInfo->height);

  if (_bt == nullptr)
    _bt = new BallTracking(_vh->getWidth(), _vh->getHeight(), threshold);

  if (_tfListener == nullptr)
    _tfListener = new tf2_ros::TransformListener(_tfBuffer, _nh);

  if (_ballPub.getTopic().empty()) {
    _ballPub = _nh.advertise<Marker>("ball", 5, false);
  }

  cv::namedWindow(_winName, cv::WINDOW_AUTOSIZE);
  // cv::namedWindow("test", cv::WINDOW_AUTOSIZE);

  _loopTimer = ros::Time::now();
  _detectionTimer = ros::Time::now();
  _resetTimer = ros::Time::now();

  return true;
}

void StateTracking::drawArrow(const tf2::Vector3 &position, const tf2::Quaternion &orientation, const std_msgs::ColorRGBA &c,
                              const char *name) {
  Pose p;
  tf2::toMsg(position, p.position);
  p.orientation = tf2::toMsg(orientation);

  drawArrow(p, c, name);
}

void StateTracking::drawArrow(const Pose &pose, const std_msgs::ColorRGBA &c, const char *name) {
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
  m.color = Utils::getColorMsg(0, 1, 0);

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
}

void StateTracking::drawObjRealLine(std::list<geometry_msgs::Point> &list) {
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
}

void StateTracking::pubMarker(Marker m) { _ballPub.publish(m); }

void StateTracking::wait() {
  ROS_INFO("Transition from %s state to Wait state", toString().c_str());
  cv::destroyAllWindows();
  _context.setState(static_cast<State *>(_context.getStateWait()));
}

float StateTracking::getDistToObj(cv::Mat &mask, uint16_t &radius) {
  uint16_t deametr = radius + radius;
  if(deametr > 100)
    deametr = 100;
  cv::Mat ballDist(cv::Size2i(deametr, deametr), CV_16UC1, cv::Scalar::all(0));
  _depth.copyTo(ballDist, mask);

  std::map<uint16_t, uint16_t> mapOfDist;
  for (auto &&d : cv::Mat_<uint16_t>(ballDist)) {
    if (d == 0)
      continue;
    if (mapOfDist.count(d) > 0)
      mapOfDist.at(d) += 1;
    else
      mapOfDist.insert(std::pair<uint16_t, uint16_t>(d, 0));
  }

  std::multimap<uint16_t, uint16_t> inv;
  for (auto &&i : mapOfDist)
    inv.insert(std::pair<uint16_t, uint16_t>(i.second, i.first));

  uint16_t distToBall;
  for (std::multimap<uint16_t, uint16_t>::const_iterator i = inv.cend(); i != inv.cbegin(); i--) {
    if (i->second <= 100)
      continue;

    distToBall = i->second;
    break;
  }

  return distToBall * 0.001f;
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

Pose StateTracking::transformPose2(tf2::Vector3 &position, const tf2::Quaternion &orientation) {
  tf2::Transform originalTransform(orientation, position);
  tf2::Stamped<tf2::Transform> ts4Transform(originalTransform, ros::Time::now(), "map");
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

void StateTracking::conceptOne(cv::Mat &mask, cv::Point2i &center, uint16_t &radius) {
  float distToObj = getDistToObj(mask, radius);
  tf2::Vector3 newObjPosition;
  getObjPosFromImg(center, distToObj, newObjPosition);
  transformPose(newObjPosition);

  if (_firstObjPosition.isZero()) {
    _lastObjPosition = newObjPosition;
    // tf2::fromMsg(newObjPosition.position, _firstObjPosition);
    _firstObjPosition = newObjPosition;
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

        geometry_msgs::Point p;
        tf2::toMsg(tmpVecObjPos, p);
        if (_objPredictedLine.size() > 50)
          _objPredictedLine.pop_front();
        _objPredictedLine.push_back(p);
      }
      // fixTragectory(_ballPredictedTrajs);

      drawObjPredictedLine(_objPredictedLine);
    }
    geometry_msgs::Point pointPath;
    tf2::toMsg(newObjPosition, pointPath);
    _objRealLine.push_back(pointPath);
    _detectionTimer = ros::Time::now();
  }
  drawObjRealLine(_objRealLine);
  // ROS_INFO("Ball move %f", dist);

  Pose pose;
  tf2::Quaternion q = tf2::shortestArcQuatNormalize2(vecLastObjPosition, vecNewObjPosition);
  tf2::toMsg(tf2::Transform(q, vecNewObjPosition), pose);
  drawObjPose(pose);

  _lastObjPosition = newObjPosition;
}

void StateTracking::conceptTwo(cv::Mat &mask, cv::Point2i &point2d, uint16_t &radius) {
  float distToObj = getDistToObj(mask, radius);
  tf2::Vector3 newObjPosition;
  getObjPosFromImg(point2d, distToObj, newObjPosition);
  transformPose(newObjPosition);

  if (_firstObjPosition.isZero()) {
    _firstObjPosition = newObjPosition;
    _isObjDetected = true;
    _startTrackingTimer = ros::Time::now();
    return;
  }

  drawObjPose(newObjPosition, Utils::Colors.at(Utils::Color::DarkGreen));

  // tf2::Vector3 newObjPosition;
  // tf2::fromMsg(newObjPosition.position, newObjPosition);

  if (ros::Time::now() - _buildRealTrekLineTimer >= ros::Duration(0.01)) {
    tf2::Vector3 lastPointInRealTrajectory;
    tf2::fromMsg(_objRealLine.back(), lastPointInRealTrajectory);

    if (tf2::tf2Distance2(lastPointInRealTrajectory, newObjPosition) >= 0.04) {
      if (_objRealLine.size() > 50) ///< TODO: add to launch param
        _objRealLine.pop_front();
      
      geometry_msgs::Point newObjPointMsg;
      tf2::toMsg(newObjPosition, newObjPointMsg);
      _objRealLine.push_back(newObjPointMsg);
      drawObjRealLine(_objRealLine);
      _buildRealTrekLineTimer = ros::Time::now();
    }
  }

  TransformStamped transform = _tfBuffer.lookupTransform("map", "base_link_frd", ros::Time(0));
  Pose cameraPose; ///< TODO: one use
  cameraPose.position.x = transform.transform.translation.x;
  cameraPose.position.y = transform.transform.translation.y;
  cameraPose.position.z = transform.transform.translation.z;
  cameraPose.orientation = transform.transform.rotation;

  tf2::Quaternion cameraOrientation; ///< camera orientation
  tf2::fromMsg(cameraPose.orientation, cameraOrientation);

  tf2::Vector3 cameraPosition; ///< camera position
  tf2::fromMsg(cameraPose.position, cameraPosition);

  tf2::Vector3 camToObjDirection = _firstObjPosition - cameraPosition; ///< vector from camera to detected obj
  tf2::Vector3 camFrwdDirecton = (cameraPosition + tf2::Vector3(camToObjDirection.x(), camToObjDirection.y(), 0)) - cameraPosition;

  tf2::Vector3 v = camToObjDirection.normalized() - camFrwdDirecton.normalized();
  // ROS_INFO_STREAM("z: " << v.z());

  drawObjPose(_firstObjPosition, Utils::Colors.at(Utils::Color::SonicSilver));
  drawArrow(cameraPose, Utils::Colors.at(Utils::Color::Red), "camera_pose");

  float angle = acosf(tf2::tf2Dot(camToObjDirection.normalized(), camFrwdDirecton.normalized()));
  ROS_DEBUG_STREAM_NAMED("angle_dot_prod", "InRad " << angle << " InDeg " << tf2Degrees(angle));

  // just for rviz
  tf2::Quaternion dirOrientation;
  dirOrientation.setRotation(tf2::tf2Cross(camFrwdDirecton.normalized(), camToObjDirection.normalized()), angle);
  tf2::Quaternion test = dirOrientation * cameraOrientation;
  test.normalize();
  drawArrow(cameraPosition, test, Utils::Colors.at(Utils::Color::Yellow), "camera_frwd_pose");

  if (_isObjDetected && !_isTrekLinePredicted) {
    float dt = _dt4prediction;                ///< TODO: move to launch param
    float gt = (PZ_GRAVITY * powf(dt, 2)) / 2.0f;     ///< may by 4.9 * pow(dt, 2)?
    tf2::Vector3 tmpPosition = _firstObjPosition; ///< trek line start from camera position
    if (v.z() < 0)
      tmpPosition = cameraPosition;

    tf2::Vector3 prevPosition = tmpPosition;
    geometry_msgs::Point tmpMsgPoint;
    tf2::Vector3 fromTo = newObjPosition - cameraPosition;
    tf2::Vector3 fromToXY(fromTo.x(), fromTo.y(), 0);
    float v0 = getVelocity(fromToXY.length(), fromTo.getZ(), angle);
    float x = v0 * cosf(angle) * dt;
    float z = (v0 * sinf(angle) * dt) - gt;
    dt = fromToXY.length() / 25.f / v0; ///< TODO: auto abjust dt for pretty rviz line

    for (uint8_t i = 0; i < 25; i++) { ///< TODO: move to launch param
      if (v.z() < 0)
        fromTo = _firstObjPosition - tmpPosition;
      else
        fromTo = tmpPosition - cameraPosition;

      if (fromTo.length2() <= 0.09)
        break;

      tf2::Vector3 velDirection = fromTo.normalized() * x;
      if (v.z() < 0) {
        velDirection.setZ(fromTo.normalized().z() + z);
        velDirection = tmpPosition + velDirection;
      } else {
        velDirection.setZ(fromTo.normalized().z() + z);
        velDirection = tmpPosition - velDirection;
      }

      if (!prevPosition.isZero() && prevPosition != tmpPosition) {
        Line line(prevPosition, velDirection);
        _predictedSigments.push_back(line);
      }

      prevPosition = tmpPosition;
      tmpPosition = velDirection;

      tf2::toMsg(velDirection, tmpMsgPoint);
      _objPredictedLine.push_back(tmpMsgPoint);
    }

    drawObjPredictedLine(_objPredictedLine);
    _isTrekLinePredicted = true;
  }
  getContactProbobility(newObjPosition, cameraPosition);
}

void StateTracking::execute() {
  _vh->readColor(_frame);
  _vh->readDepth(_depth);

  if (_frame.empty() || _depth.empty()) {
    ROS_WARN("Frame is empty");
    return;
  }

  cv::Mat tmpFrame = _frame.clone();
  cv::Mat mask;
  cv::Point2i center = {0, 0};
  uint16_t radius = 0;

  _bt->process(_frame, mask, &center, &radius);

  // ROS_DEBUG("RADIUS: %d", radius);

  if (radius != 0) {

    try {
      // conceptOne(mask, center, radius);
      conceptTwo(mask, center, radius);
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

    cv::circle(tmpFrame, center, radius + 7.0, cv::Scalar::all(128), 1, cv::LINE_4);
    cv::circle(tmpFrame, center, 3, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);

    _fps = static_cast<int>(std::ceil(1.0 / ros::Duration(ros::Time::now() - _loopTimer).toSec()));

    cv::Size textSize = cv::getTextSize(std::to_string(_fps), cv::FONT_HERSHEY_SIMPLEX, 0.4, 2, nullptr);
    cv::putText(tmpFrame, std::to_string(_fps), cv::Point(5, textSize.height * 2), cv::FONT_HERSHEY_SIMPLEX, 0.4,
                cv::Scalar::all(0));
    /* cv::putText(tmpFrame, info.str(),
                cv::Point(5, (textSize.height * 4) + textSize.height),
                cv::FONT_HERSHEY_SIMPLEX, .4, cv::Scalar::all(0)); */
  } else {
    if (ros::Time::now() - _resetTimer >= ros::Duration(3.0)) {
      _resetTimer = ros::Time::now();
      _objPredictedLine.clear();
      _objRealLine.clear();
      _predictedSigments.clear();
      _isObjDetected = false;
      _isTrekLinePredicted = false;
      _firstObjPosition.setZero();
      _lastObjPosition.setZero();
      tmpMarkerIndex++;
      ROS_DEBUG("Clean lines");
    }
  }

  try {
    // cv::imshow("test", mask);
    cv::imshow(_winName, tmpFrame);
    cv::waitKey(1);
  } catch (const cv::Exception &e) {
    ROS_ERROR("[StateTracking] The video in current environment not available.\n%s", e.what());
    wait();
  }

  _loopTimer = ros::Time::now();
}