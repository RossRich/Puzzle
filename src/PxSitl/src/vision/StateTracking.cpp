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
    ROS_WARN("No filter_gain param. Use default value %f", _filterGain);
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
  cv::namedWindow("test", cv::WINDOW_AUTOSIZE);

  _loopTimer = ros::Time::now();
  _detectionTimer = ros::Time::now();
  _resetTimer = ros::Time::now();

  return true;
}

void StateTracking::drawObjPose(Pose &p) {
  Marker m;

  m.header.frame_id = "map";
  m.header.stamp = ros::Time::now();
  m.ns = "obj_pose";
  m.id = 0;

  m.pose = p;

  geometry_msgs::Vector3 scale;
  scale.x = .1;
  scale.y = .1;
  scale.z = .1;
  m.scale = scale;

  std_msgs::ColorRGBA c;
  c.a = 1;
  c.b = 0;
  c.g = 1;
  c.r = 0;
  m.color = c;

  m.type = Marker::SPHERE;

  pubMarker(m);
}

void StateTracking::drawObjPredictedLine(std::list<geometry_msgs::Point> &list) {
  Marker m;

  m.header.frame_id = "map";
  m.header.stamp = ros::Time::now();
  m.ns = "obj_predicted_line";
  m.id = 1;
  m.action = Marker::MODIFY;

  for (auto &&p : list)
    m.points.push_back(p);

  m.pose.orientation.w = 1;

  geometry_msgs::Vector3 scale;
  scale.x = .07;
  scale.y = .07;
  scale.z = .07;
  m.scale = scale;

  std_msgs::ColorRGBA c;
  c.a = 1;
  c.b = 1;
  c.g = 1;
  c.r = 1;
  m.color = c;

  m.type = Marker::SPHERE_LIST;

  pubMarker(m);
}

void StateTracking::drawObjRealLine(std::list<geometry_msgs::Point> &list) {
  Marker line;

  line.header.frame_id = "map";
  line.header.stamp = ros::Time::now();

  line.id = 3;
  line.ns = "obj_real_line";
  line.type = Marker::LINE_STRIP;

  std_msgs::ColorRGBA c;
  c.a = 1;
  c.b = 0;
  c.g = 1;
  c.r = 0;

  line.color = c;

  geometry_msgs::Vector3 scale;
  scale.x = .02f;
  scale.y = .02f;

  line.scale = scale;

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

  /* for (auto &&ii : inv)
    std::cout << ii.first << ": " << ii.second << std::endl; */

  uint16_t distToBall;
  for (std::multimap<uint16_t, uint16_t>::const_iterator i = inv.cend(); i != inv.cbegin(); i--) {
    if (i->second <= 100)
      continue;

    distToBall = i->second;
    break;
  }

  return distToBall * 0.001f;
}

void StateTracking::getObjPoseFromCameraModel(cv::Point2i &point2d, float distToObj, Pose &objPose) {
  cv::Point3d point3d = _cameraModel.projectPixelTo3dRay(point2d);
  objPose.position.x = point3d.x;
  objPose.position.y = point3d.y;
  objPose.position.z = distToObj;
}

void StateTracking::transformPose(Pose &pose) {
  tf2::Vector3 vectorFromPose;
  tf2::fromMsg(pose.position, vectorFromPose);
  tf2::Transform transformFromPose(tf2::Quaternion::getIdentity(), vectorFromPose);
  tf2::Stamped<tf2::Transform> ts4Transform(transformFromPose, ros::Time::now(), "map");
  TransformStamped ts(tf2::toMsg(ts4Transform));
  ts.child_frame_id = _cameraModel.tfFrame();

  TransformStamped transform = _tfBuffer.lookupTransform("map", _cameraModel.tfFrame(), ros::Time(0));
  TransformStamped newTransform;
  tf2::doTransform(ts, newTransform, transform);

  pose.orientation = newTransform.transform.rotation;
  pose.position.x = newTransform.transform.translation.x;
  pose.position.y = newTransform.transform.translation.y;
  pose.position.z = newTransform.transform.translation.z;
}

void StateTracking::conceptOne(cv::Mat &mask, cv::Point2i &center, uint16_t &radius) {
  try {
    float distToObj = getDistToObj(mask, radius);
    Pose newObjPose;
    getObjPoseFromCameraModel(center, distToObj, newObjPose);
    transformPose(newObjPose);

    if (_lastObjPose == Pose()) {
      _lastObjPose = newObjPose;
      tf2::fromMsg(newObjPose.position, _firstObjPose);
      _startTrackingTimer = ros::Time::now();
      return;
    }

    tf2::Vector3 vecLastObjPosition;
    tf2::Vector3 vecNewObjPosition;
    tf2::fromMsg(_lastObjPose.position, vecLastObjPosition);
    tf2::fromMsg(newObjPose.position, vecNewObjPosition);

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
      tf2::Vector3 objDirection = vecLastObjPosition - vecNewObjPosition;
      // ROS_INFO("objDirection x %f; y %f; z %f;", objDirection.x(), objDirection.y(), objDirection.z());
      if (objDirection.length2() != 0) {
        tf2::Vector3 totalLength = vecNewObjPosition - _firstObjPose;
        ros::Duration totalTime = ros::Time::now() - _startTrackingTimer;

        double v = totalLength.length() / totalTime.toSec(); ///< velocity m/s
        double vX = totalLength.x() / totalTime.toSec();     ///< x axsis velocity
        double vY = totalLength.y() / totalTime.toSec();     ///< y axsis velocity
        double vZ = totalLength.z() / totalTime.toSec();     ///< z axsis velocity

        // ROS_INFO("Speed: %f", v);
        // ROS_INFO("Len for speed: %f", totalLength.length());
        // ROS_INFO("vecNewObjPosition x %f; y %f; z %f;", vecNewObjPosition.x(), vecNewObjPosition.y(), vecNewObjPosition.z());
        // ROS_INFO("vecLastObjPosition x %f; y %f; z %f;", vecLastObjPosition.x(), vecLastObjPosition.y(), vecLastObjPosition.z());
        // double angle = 0.05; ///< angle in radian
        double angle = asin(totalLength.z() / objDirection.length()); ///< angle in radian
        // double angle = _medianFilter.filtered(tf2Degrees(asin(objDirection.z() / objDirection.length())));
        // ROS_INFO("Angle: %f", angle);

        tf2::Vector3 tmpVecObjPos(vecNewObjPosition);
        tf2::Vector3 tmpVecLastObjPos(vecLastObjPosition);
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
      _objRealLine.push_back(newObjPose.position);
      _detectionTimer = ros::Time::now();
    }
    drawObjRealLine(_objRealLine);
    // ROS_INFO("Ball move %f", dist);

    Pose pose;
    tf2::Quaternion q = tf2::shortestArcQuatNormalize2(vecLastObjPosition, vecNewObjPosition);
    tf2::toMsg(tf2::Transform(q, vecNewObjPosition), pose);
    drawObjPose(pose);
    
    _lastObjPose = newObjPose;
  } catch (const cv::Exception &e) {
    ROS_ERROR("[BallTrackingRos] %s", e.what());
    ros::Duration(1.0).sleep();
    wait();
  } catch (const tf2::TransformException &e) {
    ROS_ERROR("[BallTrackingRos] %s", e.what());
    ros::Duration(1.0).sleep();
    wait();
  }
}

void StateTracking::conceptTwo(cv::Mat &mask, cv::Point2i &center, uint16_t &radius) {
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

    conceptOne(mask, center, radius);

    cv::circle(tmpFrame, center, radius + 7.0, cv::Scalar::all(128), 1,
               cv::LINE_4);
    cv::circle(tmpFrame, center, 3, cv::Scalar(0, 255, 0), cv::FILLED,
               cv::LINE_8);

    _fps = static_cast<int>(
        std::ceil(1.0 / ros::Duration(ros::Time::now() - _loopTimer).toSec()));

    cv::Size textSize = cv::getTextSize(
        std::to_string(_fps), cv::FONT_HERSHEY_SIMPLEX, 0.4, 2, nullptr);
    cv::putText(tmpFrame, std::to_string(_fps), cv::Point(5, textSize.height * 2),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar::all(0));
    /* cv::putText(tmpFrame, info.str(),
                cv::Point(5, (textSize.height * 4) + textSize.height),
                cv::FONT_HERSHEY_SIMPLEX, .4, cv::Scalar::all(0)); */
  } else {
    if (ros::Time::now() - _resetTimer >= ros::Duration(3.0, 0) && _lastObjPose != Pose()) {
      _lastObjPose = Pose();
      _resetTimer = ros::Time::now();
      _objPredictedLine.clear();
      _objRealLine.clear();
      ROS_DEBUG("Clean lines");
    }
  }

  try {
    cv::imshow("test", mask);
    cv::imshow(_winName, tmpFrame);
    cv::waitKey(1);
  } catch (const cv::Exception &e) {
    ROS_ERROR("[BallTrackingRos] The video in current environment not available.\n%s", e.what());
    wait();
  }

  _loopTimer = ros::Time::now();
}