#include "../../include/PxSitl/Vision/BallTrackingRos.hpp"

BallTrackingRos::BallTrackingRos(ros::NodeHandle &nh, VideoHandler &vh) : _nh(nh), _vh(vh) {

  if (!loadParam()) {
    ROS_ERROR("Load parameters error.\nNode init failed.");
    return;
  }

  _strategySrv = _nh.advertiseService("strategy_srv", &BallTrackingRos::runSetupSrv, this);
  ROS_INFO("Change strategy server ready");

  _ballPub = _nh.advertise<Marker>("ball", 5, false);
}

BallTrackingRos::~BallTrackingRos() {
  delete _strategy;
  delete _state;
}

bool BallTrackingRos::runSetupSrv(std_srvs::EmptyRequest &request, std_srvs::EmptyResponse &response) { return true; }

bool BallTrackingRos::loadParam() {

  if (!_nh.getParam("conf_file_path", _confFile)) {
    ROS_WARN("No conf_file_path param");
    ROS_WARN("Use a default configuration file path '%s'", _confFile.c_str());
  }

  ROS_INFO("Reads camera info...");
  try {
    _cameraInfo =
        ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/color/camera_info", _nh, ros::Duration(5));
  } catch (const ros::Exception &e) {
    ROS_ERROR("%s", e.what());
    ROS_ERROR("Failed to get camera info. Exit.");
    return false;
  }

  if (!_cameraInfo) {
    ROS_ERROR("Failed to get camera info. Exit.");
    return false;
  }
  ROS_INFO("Camera info ready");

  if (!_nh.getParam("filter_gain", _filterGain)) {
    _filterGain = 0.45f;
    ROS_INFO("No filter_gain param. Use default value %f", _filterGain);
  }

  return true;
}

void BallTrackingRos::drawBallPos(geometry_msgs::Pose p) {
  Marker m;

  m.header.frame_id = "camera_link";
  m.header.stamp = ros::Time::now();
  m.ns = "ball_pos";
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

void BallTrackingRos::drawPredicted(std::array<geometry_msgs::Point, 5> array) {
  Marker m;

  m.header.frame_id = "camera_link";
  m.header.stamp = ros::Time::now();
  m.ns = "ball_pred";
  m.id = 2;

  for (auto &&p : array) {
    m.points.push_back(p);
  }

  m.pose.orientation.w = 1;

  geometry_msgs::Vector3 scale;
  scale.x = .1;
  scale.y = .1;
  scale.z = .1;
  m.scale = scale;

  std_msgs::ColorRGBA c;
  c.a = 1;
  c.b = .321;
  c.g = .34;
  c.r = .0338;
  m.color = c;

  m.type = Marker::POINTS;

  pubMarker(m);
}

void BallTrackingRos::drawBallDiract(geometry_msgs::Pose pose) {
  Marker m;

  m.header.frame_id = "camera_link";
  m.header.stamp = ros::Time::now();
  m.ns = "ball_diract";
  m.id = 1;

  m.pose = pose;

  geometry_msgs::Vector3 scale;
  scale.x = .3;
  scale.y = .05;
  scale.z = .05;
  m.scale = scale;

  std_msgs::ColorRGBA c;
  c.a = 1;
  c.b = 0;
  c.g = 0;
  c.r = 1;
  m.color = c;

  m.type = Marker::ARROW;

  pubMarker(m);
}

void BallTrackingRos::draBallTrajectory(std::list<cv::Point3d> &bt) {
  Marker line;

  line.header.frame_id = "camera_link";
  line.header.stamp = ros::Time::now();

  line.id = 0;
  line.ns = "trajectory";
  line.type = Marker::LINE_STRIP;

  std_msgs::ColorRGBA c;
  c.a = 1;
  c.b = 0;
  c.g = 1;
  c.r = 0;

  line.color = c;

  geometry_msgs::Vector3 scale;
  scale.x = .05f;
  scale.y = .05f;

  line.scale = scale;

  geometry_msgs::Point point;
  for (auto &&i : bt) {
    point.x = i.x;
    point.y = i.y;
    point.z = i.z;
    line.points.push_back(point);
  }

  line.pose.orientation.w = 1;

  pubMarker(line);
}

void BallTrackingRos::pubMarker(Marker m) { _ballPub.publish(m); }

void BallTrackingRos::setState(State *state) {
  if (state != nullptr) {
    delete _state;
    _state = state;
  }
}

void BallTrackingRos::setStrategy(Strategy *strategy) {
  if (strategy != nullptr) {
    delete _strategy;
    _strategy = strategy;
  }
}

void BallTrackingRos::tracking() { _state->tracking(); }

void BallTrackingRos::wait() { _state->wait(); }

void BallTrackingRos::loop() {
  if (_strategy != nullptr)
    _strategy->execute();
}

void BallTrackingRos::shutdown() {
  _nh.shutdown();
  ros::shutdown();
}

void StateWait::tracking() {
  StrategyTracking *ts = new StrategyTracking(_context->getVideoHandler(), _context);

  if (!ts->init()) {
    delete ts;
    ROS_WARN("Translation to tracking strategy is rejected");
    return;
  }

  ROS_INFO("Go to tracking strategy");
  ts->setFilterGain(_context->getFilterGain());
  _context->setStrategy(ts);
  _context->setState(new StateTracking(_context));
}

void StateWait::wait() { ROS_INFO("In wait state"); }

void StateTracking::tracking() { ROS_INFO("In tracking state"); }

void StateTracking::wait() {
  ROS_INFO("Go to wait strategy");
  _context->setStrategy(new StrategyWait(_context));
  _context->setState(new StateWait(_context));
}

void StrategyWait::execute() {

  if (ros::Duration(ros::Time::now() - _timer) >= _timeOut) {

    threshold_t thresh;
    if (Utils::readThresholds(_context->getConfFile(), thresh)) {
      ROS_INFO("Data available");
      _context->tracking();
    } else
      ROS_WARN("No data");

    _timer = ros::Time::now();
  }
}

bool StrategyTracking::init() {
  threshold_t threshold;
  if (Utils::readThresholds(_context->getConfFile(), threshold))
    _bt = BallTracking(_vh.getWidth(), _vh.getHeight(), threshold);
  else
    return false;

  if (!_cameraModel.fromCameraInfo(_context->getCameraInfo())) {
    ROS_ERROR("Failed to build model of camera. Exit.");
    return false;
  }

  return true;
}

void StrategyTracking::execute() {
  ros::Time tt = ros::Time::now();
  _vh >> _frame;
  _vh.readDepth(_depth);

  if (_frame.empty() || _depth.empty()) {
    ROS_WARN("Frame is empty");
    return;
  }

  cv::Mat mask;
  cv::Point2i center = {0, 0};
  uint16_t radius = 0;

  _bt.process(_frame, mask, &center, &radius);

  ROS_DEBUG("RADIUS: %d", radius);

  if (radius != 0) {
    cv::circle(_frame, center, radius + 7.0, cv::Scalar::all(128), 1, cv::LINE_4);
    cv::circle(_frame, center, 3, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);

    std::stringstream info;
    info << "CENTER: " << center << " RADIUS: " << radius;

    cv::Point2f textPos = {center.x + radius + 15.0f, center.y - 15.0f};

    if (ros::Duration(ros::Time::now() - _timer) >= _timeOut) {
      try {
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

        // std::cout << "Hight accur: " << distToBall * 0.001f << std::endl;
        info << " DIST: " << distToBall;

        // std::cout << "Raw" << _depth.at<uint16_t>(center) * 0.001f <<
        // std::endl;

        cv::Point3d newBallPos = _cameraModel.projectPixelTo3dRay(center);
        newBallPos.z = distToBall * 0.001f;

        if (_ballTragectory.size() > 10)
          _ballTragectory.pop_front();

        _ballTragectory.push_back(newBallPos);

        tf2::Vector3 ballPoseV(_ballPos.x, _ballPos.y, _ballPos.z);
        tf2::Vector3 newBallPoseV(newBallPos.x, newBallPos.y, newBallPos.z);

        float dist = tf2::tf2Distance2(newBallPoseV, ballPoseV);

        if (ros::Time::now() - _lastDetection >= ros::Duration(0.1)) {
          double velocity = dist / ros::Duration(0.1).toSec();

          tf2::Vector3 ballDirV = newBallPoseV - ballPoseV;

          double vX = ballDirV.x() / ros::Duration(0.1).toSec();
          double vY = ballDirV.y() / ros::Duration(0.1).toSec();
          double vZ = ballDirV.z() / ros::Duration(0.1).toSec();

          // ROS_INFO("vX: %lf, xY: %lf vZ: %lf", vX, vY, vZ);

          tf2::Vector3 normVBall(ballDirV.normalize());

          tf2::Vector3 tt(newBallPoseV);
          std::array<geometry_msgs::Point, 5> linePoints;
          
          for (size_t i = 0; i < 5; i++) {
            normVBall.setZ(normVBall.z() +(vZ * ros::Duration(0.25).toSec()) + 5.0 * 0.0625);
            tt += normVBall;

            geometry_msgs::Point p;
            p.x = tt.x();
            p.y = tt.y();
            p.z = tt.z();
            
            linePoints.at(i) = p;
          }
          _context->drawPredicted(linePoints);

          _lastDetection = ros::Time::now();
        }
        // ROS_INFO("Ball move %f", dist);

        tf2::Quaternion q = tf2::shortestArcQuatNormalize2(ballPoseV, newBallPoseV);

        // tf2::Quaternion q;
        // q.setEulerZYX(ballDirV.z(), ballDirV.y(), ballDirV.x());

        Utils::fastFilterCvPoint3d(_ballPos, newBallPos, _filterGain);

        geometry_msgs::Pose pose;
        pose.position.x = _ballPos.x;
        pose.position.y = _ballPos.y;
        pose.position.z = _ballPos.z;
        pose.orientation.w = 1;

        _context->drawBallPos(pose);

        pose.orientation.w = q.w();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();

        _context->drawBallDiract(pose);

        _context->draBallTrajectory(_ballTragectory);

        // std::stringstream info2;
        // info2 << _ballPos;
        // ROS_INFO("Pixel in 3d: %s", info2.str().c_str());
      } catch (const cv::Exception &e) {
        std::cerr << e.what() << '\n';
        _context->shutdown();
      }
      _timer = ros::Time::now();
    }

    cv::putText(_frame, info.str(), textPos, cv::FONT_HERSHEY_SIMPLEX, .6, cv::Scalar::all(0), 2);
    /* TransformStamped transform;

    try {
      ros::Time acquisition_time = ros::Time(_vh.getLastTime());
      ROS_INFO("Img last time %f", acquisition_time.toSec());
      ros::Duration timeout(1.0 / 30);
      // _tfListener.waitForTransform(_cameraModel.tfFrame(), frame_id,
    acquisition_time, timeout); transform =
    _tfBuffer.lookupTransform(_cameraModel.tfFrame(), "camera_link",
    acquisition_time, timeout); } catch (tf2::TransformException &ex) {
      ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
      // return;
    } */

    /* geometry_msgs::Vector pt = transform.transform.translation;
    cv::Point3d pt_cv(pt.x, pt.y, pt.z);
    cv::Point2d uv = _cameraModel.project3dToPixel(pt_cv); */
    // ROS_INFO("dt: %f ms", ros::Duration(ros::Time::now() - tt).toSec() * 1000.0f);
  }

  try {
    cv::imshow(_winName, _frame);
    cv::waitKey(1);
  } catch (const cv::Exception &e) {
    ROS_ERROR("The video in current environment not available.\n%s", e.what());
  }
}
