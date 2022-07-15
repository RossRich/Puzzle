#include "StrategyTracking.hpp"

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
  _vh >> _frame;
  _vh.readDepth(_depth);

  if (_frame.empty() || _depth.empty()) {
    ROS_WARN("Frame is empty");
    return;
  }

  cv::Mat tmpFrame = _frame.clone();
  cv::Mat mask;
  cv::Point2i center = {0, 0};
  uint16_t radius = 0;

  _bt.process(_frame, mask, &center, &radius);

  // ROS_DEBUG("RADIUS: %d", radius);

  if (radius != 0) {
    std::stringstream info;

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

      info << "CENTER: " << center << " RADIUS: " << radius;
      cv::Point2f textPos = {center.x + radius + 15.0f, center.y - 15.0f};

      info << " DIST: " << distToBall * 0.001f;

      cv::Point3d newBallPos = _cameraModel.projectPixelTo3dRay(center);
      newBallPos.z = distToBall * 0.001f;

      tf2::Transform trPoint(tf2::Quaternion::getIdentity(), tf2::Vector3(newBallPos.x, newBallPos.y, newBallPos.z));
      tf2::Stamped<tf2::Transform> newPointTs(trPoint, ros::Time::now(), "map");
      geometry_msgs::TransformStamped ts = tf2::toMsg(newPointTs);
      ts.child_frame_id = _cameraModel.tfFrame();

      TransformStamped transform;

      try {
        transform = _tfBuffer.lookupTransform("map", _cameraModel.tfFrame(), ros::Time(0));
      } catch (const tf2::TransformException &e) {
        ROS_ERROR("[BallTrackingRos] %s", e.what());
        ros::Duration(1.0).sleep();
      }

      geometry_msgs::TransformStamped newTs;
      tf2::doTransform(ts, newTs, transform);
      newBallPos.x = newTs.transform.translation.x;
      newBallPos.y = newTs.transform.translation.y;
      newBallPos.z = newTs.transform.translation.z;

      if (_ballTragectory.size() > 10)
        _ballTragectory.pop_front();

      _ballTragectory.push_back(newBallPos);

      if(_ballPos == cv::Point3d(0,0,0)){
        _ballPos = cv::Point3d(newBallPos.x, newBallPos.y, newBallPos.z);
        startPoint = _ballPos;
        startTime = ros::Time::now();
        return;
      }

      tf2::Vector3 ballPoseV(_ballPos.x, _ballPos.y, _ballPos.z);
      tf2::Vector3 newBallPoseV(newBallPos.x, newBallPos.y, newBallPos.z);

      double dt = 0.01; // 12.5hz

      if (ros::Time::now() - _lastDetection >= ros::Duration(dt)) {
        // ROS_INFO("New timer cycle");
        // double velocity = dist / ros::Duration(0.1).toSec();
        // float dist = tf2::tf2Distance2(newBallPoseV, ballPoseV);

        // double vX = ballDirV.x() / ros::Duration(dt).toSec();
        // double vY = ballDirV.y() / ros::Duration(dt).toSec();
        // double vZ = ballDirV.z() / ros::Duration(dt).toSec();

        // ROS_INFO("vX: %lf, xY: %lf vZ: %lf", vX, vY, vZ);
        
        // ROS_INFO("ballDirV.len = %f", ballDirV.length());
        // ROS_INFO("ballDirV.len2 = %f", ballDirV.length2());
        tf2::Vector3 ballDirV = ballPoseV - newBallPoseV;
        // ROS_INFO("ballDirV x %f; y %f; z %f;", ballDirV.x(), ballDirV.y(), ballDirV.z());
        if (ballDirV.length2() != 0) {
          tf2::Vector3 totalLength = newBallPoseV - tf2::Vector3(startPoint.x, startPoint.y, startPoint.z);
          ros::Duration totalTime = ros::Time::now() - startTime;

          double v =  totalLength.length() / totalTime.toSec(); // speed m/s
          double vX = totalLength.x() / totalTime.toSec();
          double vY = totalLength.y() / totalTime.toSec();
          double vZ = totalLength.z() / totalTime.toSec();

          // ROS_INFO("Speed: %f", v);
          // ROS_INFO("Len for speed: %f", totalLength.length());
          // ROS_INFO("newBallPoseV x %f; y %f; z %f;", newBallPoseV.x(), newBallPoseV.y(), newBallPoseV.z());
          // ROS_INFO("ballPoseV x %f; y %f; z %f;", ballPoseV.x(), ballPoseV.y(), ballPoseV.z());
          // double angle = 0.05; ///< angle in radian
          double angle = asin(totalLength.z() / ballDirV.length()); ///< angle in radian
          // double angle = _medianFilter.filtered(tf2Degrees(asin(ballDirV.z() / ballDirV.length())));
          // ROS_INFO("Angle: %f", angle);
          
          tf2::Vector3 tt(newBallPoseV);
          tf2::Vector3 ptt(ballPoseV);
          
          std::list<geometry_msgs::Point> trajectory;
          for (size_t i = 0; i < 5; i++) {
            tf2::Vector3 pDir = ptt - tt;
            // ROS_INFO("pDir x %f; y %f; z %f;", pDir.x(), pDir.y(), pDir.z());
            // ROS_INFO("pDir.len: %f", pDir.length());
            // angle = _medianFilter.filtered(tf2Degrees(asin(pDir.z() / pDir.length())));
            // angle = asin(pDir.z() / pDir.length()); ///< in radian
            // ROS_INFO("angle2: %f", angle);
            
            double timePred = .1;
            
            double dx = vX * ros::Duration(timePred).toSec();
            double gt = (9.8 * powl(ros::Duration(timePred).toSec(), 2)) / 2;
            double dy = (vZ * ros::Duration(timePred).toSec()) - gt;

            // ROS_INFO("dx %f, dy %f", dx, dy);

            ptt = tt;
            tt.setX(tt.getX() + dx);
            tt.setZ(tt.getZ() + dy);

            geometry_msgs::Point p;
            p.x = tt.x();
            p.y = tt.y();
            p.z = tt.z();

            // trajectory.push_back(p);
            _ballPredictedTraj.push_back(p);
            // if (_ballPredictedTraj.size() > 50)
            //   _ballPredictedTraj.pop_front();
          }
          // fixTragectory(_ballPredictedTrajs);

          _context->drawPredicted(_ballPredictedTraj);
          _isFirstDetection = false;
        }
        _lastDetection = ros::Time::now();
      }
      // ROS_INFO("Ball move %f", dist);

      tf2::Quaternion q = tf2::shortestArcQuatNormalize2(ballPoseV, newBallPoseV);

      // tf2::Quaternion q;
      // q.setEulerZYX(ballDirV.z(), ballDirV.y(), ballDirV.x());

      // Utils::fastFilterCvPoint3d(_ballPos, newBallPos, _filterGain);

      geometry_msgs::Pose pose;
      pose.position.x = newBallPos.x;
      pose.position.y = newBallPos.y;
      pose.position.z = newBallPos.z;
      pose.orientation.w = 1;

      _context->drawBallPos(pose);

      pose.orientation.w = q.w();
      pose.orientation.x = q.x();
      pose.orientation.y = q.y();
      pose.orientation.z = q.z();

      // _context->drawBallDiract(pose);

      _context->draBallTrajectory(_ballTragectory);
      _ballPos = cv::Point3d(newBallPos.x, newBallPos.y, newBallPos.z);

      // std::stringstream info2;
      // info2 << _ballPos;
      // ROS_INFO("Pixel in 3d: %s", info2.str().c_str());
    } catch (const cv::Exception &e) {
      std::cerr << e.what() << '\n';
      _context->shutdown();
    }

    /* if (ros::Duration(ros::Time::now() - _timer) <= ros::Duration(1.0)) {
      framesCount++;
    } else {
      fps = framesCount;
      _timer = ros::Time::now();
      framesCount = 0;
    } */

    cv::circle(tmpFrame, center, radius + 7.0, cv::Scalar::all(128), 1,
               cv::LINE_4);
    cv::circle(tmpFrame, center, 3, cv::Scalar(0, 255, 0), cv::FILLED,
               cv::LINE_8);

    _fps = static_cast<int>(
        std::ceil(1.0 / ros::Duration(ros::Time::now() - _timer).toSec()));

    cv::Size textSize = cv::getTextSize(
        std::to_string(_fps), cv::FONT_HERSHEY_SIMPLEX, 0.4, 2, nullptr);
    cv::putText(tmpFrame, std::to_string(_fps), cv::Point(5, textSize.height * 2),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar::all(0));
    cv::putText(tmpFrame, info.str(),
                cv::Point(5, (textSize.height * 4) + textSize.height),
                cv::FONT_HERSHEY_SIMPLEX, .4, cv::Scalar::all(0));
  } else {

    if(ros::Time::now() - resetTimer >= ros::Duration(1.0)) {
      _ballPos = cv::Point3d(0,0,0);
      resetTimer = ros::Time::now();
       _ballPredictedTraj.clear();
    }

  }

  try {
    cv::imshow("test", mask);
    cv::imshow(_winName, tmpFrame);
    cv::waitKey(1);
  } catch (const cv::Exception &e) {
    ROS_ERROR("The video in current environment not available.\n%s", e.what());
  }
  _timer = ros::Time::now();
}