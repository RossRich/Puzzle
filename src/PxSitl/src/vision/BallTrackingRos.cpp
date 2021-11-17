#include "../../include/PxSitl/Vision/BallTrackingRos.hpp"

BallTrackingRos::BallTrackingRos(ros::NodeHandle &nh, VideoHandler &vh) : _nh(nh), _vh(vh) {

  if (!loadParam()) {
    ROS_ERROR("Load parameters error.\nNode init failed.");
    return;
  }

  _strategySrv = _nh.advertiseService("strategy_srv", &BallTrackingRos::runSetupSrv, this);
  ROS_INFO("Change strategy server ready");
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

  return true;
}

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
    ROS_WARN("Translation to tracking strategy is rejected");
    return;
  }

  ROS_INFO("Go to tracking strategy");
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

  return true;
}

void StrategyTracking::execute() {
  _vh >> _frame;
  _vh.readDepth(_depth);

  if (_frame.empty() || _depth.empty()) {
    ROS_WARN("Frame is empty");
    return;
  }

  cv::Mat mask;
  cv::Point2i center;
  uint16_t radius;

  _bt.process(_frame, mask, &center, &radius);

  if (radius != 0) {
    cv::circle(_frame, center, radius + 7.0, cv::Scalar::all(128), 1, cv::LINE_4);
    cv::circle(_frame, center, 3, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);

    std::stringstream info;
    info << "CENTER: " << center << " RADIUS: " << radius << std::endl;

    cv::Point2f textPos = {center.x + radius + 15.0f, center.y + radius + 15.0f};
    cv::putText(_frame, info.str(), textPos, cv::FONT_HERSHEY_SIMPLEX, .6, cv::Scalar::all(0), 2);

    // cv::Mat ballDepth (_depth, mask);
    if (ros::Duration(ros::Time::now() - _timer) >= _timeOut) {
      try {
        uint16_t posRadius = radius * 2;
        cv::Mat ballDist(cv::Size2i(posRadius, posRadius), CV_16UC1, cv::Scalar::all(0));
        _depth.copyTo(ballDist, mask);

        std::map<uint16_t, uint16_t> mapOfDist;
        for (auto &&d : cv::Mat_<uint16_t>(ballDist)) {
          if (mapOfDist.count(d) > 0)
            mapOfDist.at(d) += 1;
          else
            mapOfDist.insert(std::pair<uint16_t, uint16_t>(d, 0));
        }

        std::multimap<uint16_t, uint16_t> inv;
        for (auto &&i : mapOfDist)
          inv.insert(std::pair<uint16_t, uint16_t>(i.second, i.first));
      

        for (auto &&ii : inv)
          std::cout << ii.first << ": " << ii.second << std::endl;

        uint16_t distToBall;
        for (std::multimap<uint16_t, uint16_t>::const_iterator i = inv.cend(); i != inv.cbegin(); i--)
        {
          if(i->second <= 100)
            continue;

          distToBall = i->second;
          break;
        }

        std::cout << "Hight accur: " << distToBall * 0.001f << std::endl;
        std::cout << "Raw" << _depth.at<uint16_t>(center) * 0.001f << std::endl;
      } catch (const cv::Exception &e) {
        std::cerr << e.what() << '\n';
        _context->shutdown();
      }
      _timer = ros::Time::now();
    }
  }

  try {
    cv::imshow(_winName, _frame);
    cv::waitKey(1);
  } catch (const cv::Exception &e) {
    ROS_ERROR("The video in current environment not available.\n%s", e.what());
  }
}
