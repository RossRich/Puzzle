#include "../../include/PxSitl/Vision/BallTrackingRos.hpp"

BallTrackingRos::BallTrackingRos(ros::NodeHandle &nh, VideoHandler &vh)
    : _nh(nh), _vh(vh) {

  if (!loadParam()) {
    ROS_ERROR("Load parameters error.\nNode init failed.");
    return;
  }
}

BallTrackingRos::~BallTrackingRos() {
  // cv::destroyAllWindows();
  delete _strategy;
  delete _state;
}

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

/* void BallTrackingRos::updateDetector() {
  TrackingParam tp(_vh);
  threshold_t newTreshold;

  if (!tp.getThreshold(newTreshold)) {
    ROS_INFO("No threshold for detect color. Start setup");

    Mat mask;
    tp.maskFormGUI(mask);
    if (tp.newThreshold(mask)) {
      ROS_INFO("New threshold saved in file");
      if (!tp.getThreshold(newTreshold)) {
        ROS_ERROR("Fatal error. New threshold not readable from file");
        return;
      }

      _bt.setThreshold(newTreshold);
      ROS_INFO("Threshold updated");
    } else {
      ROS_ERROR("Fatal error. New threshold not readable from file");
      return;
    }
  }

  ROS_INFO("Threshold updated");
} */

void BallTrackingRos::tracking() {

  _state->tracking();

  /* uint16_t radius = 0;
  Point2i center = {0};
  cv::Mat frame;
  cv::Mat m;

  _vh >> frame;

  if (frame.empty()) {
    ROS_WARN("Empty frame");
    return;
  }

  _bt.process(frame, m, &center, &radius);

  cv::circle(frame, center, radius + 7.0, cv::Scalar(0, 255, 0), 1, cv::LINE_4);
  cv::circle(frame, center, 3, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);
*/
  // cv::Point2f textPos = {center.x + radius + 15.0f, center.y + radius
  // + 15.0f}; cv::putText(frame, _info.str(), textPos,
  // cv::FONT_HERSHEY_SIMPLEX, .6, cv::Scalar::all(0), 2);

  // cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);

  /* cv::imshow("MASK", m);
  cv::imshow("TRACKING", frame); */
}

void BallTrackingRos::wait() { _state->wait(); }

void BallTrackingRos::loop() {
  if (_strategy != nullptr)
    _strategy->execute();
}

void StateWait::tracking() {
  StrategyTracking *ts =
      new StrategyTracking(_context->getVideoHandler(), _context);

  if (!ts->init()) {
    ROS_WARN("translation to tracking strategy is rejected");
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