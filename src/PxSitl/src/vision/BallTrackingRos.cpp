#include "../../include/PxSitl/Vision/BallTrackingRos.hpp"

BallTrackingRos::BallTrackingRos(ros::NodeHandle &nh, VideoHandler &vh) : _nh(nh), _vh(vh) {

  if (!loadParam()) {
    ROS_ERROR("Load parameters error.\nNode init failed.");
    return;
  }

  transitionTo(new TrackingState());

  // _bt = BallTracking(_vh.getWidth(), _vh.getHeight(), threshold_t());
  setup();

  _strategySrv = _nh.advertiseService("strategy_srv", &BallTrackingRos::runSetupSrv, this);
  ROS_INFO("Change strategy server ready");

  cv::namedWindow("MASK", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("TRACKING", cv::WINDOW_AUTOSIZE);
}

BallTrackingRos::~BallTrackingRos() {
  cv::destroyWindow("MASK");
  cv::destroyWindow("TRACKING");
  delete _strategy;
  _strategy = nullptr;
  delete _state;
  _state = nullptr;
}

void BallTrackingRos::transitionTo(State *state) {
  if (state != nullptr) {
    delete _state;
    _state = state;
    _state->setContext(this);
    std::cout << "Transition to next state\n";
  }
}

void BallTrackingRos::setStrategy(Strategy *strategy) {
  if (strategy != nullptr) {
    delete _strategy;
    _strategy = strategy;
  }
}

// void BallTrackingRos::toSetupStrategy() { transitionTo(new SetupState()); }

// void BallTrackingRos::toTrackingStrategy() {
// transitionTo(new TrackingState());
// }

void BallTrackingRos::test1() { _state->setup(); }

void BallTrackingRos::test2() { _state->tracking(); }

bool BallTrackingRos::loadParam() { return true; }

bool BallTrackingRos::runSetupSrv(std_srvs::EmptyRequest &request, std_srvs::EmptyResponse &response) {
  // toSetupStrategy();
  test1();
  return true;
}

void BallTrackingRos::setup() {
  TrackingParam tp(_vh);
  threshold_t newTreshold;

  if (!tp.getThreshold(newTreshold)) {
    ROS_INFO("No threshold for detect color. Start setup");
    // toSetupStrategy();
    test1();

    /* cv::Mat mask;
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
    } */
  } else {
    ROS_INFO("Go to tracking strategy");
    // toTrackingStrategy();
    test2();
  }
}

void BallTrackingRos::tracking() {
  /* uint16_t radius = 0;
  cv::Point2i center = {0};
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
/* 
  cv::imshow("MASK", m);
  cv::imshow("TRACKING", frame);
  cv::waitKey(1); */
}

void BallTrackingRos::run() {
  if (_strategy != nullptr)
    _strategy->execute();
  else
    ROS_INFO("No strategy");
}

void SetupState::setup() { std::cout << "In setup mode\n"; }

void SetupState::tracking() {
  _context->transitionTo(new TrackingState());
  // _context->setStrategy(new SetupStrategy(_context->getVideoHandler()));
  std::cout << "Transition to tracking mode\n";
}

void TrackingState::setup() {
  _context->transitionTo(new SetupState());
  std::cout << "Transition to setup mode\n";
}

void TrackingState::tracking() { std::cout << "In tracking mode\n"; }