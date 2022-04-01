// #include "../../include/PxSitl/vision/StateWait.hpp"
#include "BallTrackingRos.hpp"
#include "State.hpp"
#include "StateWait.hpp"

//void StateWait::tracking(BallTrackingRos *context) {
//  std::cout << "Transition to tracking form " << toString() << std::endl;
  // context->setState(context->getStateTracking());
  /* StrategyTracking *ts = new StrategyTracking(_context->getVideoHandler(), _context);

  if (!ts->init()) {
    delete ts;
    ROS_WARN("Translation to tracking strategy is rejected");
    return;
  }

  ROS_INFO("Go to tracking strategy");
  ts->setFilterGain(_context->getFilterGain());
  _context->setStrategy(ts);
  _context->setState(new StateTracking(_context)); */
  // context->setState(new StateTracking());
//}

void StateWait::wait(BallTrackingRos *context) {
  std::cerr << "Can't transition from wait to wait\n";
  // ROS_INFO("In wait state");
}

void StateWait::init(BallTrackingRos *context) {
  std::cout << "Transition to init form " << toString() << std::endl;
  context->setState(static_cast<State *>(context->getStateInit()));

  // ROS_INFO("%s state", toString());
  // context->setState(con);
}

void StateWait::execute() {
  return;
}