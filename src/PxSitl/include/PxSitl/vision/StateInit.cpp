// #include "../../include/PxSitl/vision/StateInit.hpp"
#include "BallTrackingRos.hpp"
#include "State.hpp"
#include "StateInit.hpp"

/* void StateInit::tracking(BallTrackingRos *context) {
  std::cout << "Transition to tracking form " << toString() << std::endl;
  // context->setState(context->getStateTracking());
} */

void StateInit::wait(BallTrackingRos *context) {
  std::cout << "Transition to wait form " << toString() << std::endl;
  context->setState(static_cast<State *>(context->getStateWait()));
}

void StateInit::init(BallTrackingRos *context) {
  std::cerr << "Can't transition from init to init\n";
}

void StateInit::execute() {}