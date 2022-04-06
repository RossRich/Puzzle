// #include "../../include/PxSitl/vision/StateInit.hpp"
#include "StateInit.hpp"

/* void StateInit::tracking(BallTrackingRos *context) {
  std::cout << "Transition to tracking form " << toString() << std::endl;
  // context->setState(context->getStateTracking());
} */

void StateInit::wait() {
  std::cout << "Transition to wait form " << toString() << std::endl;
  _context.setState(static_cast<State *>(_context.getStateWait()));
}

void StateInit::init() {
  std::cerr << "Can't transition from init to init\n";
}

void StateInit::execute() {

  

}