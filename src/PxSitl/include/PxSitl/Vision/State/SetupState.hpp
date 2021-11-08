#if !defined(_SETUP_STATE_H_)
#define _SETUP_STATE_H_

#include "../BallTrackingRos.hpp"
#include "../Strategy/SetupStrategy.hpp"
#include "TrackingState.hpp"

class SetupState : public State {
private:
public:
  SetupState() {}
  ~SetupState() {}

  void setup() override {
    std::cout << "In setup mode\n";
  }

  void tracking() override {
    _context->transitionTo(new TrackingState());
    // _context->setStrategy(new SetupStrategy(_context->getVideoHandler()));
    std::cout << "Transition to tracking mode\n";
  }
};

#endif // _SETUP_STATE_H_
