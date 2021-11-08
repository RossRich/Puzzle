#if !defined(_SETUP_STATE_H_)
#define _SETUP_STATE_H_

#include "../BallTrackingRos.hpp"

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
    std::cout << "Transition to tracking mode\n";
  }
};

#endif // _SETUP_STATE_H_
