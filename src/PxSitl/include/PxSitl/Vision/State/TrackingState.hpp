#if !defined(_TRACKING_STATE_H_)
#define _TRACKING_STATE_H_

#include "../BallTrackingRos.hpp"
#include "../Strategy/TrakcingStrategy.hpp"

class TrackingState: public State
{
private:
  
public:
  TrackingState() {}
  ~TrackingState() {}

  void setup() override {
    _context->transitionTo(new SetupState());
    std::cout << "Transition to setup mode\n";
  }

  void tracking() override {
    std::cout << "In tracking mode\n";
  }
};

#endif // _TRACKING_STATE_H_
