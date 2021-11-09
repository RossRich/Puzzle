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

  void setup() override; 
  void tracking() override;
};

#endif // _SETUP_STATE_H_
