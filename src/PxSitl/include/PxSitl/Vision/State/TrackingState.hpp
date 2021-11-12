#if !defined(_TRACKING_STATE_H_)
#define _TRACKING_STATE_H_

#include "../BallTrackingRos.hpp"
#include "../Strategy/TrakcingStrategy.hpp"
#include "SetupState.hpp"

class TrackingState: public State
{
private:
  
public:
  TrackingState() {}
  ~TrackingState() {
    std::cout << "Delete state tracking\n";
  }

  void setup() override;
  void tracking() override;
};

#endif // _TRACKING_STATE_H_
