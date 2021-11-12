#if !defined(_STATE_TRACKING_H_)
#define _STATE_TRACKING_H_

#include <iostream>

class BallTrackingRos;

class State {
protected:
    BallTrackingRos *_context = nullptr;
public:
  State(BallTrackingRos *context): _context(context) {}
  virtual ~State() {
      std::cout << "Delete state base\n";
  }

  virtual void tracking() = 0;
  virtual void wait() = 0;
};

#endif // _STATE_TRACKING_H_
