#if !defined(_STATE_H_)
#define _STATE_H_

#include <iostream>

class BallTrackingRos;

class State {
protected:
  BallTrackingRos *_context = nullptr;

public:
  State() {}
  virtual ~State() {
    std::cout << "Delete base state\n";
  }

  void setContext(BallTrackingRos *context) {
    if (context != nullptr && _context != context)
      _context = context;
  }

  virtual void setup() = 0;
  virtual void tracking() = 0;
};

#endif // _STATE_H_
