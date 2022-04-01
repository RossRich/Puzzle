#if !defined(_STATE_VISION_H_)
#define _STATE_VISION_H_

#include <iostream>
#include "BallTrackingRos.hpp"

class State {
protected:
  std::string _name = "";

public:
  State(std::string name) : _name(name) {
    std::cout << "New state " << name << std::endl;
  }

  virtual ~State() {
    std::cout << "Delete state base: " << _name << std::endl;
  }

  virtual std::string toString() { return _name; }

  // virtual void tracking(BallTrackingRos *context) = 0;
  virtual void wait(BallTrackingRos *context) = 0;
  virtual void init(BallTrackingRos *context) = 0;
  virtual void execute() = 0;
};

// void State::tracking(BallTrackingRos *context) {}

#endif // _STATE_VISION_H_
