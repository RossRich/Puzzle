#if !defined(_STATE_VISION_H_)
#define _STATE_VISION_H_

#include <iostream>

class BallTrackingRos;

class State {
protected:
  BallTrackingRos &_context;
  std::string _name = "";

public:
  State(BallTrackingRos &context, std::string name) : _context(context), _name(name) {
    std::cout << "New state " << name << std::endl;
  }

  virtual ~State() {
    std::cout << "Delete state base: " << _name << std::endl;
  }

  virtual std::string toString() { return _name; }

  virtual void tracking() = 0;
  virtual void wait() = 0;
  virtual void execute() = 0;
};

// void State::tracking(BallTrackingRos *context) {}

#endif // _STATE_VISION_H_
