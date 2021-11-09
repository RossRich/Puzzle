#if !defined(_STRATEGY_H_)
#define _STRATEGY_H_

#include <iostream>

class Strategy {
private:
public:
  Strategy() {}
  virtual ~Strategy() {
    std::cout << "Delete base strategy\n";
  }
  
  void test() {}

  virtual void execute() = 0;
};

#endif // _STRATEGY_H_
