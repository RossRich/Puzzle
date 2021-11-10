#if !defined(_VISION_STRATEGY_H_)
#define _VISION_STRATEGY_H_

class Strategy {
private:
public:
  Strategy() {}
  virtual ~Strategy() {
    std::cout << "Delete strategy base\n";
  }

  virtual void execute() = 0;
};

#endif // _VISION_STRATEGY_H_
