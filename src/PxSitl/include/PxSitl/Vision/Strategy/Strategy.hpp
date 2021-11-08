#if !defined(_STRATEGY_H_)
#define _STRATEGY_H_

class Strategy {
private:
public:
  Strategy() {}
  virtual ~Strategy() {}

  virtual void execute() = 0;
};

#endif // _STRATEGY_H_
