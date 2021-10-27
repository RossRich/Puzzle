#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using std::cerr;
using std::cout;
using std::endl;

class TrackingParam {
private:
  cv::VideoCapture _vc;
  const char *_winName = "Main";

public:
  TrackingParam();
  ~TrackingParam();

  void run();
};

TrackingParam::TrackingParam() {
  cv::namedWindow(_winName, cv::WINDOW_AUTOSIZE);
}

TrackingParam::~TrackingParam() {
  _vc.release();
  cv::destroyAllWindows();
}

void TrackingParam::run() {
  if (!_vc.open(4)) {
    cerr << "Can't open video stream" << endl;
    return;
  }

  char c = ' ';
  cv::Mat frame;
  while (c != 'q') {
    _vc >> frame;

    if (frame.empty())
      return;

    cv::imshow(_winName, frame);

    c = (char)cv::waitKey(1);
  }
}

int main(int argc, char const *argv[]) {
  TrackingParam tp;
  tp.run();
  return 0;
}
