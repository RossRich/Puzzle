#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using cv::Mat;
using std::cerr;
using std::cout;
using std::endl;

typedef cv::Vec<cv::Scalar_<uint8_t>, 2> threshold_t;

class TrackingParam {
private:
  cv::VideoCapture _vc;
  const char *_winName = "Main";
  static const char *_confFile;
  uint8_t _roiPointsNum = 0;
  std::vector<cv::Point2i> _roiPoints;
  cv::Point2i _cursor;

public:
  TrackingParam();
  ~TrackingParam();

  void onMouseCallnack(int ev, int x, int y, int flag);
  threshold_t scanNewThreshold();
  threshold_t getThreshold();
};

const char *TrackingParam::_confFile = "/workspaces/Puzzle/src/PxSitl/data/config.yaml";

TrackingParam::TrackingParam() {}

TrackingParam::~TrackingParam() { cv::destroyAllWindows(); }

void onMouseCallback(int ev, int x, int y, int flag, void *uData) {
  TrackingParam *tp = static_cast<TrackingParam *>(uData);
  tp->onMouseCallnack(ev, x, y, flag);
}

threshold_t TrackingParam::getThreshold() {
  cv::FileStorage conf;
  threshold_t threshold = {0};
  bool isDataAvalable = false;

  try {
    if (conf.open(_confFile, cv::FileStorage::READ)) {
      cv::FileNode thresholdNode = conf["threshold"];
      if (!thresholdNode.empty()) {
        int i = 0;
        for (auto t : thresholdNode)
          thresholdNode[t.name()] >> threshold[i++];

        isDataAvalable = true;
      }
    }

    conf.release();
  } catch (cv::Exception e) {
    std::cerr << e.what() << '\n';
  }

  if (!isDataAvalable) {
    std::cout << "No available threshold data in file" << std::endl;
    std::cout << "Start scan..." << endl;

  } else {
    std::cout << "Threshold is available" << std::endl;
  }

  return threshold;
}

threshold_t TrackingParam::scanNewThreshold() {
  Mat frame;
  Mat tmpFrame;

  if (!_vc.open(6)) {
    cerr << "Can't open video stream" << endl;
    return {};
  }

  cv::namedWindow(_winName, cv::WINDOW_AUTOSIZE);
  cv::setMouseCallback(_winName, onMouseCallback, this);

  char c = ' ';
  bool isScaning = true;
  while (c != 'q') {

    if (isScaning)
      _vc >> frame;
    else
      frame = tmpFrame.clone();

    if (frame.empty())
      break;

    cv::resize(frame, frame, cv::Size2i(1280, 760));

    if (_roiPointsNum != 0) {
      for (uint8_t i = 0; i < _roiPointsNum; i++) {
        cv::Point2i start = _roiPoints.at(i);
        cv::Point2i end;

        if (i + 1 < _roiPointsNum) {
          end = _roiPoints.at(i + 1);
        } else {
          if (_roiPointsNum == 4) {
            end = _roiPoints.at(0);
            Mat mask = Mat::zeros(frame.size(), CV_8UC1);
            try {
              cv::fillConvexPoly(mask, _roiPoints, cv::Scalar::all(255));
              // cv::bitwise_not(mask, frame);
              tmpFrame.copyTo(frame, mask);
            } catch (cv::Exception &e) {
              cerr << e.what() << endl;
            }

          } else
            end = _cursor;
        }

        cv::line(frame, start, end, cv::Scalar::all(0));
      }
    }

    cv::imshow(_winName, frame);

    c = (char)cv::waitKey(1);

    if (c == 's') {
      isScaning = !isScaning;
      tmpFrame = frame.clone();
    }
  }

  _vc.release();

  return {0};
}

void TrackingParam::onMouseCallnack(int ev, int x, int y, int flag) {
  if (ev == cv::MouseEventTypes::EVENT_LBUTTONUP) {
    if (_roiPointsNum < 4) {
      _roiPoints.push_back({x, y});
      _roiPointsNum++;
    }
  }

  if (ev == cv::MouseEventTypes::EVENT_RBUTTONUP) {
    _roiPointsNum = 0;
    _roiPoints.clear();
  }

  _cursor = {x, y};
}

int main(int argc, char const *argv[]) {
  TrackingParam tp;
  tp.scanNewThreshold();
  return 0;
}
