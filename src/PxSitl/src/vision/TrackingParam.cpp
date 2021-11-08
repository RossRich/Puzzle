#include "../../include/PxSitl/Vision/TrackingParam.hpp"

const char *TrackingParam::_confFile = "/workspaces/Puzzle/src/PxSitl/data/config.yaml";

void onMouseCallbackCv(int ev, int x, int y, int flag, void *uData) {
  TrackingParam *tp = static_cast<TrackingParam *>(uData);
  tp->onMouseCallback(ev, x, y, flag);
}

TrackingParam::TrackingParam(VideoHandler &vh) : _vh(vh) {}

TrackingParam::~TrackingParam() { cv::destroyAllWindows(); }

bool TrackingParam::getThreshold(threshold_t &td) {
  cv::FileStorage conf;
  bool isDataAvalable = false;

  try {
    if (conf.open(_confFile, cv::FileStorage::READ)) {
      cv::FileNode thresholdNode = conf["threshold"];
      if (!thresholdNode.empty()) {
        int i = 0;
        for (auto t : thresholdNode)
          thresholdNode[t.name()] >> td[i++];

        isDataAvalable = true;
      }
    }

    conf.release();
  } catch (cv::Exception e) {
    std::cerr << e.what() << std::endl;
  }

  if (!isDataAvalable) {
    std::cerr << "No available threshold data in file" << std::endl;
  } else {
    std::cout << "Threshold is available" << std::endl;
  }

  return isDataAvalable;
}

void TrackingParam::maskFormGUI(Mat &mask) {

  std::cout << "Use a keyboard for navigation\n";
  std::cout << "s - stop/start a video\nleft mouse button - set keypoint\nright mouse button - reset keypoints";
  std::cout << "q - exit\n";
  std::cout << "Stop a video and select region of interest\n";

  cv::namedWindow(_winName, cv::WINDOW_AUTOSIZE);
  cv::setMouseCallback(_winName, onMouseCallbackCv, this);

  char c = ' ';
  bool isScaning = true;
  Mat tmpFrame;
  mask = Mat::zeros(_vh.getVideoSize(), CV_8UC1);
  uint16_t skippedFrames = 0;
  cv::Mat _frame1;

  while (c != 'q') {

    if (isScaning)
      _vh >> _frame1;
    else
      _frame1 = tmpFrame.clone();

    if (_frame1.empty()){
      std::cout << "Frame is empty. No video stream" << std::endl;
      skippedFrames++;

      if(skippedFrames >= _counter)
        break;
      else
        continue;
    }

    // cv::resize(_frame, mask, cv::Size2i(640, 480));

    if (_roiPointsNum != 0 && !isScaning) { 
      for (uint8_t i = 0; i < _roiPointsNum; i++) {
        cv::Point2i start = _roiPoints.at(i);
        cv::Point2i end;

        if (i + 1 < _roiPointsNum) {
          end = _roiPoints.at(i + 1);
        } else {
          if (_roiPointsNum == 4) {
            end = _roiPoints.at(0);

            try {
              cv::GaussianBlur(_frame, _frame, cv::Size2i(9, 9), 1);
              cv::fillConvexPoly(mask, _roiPoints, cv::Scalar::all(255));
            } catch (cv::Exception &e) {
              cerr << e.what() << endl;
              _roiPoints.clear();
              mask = 0;
            }
          } else
            end = _cursor;
        }
        cv::line(_frame, start, end, cv::Scalar::all(0));
      }
    }

    cv::imshow(_winName, _frame1);

    c = (char)cv::waitKey(1);

    if (c == 's') {
      isScaning = !isScaning;
      tmpFrame = _frame.clone();
    }
  }

  _frame = tmpFrame.clone();
  cv::destroyWindow(_winName);
}

bool TrackingParam::newThreshold(Mat &mask) {
  if (mask.empty() || _frame.empty())
    return false;

  Mat roi;
  _frame.copyTo(roi, mask);

  cv::cvtColor(roi, roi, cv::COLOR_BGR2HSV);

  Mat channels[3];

  cv::split(roi, channels);

  double min, max;
  int idxMin, idxMax;

  cv::Scalar_<uint8_t> maxThresh = cv::Scalar::all(0);
  cv::Scalar_<uint8_t> minThresh = maxThresh;

  for (uint8_t i = 0; i < 3; i++) {
    cv::minMaxIdx(channels[i], &min, &max, &idxMin, &idxMax, mask);
    cout << "In layer " << int(i) << " min val: " << min << " max val: " << max << endl;
    maxThresh[i] = static_cast<uint8_t>(max);
    minThresh[i] = static_cast<uint8_t>(min);
  }

  _threshold[0] = minThresh;
  _threshold[1] = maxThresh;

  cout << "New threshold\n";
  cout << "Min: " << _threshold[0] << endl;
  cout << "Max: " << _threshold[1] << endl;

  cv::FileStorage conf;
  try {
    conf.open(_confFile, cv::FileStorage::WRITE);
  } catch (const cv::Exception &e) {
    std::cerr << e.what() << '\n';
    return false;
  }

  /*   if (!conf.isOpened()) {
      std::cerr << "Faild open file in " << _confFile << endl;
      conf.release();
      return false;
    } */

  conf << "threshold"
       << "{";
  conf << "min" << minThresh;
  conf << "max" << maxThresh;
  conf << "}";

  conf.release();

  return true;
}

void TrackingParam::onMouseCallback(int ev, int x, int y, int flag) {
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
