#include "../../../include/PxSitl/Vision/Setup/BallTrackingSetupRos.hpp"

BallTrackingSetupRos::BallTrackingSetupRos(ros::NodeHandle nh, VideoHandler &vh) : _nh(nh), _vh(vh) {

  if (!loadParam()) {
    return;
  }

  _tp = TrackingParam(_confFile.c_str());

  cv::namedWindow(_winName, cv::WINDOW_AUTOSIZE | cv::WINDOW_GUI_NORMAL);
  cv::setMouseCallback(_winName, &BallTrackingSetupRos::onMouseCallbackCv, (void *)this);
}

BallTrackingSetupRos::~BallTrackingSetupRos() { cv::destroyWindow(_winName); }

bool BallTrackingSetupRos::loadParam() { return true; }

void BallTrackingSetupRos::onMouseCallback(int ev, int x, int y, int flag) {
  if (ev == cv::MouseEventTypes::EVENT_LBUTTONUP) {
    if (_roiPoints.size() < 4)
      _roiPoints.push_back({x, y});
  }

  if (ev == cv::MouseEventTypes::EVENT_RBUTTONUP) {
    _roiPoints.clear();
  }

  _cursor = {x, y};
}

void BallTrackingSetupRos::loop() {
  if (_roiPoints.size() == 0)
    _vh >> _frame;

  if (_frame.empty()) {
    ROS_WARN("Frame is empty");
    return;
  }

  if (_roiPoints.size() != 0) {
    _frame.copyTo(_staticFrame);
    for (uint8_t i = 0; i < _roiPoints.size(); i++) {
      cv::Point2i start = _roiPoints.at(i);
      cv::Point2i end;

      if (i + 1 < _roiPoints.size()) {
        end = _roiPoints.at(i + 1);
      } else {
        if (_roiPoints.size() == 4) {
          end = _roiPoints.at(0);

          if (_tp.maskFormPoints(_frame, _roiPoints)) {
            threshold_t tmp;
            _tp.newThreshold(tmp);
            _nh.shutdown();
            ros::shutdown();
          }

        } else
          end = _cursor;
      }
      cv::line(_staticFrame, start, end, cv::Scalar::all(0));
    }
    cv::imshow(_winName, _staticFrame);
  } else {
    cv::imshow(_winName, _frame);
  }

  cv::waitKey(1);
}

// static functions
void BallTrackingSetupRos::onMouseCallbackCv(int ev, int x, int y, int flag, void *uData) {
  BallTrackingSetupRos *tp = static_cast<BallTrackingSetupRos *>(uData);
  tp->onMouseCallback(ev, x, y, flag);
}