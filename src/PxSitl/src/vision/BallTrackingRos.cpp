#include "../../include/PxSitl/Vision/BallTrackingRos.hpp"

BallTrackingRos::BallTrackingRos(ros::NodeHandle &nh, VideoHandler &vh)
    : _nh(nh), _vh(vh) {

  if (!loadParam()) {
    ROS_ERROR("Load parameters error.\nNode init failed.");
    return;
  }

  _bt = BallTracking(_vh.getWidth(), _vh.getHeight(), threshold_t());
  updateDetector();

  cv::namedWindow("MASK", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("TRACKING", cv::WINDOW_AUTOSIZE);
}

BallTrackingRos::~BallTrackingRos() {
  cv::destroyAllWindows();
}

bool BallTrackingRos::loadParam() { return true; }

void BallTrackingRos::updateDetector() {
  TrackingParam tp(_vh);
  threshold_t newTreshold;

  if (!tp.getThreshold(newTreshold)) {
    ROS_INFO("No threshold for detect color. Start setup");

    Mat mask;
    tp.maskFormGUI(mask);
    if (tp.newThreshold(mask)) {
      ROS_INFO("New threshold saved in file");
      if (!tp.getThreshold(newTreshold)) {
        ROS_ERROR("Fatal error. New threshold not readable from file");
        return;
      }

      _bt.setThreshold(newTreshold);
      ROS_INFO("Threshold updated");
    } else {
      ROS_ERROR("Fatal error. New threshold not readable from file");
      return;
    }
  }

  ROS_INFO("Threshold updated");
}

void BallTrackingRos::tracking() {
  uint16_t radius = 0;
  Point2i center = {0};
  cv::Mat frame;
  cv::Mat m;

  _vh >> frame;

  if (frame.empty()) {
    ROS_WARN("Empty frame");
    return;
  }

  _bt.process(frame, m, &center, &radius);

  cv::circle(frame, center, radius + 7.0, cv::Scalar(0, 255, 0), 1, cv::LINE_4);
  cv::circle(frame, center, 3, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);
  // cv::Point2f textPos = {center.x + radius + 15.0f, center.y + radius
  // + 15.0f}; cv::putText(frame, _info.str(), textPos,
  // cv::FONT_HERSHEY_SIMPLEX, .6, cv::Scalar::all(0), 2);

  // cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);

  cv::imshow("MASK", m);
  cv::imshow("TRACKING", frame);
  cv::waitKey(1);
}