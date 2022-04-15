#if !defined(_BALL_TRACKING_SETUP_H_)
#define _BALL_TRACKING_SETUP_H_

#include "../VideoHandler.hpp"
#include "TrackingParam.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

class BallTrackingSetupRos {
private:
  const char *_winName = "Setup";

  std::string _confFile = "/workspaces/Puzzle/src/PxSitl/data/config.yaml";
  std::vector<cv::Point2i> _roiPoints;
  cv::Point2i _cursor;
  cv::Mat _frame;
  cv::Mat _staticFrame;

  TrackingParam _tp;
  VideoHandler &_vh;
  ros::NodeHandle _nh;

public:
  BallTrackingSetupRos(ros::NodeHandle nh, VideoHandler &vh);
  ~BallTrackingSetupRos();

  bool loadParam();
  void onMouseCallback(int ev, int x, int y, int flag);
  static void onMouseCallbackCv(int ev, int x, int y, int flag, void *uData);
  void loop();
};

#endif // _BALL_TRACKING_SETUP_H_