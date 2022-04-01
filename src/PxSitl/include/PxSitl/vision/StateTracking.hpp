#if !defined(_VISION_STATE_TRACKING_H_)
#define _VISION_STATE_TRACKING_H_

#include "State.hpp"
#include "RosVH.hpp"
#include "VideoHandler.hpp"
#include <geometry_msgs/Point.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_srvs/Empty.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

using sensor_msgs::CameraInfo;
using sensor_msgs::CameraInfoConstPtr;
using visualization_msgs::Marker;
using visualization_msgs::MarkerConstPtr;

class StateTracking : public State {
private:
  // ros::NodeHandle &_nh;
  std::string _confFile = "/workspaces/Puzzle/src/PxSitl/data/config.yaml";
  float _filterGain;
  ros::Publisher _ballPub;
  CameraInfoConstPtr _cameraInfo;
  

public:
  StateTracking(): State("Tracking") {
    // _ballPub = _nh.advertise<Marker>("ball", 5, false);
  }
  ~StateTracking() {
    std::cout << "Delete state tracking\n";
  }

 
  void drawBallPos(geometry_msgs::Pose p);
  void drawBallDiract(geometry_msgs::Pose p);
  void drawPredicted(std::list<geometry_msgs::Point> list);
  void draBallTrajectory(std::list<cv::Point3d> &bt);
  void pubMarker(Marker m);

  void tracking(BallTrackingRos *) override;
  void wait(BallTrackingRos *) override;
  void init(BallTrackingRos *) override;
  void execute() override;
};

#endif // _VISION_STATE_TRACKING_H_
