#include "PxSitl/vision/BallTrackingRos.hpp"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "ball_tracking");
  ros::NodeHandle nh("ball_tracking");

  auto ballTrackingRos = std::make_unique<BallTrackingRos>(nh);

  ros::Rate loop_rate(250);

  while (nh.ok()) {
    ballTrackingRos->loop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}