#include "../../include/PxSitl/vision/BallTrackingRos.hpp"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "ball_tracking");
  ros::NodeHandle nh("ball_tracking");

  BallTrackingRos *ballTrackingRos = BallTrackingRos::getInstanse(nh);

  if(ballTrackingRos == nullptr)
    EXIT_FAILURE;

  ros::Rate loop_rate(250);

  while (nh.ok()) {
    ballTrackingRos->loop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  delete ballTrackingRos;

  return 0;
}