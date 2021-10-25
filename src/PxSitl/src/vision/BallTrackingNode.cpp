#include "../../include/PxSitl/vision/BallTrackingRos.hpp"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "ball_tracking");

  ros::NodeHandle nh("ball_tracking");
  image_transport::ImageTransport it(nh);

  BallTrackingRos ballTracking(nh, it);

  ros::Rate loop_rate(5);
  while (ros::ok()) {
    // ballTracking.run();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}