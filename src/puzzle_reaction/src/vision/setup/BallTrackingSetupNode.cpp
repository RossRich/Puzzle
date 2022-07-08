#include "puzzle_reaction/vision/RosVH.hpp"
#include "puzzle_reaction/vision/setup/BallTrackingSetupRos.hpp"
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "BallTrackingSetup");
  ros::NodeHandle nh;
  RosVH videoHandler;
  image_transport::ImageTransport it(nh);

  try {
    videoHandler = RosVH(nh, it, 640, 480);
  } catch (const ros::Exception &e) {
    ROS_ERROR_STREAM("[RosVH] " << e.what());
    ros::shutdown();
  }

  if(!videoHandler.isValid())
    return EXIT_FAILURE;

  BallTrackingSetupRos tp(nh, videoHandler);
  ros::Rate loop_rate(30);
  while (ros::ok()) {
    tp.loop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO("[BallTrackingSetupNode] Threshold updated");
  return 0;
}