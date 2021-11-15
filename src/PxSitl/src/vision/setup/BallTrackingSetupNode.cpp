#include "../../../include/PxSitl/Vision/RosVH.hpp"
#include "../../../include/PxSitl/Vision/Setup/BallTrackingSetupRos.hpp"
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "BallTrackingSetup");
  ros::NodeHandle nh;

  nh.setParam("color_topic", "/camera/color/image_raw");
  nh.setParam("depth_topic", "/camera/aligned_depth_to_color/image_raw");

  image_transport::ImageTransport it(nh);
  RosVH videoHandler(nh, it, 640, 480);
  BallTrackingSetupRos tp(nh, videoHandler);
  ros::Rate loop_rate(30);

  while (ros::ok()) {
    tp.loop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  std::cout << "Threshold updated\n";
}