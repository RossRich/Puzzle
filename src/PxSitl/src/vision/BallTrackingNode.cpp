#include "../../include/PxSitl/vision/BallTrackingRos.hpp"
#include "../../include/PxSitl/vision/RosVH.hpp"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "ball_tracking");

  ros::NodeHandle nh("ball_tracking");
  image_transport::ImageTransport it(nh);

  if (!nh.getParam("/camera/realsense2_camera/color_width", /camera/realsense2_camera/color_width))
    { ROS_ERROR("No /camera/realsense2_camera/color_width param"); }
  

  RosVH videoHandler(nh, it, )

  BallTrackingRos ballTracking(nh, it);

  ros::Rate loop_rate(5);
  while (ros::ok()) {
    // ballTracking.run();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}