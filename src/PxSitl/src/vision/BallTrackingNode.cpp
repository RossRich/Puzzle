#include "../../include/PxSitl/Vision/BallTrackingRos.hpp"
#include "../../include/PxSitl/Vision/RosVH.hpp"

using sensor_msgs::CameraInfo;
using sensor_msgs::CameraInfoConstPtr;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "ball_tracking");
  ros::NodeHandle nh("ball_tracking");

  int cameraWidth, cameraHeight = 0;
  ros::param::param<int>("/camera/realsense2_camera/color_width", cameraWidth, 0);
  ros::param::param<int>("/camera/realsense2_camera/color_height", cameraHeight, 0);

  if (cameraHeight <= 0 || cameraWidth <= 0) {
    ROS_INFO("No camera resolution in params. Trying to request resolution from CameraInfo...");
    CameraInfoConstPtr cameraInfo;
    try {
      cameraInfo = ros::topic::waitForMessage<CameraInfo>("/camera/color/camera_info", nh, ros::Duration(5));
    } catch (const ros::Exception &e) {
      ROS_ERROR("%s", e.what());
      ROS_ERROR("No camera rosolution. Exit.");
      return EXIT_FAILURE;
    }

    if(!cameraInfo) {
      ROS_ERROR("No camera rosolution. Exit.");
      return EXIT_FAILURE;
    }

    cameraWidth = cameraInfo->width;
    cameraHeight = cameraInfo->height;
  }

  if (cameraHeight <= 0 || cameraWidth <= 0) {
    ROS_ERROR("No camera rosolution. Exit.");
    return EXIT_FAILURE;
  }

  ROS_INFO("Camera resolution received. Width = %i, height = %i", cameraWidth, cameraHeight);

  image_transport::ImageTransport it(nh);
  RosVH videoHandler(nh, it, cameraWidth, cameraHeight);
  BallTrackingRos ballTracking(nh, videoHandler);
  ballTracking.setState(new StateWait(&ballTracking));
  ballTracking.setStrategy(new StrategyWait(&ballTracking));
  ballTracking.wait();

  ros::Rate loop_rate(30);
  
  while (ros::ok()) {
    ballTracking.loop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}