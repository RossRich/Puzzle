#include "puzzle_transform/PuzzleTransform.hpp"

#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "puzzle_transform");
  ros::NodeHandle nh;

  std::unique_ptr<PuzzleTransform> puzzleTransform;

  try {
    puzzleTransform = std::make_unique<PuzzleTransform>(nh);
  } catch (const std::exception &e) {
    ROS_ERROR_STREAM("[PuzzleTransformNode::main] " << e.what());
    return EXIT_FAILURE;
  }

  ros::Rate loop_rate(250);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}