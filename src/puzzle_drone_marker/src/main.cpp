#include "puzzle_drone_marker/RvizPainterObjects.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <puzzle_common/RvizPainter.hpp>
#include <ros/ros.h>

RvizPainterObjects painterObjects;
std::unique_ptr<RvizPainter> rvizPainter;

void callback(const geometry_msgs::PoseStampedConstPtr &pose) {
  rvizPainter->draw(painterObjects.getDroneMarker(), pose->pose);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "drone_marker");
  ros::NodeHandle nh;

  ros::Subscriber mavrosLocalPosition = nh.subscribe<geometry_msgs::PoseStamped>("/iris_puzzle0/mavros/local_position/pose", 10, callback);

  rvizPainter = std::make_unique<RvizPainter>(nh);

  ros::Rate loop_rate(500);
  while (ros::ok()) {
    ros::spin();
    loop_rate.sleep();
  }
}