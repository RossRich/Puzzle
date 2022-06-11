#include "puzzle_drone_marker/RvizPainterObjects.hpp"
#include <puzzle_common/RvizPainter.hpp>
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "drone_marker");
  ros::NodeHandle nh;

  RvizPainterObjects painterObjects;
  RvizPainter rvizPainter = {nh, "puzzle_drone_marker"};

  ros::Rate loop_rate(20);
  while (ros::ok()) {
    rvizPainter.update(painterObjects.getDroneMarker());
    loop_rate.sleep();
  }
}