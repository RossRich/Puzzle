#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <visualization_msgs/Marker.h>

using mavros_msgs::State;
using mavros_msgs::StateConstPtr;
using sensor_msgs::BatteryState;
using sensor_msgs::BatteryStateConstPtr;
using visualization_msgs::Marker;

ros::Publisher statsPub;
geometry_msgs::Vector3 scale;
std_msgs::ColorRGBA color;
geometry_msgs::Pose textPose;

void mavrosStateCallback(const StateConstPtr &state) {
  Marker mode;
  mode.header.frame_id = "base_link";
  mode.header.stamp = ros::Time::now();

  mode.id = 0;
  mode.lifetime = ros::Duration(0);
  mode.type = Marker::TEXT_VIEW_FACING;
  mode.text = state->mode.c_str();
  mode.scale = scale;
  mode.color = color;

  textPose.position.z = 0.15;
  mode.pose = textPose;

  statsPub.publish(mode);
}

void mavrosBatteryCallback(const BatteryStateConstPtr &state) {
  Marker mode;
  mode.header.frame_id = "base_link";
  mode.header.stamp = ros::Time::now();

  mode.id = 1;
  mode.lifetime = ros::Duration(0);
  mode.type = Marker::TEXT_VIEW_FACING;
  mode.text = std::to_string(state->voltage).substr(0, 5) + "v";
  mode.scale = scale;
  mode.color = color;

  textPose.position.z = 0.25;
  mode.pose = textPose;

  statsPub.publish(mode);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "DroneStats");
  ros::NodeHandle nh;

  std::string nodeName = ros::this_node::getName();
  std::string mavrosState;

  if (!nh.getParam(nodeName + "/mavros_state", mavrosState)) {
    ROS_ERROR("No mavros_state param");
    return EXIT_FAILURE;
  }

  std::string mavrosBattery;
  if (!nh.getParam(nodeName + "/mavros_battary", mavrosBattery)) {
    ROS_ERROR("No mavros_battary param");
    return EXIT_FAILURE;
  }

  ros::Subscriber flyMode =
      nh.subscribe<State>(mavrosState, 3, mavrosStateCallback);

  ros::Subscriber batteryState =
      nh.subscribe<BatteryState>(mavrosBattery, 3, mavrosBatteryCallback);

  statsPub = nh.advertise<Marker>("drone_stats", 1, false);

  scale.x = .07;
  scale.y = .07;
  scale.z = .07;

  color.r = 1;
  color.g = 1;
  color.b = 1;
  color.a = 1;

  textPose.position.x = 0.2;
  textPose.position.y = 0.25;
  textPose.position.z = 0.1;

  ros::Rate loop_rate(1);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}