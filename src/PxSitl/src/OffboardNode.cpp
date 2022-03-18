#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

using mavros_msgs::CommandBool;
using mavros_msgs::SetMode;
using mavros_msgs::State;
using visualization_msgs::Marker;

State currentState;
geometry_msgs::PoseStamped currentPos;
ros::Publisher droneMarkerPublisher;

void stateCb(const State::ConstPtr &msg) {
  currentState = *msg;
  ROS_INFO("Current state: %s", currentState.mode.c_str());
  ROS_INFO("Is arm: %i", currentState.armed);
}

void poseCb(const geometry_msgs::PoseStampedConstPtr &msg) {
  currentPos = *msg;
}

void drawDronePose(geometry_msgs::TransformStamped &ts) {
  Marker drone;
  drone.action = Marker::ADD;
  drone.ns = "drone";
  drone.type = Marker::MESH_RESOURCE;
  drone.mesh_resource = "package://PxSitl/data/quadrotor.dae";
  drone.header.frame_id = "map";
  drone.header.stamp = ros::Time::now();

  geometry_msgs::Vector3 scale;
  scale.x = 1;
  scale.y = 1;
  scale.z = 1;
  drone.scale = scale;
  drone.pose.orientation = ts.transform.rotation;
  drone.pose.position.x = ts.transform.translation.x;
  drone.pose.position.y = ts.transform.translation.y;
  drone.pose.position.z = ts.transform.translation.z;

  drone.color.a = 1.0;
  drone.color.r = 0;
  drone.color.g = 0;
  drone.color.b = 0;

  droneMarkerPublisher.publish(drone);
}

int main(int argc, char *argv[]) {
  std::string vehicleName = "";

  ros::init(argc, argv, "offboard");
  ros::NodeHandle nh;

  std::string nodeName = ros::this_node::getName();
  ROS_INFO("Node name: %s", nodeName.c_str());

  if (!nh.hasParam(nodeName + "/vehicle_name")) {
    ROS_ERROR("No uav name");
    return -1;
  }

  nh.getParam(nodeName + "/vehicle_name", vehicleName);
  ROS_INFO("New vehicle %s", vehicleName.c_str());

  ros::Subscriber stateSub =
      nh.subscribe<State>(vehicleName + "/mavros/state", 10, stateCb);
  ros::Publisher localPosPub = nh.advertise<geometry_msgs::PoseStamped>(
      vehicleName + "/mavros/setpoint_position/local", 10);
  ros::ServiceClient armingCli =
      nh.serviceClient<CommandBool>(vehicleName + "/mavros/cmd/arming");
  ros::ServiceClient setModeCli =
      nh.serviceClient<SetMode>(vehicleName + "/mavros/set_mode");
  ros::Subscriber pos = nh.subscribe<geometry_msgs::PoseStamped>(
      vehicleName + "/mavros/local_position/pose", 10, poseCb);
  droneMarkerPublisher = nh.advertise<Marker>("/drone_marker", 5);

  ros::Rate rate(20.0);

  while (ros::ok() && !currentState.connected) {
    ros::spinOnce();
    rate.sleep();
  }

  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = .8;

  tf2::Quaternion q;
  q.setRPY(0, 0, 3.14);

  tf2::convert(q, pose.pose.orientation);

  for (int i = 100; ros::ok() && i > 0; --i)
    localPosPub.publish(pose);

  CommandBool armCmd;
  armCmd.request.value = true;

  SetMode offboardModeCmd;
  offboardModeCmd.request.custom_mode = "OFFBOARD";
  offboardModeCmd.request.base_mode = 216;

  ros::Time lastRequest = ros::Time::now();

  tf2_ros::TransformBroadcaster tfBroadcaster;
  geometry_msgs::TransformStamped tfTransformStamped;

  while (ros::ok()) {
    if (currentState.mode != "OFFBOARD" &&
        (ros::Time::now() - lastRequest > ros::Duration(5.0))) {
      if (setModeCli.call(offboardModeCmd) &&
          offboardModeCmd.response.mode_sent) {
        ROS_INFO("Offboard enable");
      }
      lastRequest = ros::Time::now();
    } else {

      if (!currentState.armed &&
          (ros::Time::now() - lastRequest > ros::Duration(5.0))) {
        bool armCliRes = armingCli.call(armCmd);
        bool armCliResponse = armCmd.response.success;

        ROS_INFO("ArmCliRes: %d", armCliRes);
        ROS_INFO("ArmCliResponse: %d", armCliResponse);

        if (armCliRes && armCliResponse) {
          ROS_INFO("Vihicle armed");
        }

        lastRequest = ros::Time::now();
      }
    }

    if (currentState.mode == "OFFBOARD" && currentState.armed == 1) {
      tf2::Quaternion q;
      tf2::fromMsg(currentPos.pose.orientation, q);
      tf2::Vector3 v;
      tf2::fromMsg(currentPos.pose.position, v);
      tf2::Stamped<tf2::Transform> ts(tf2::Transform(tf2::Quaternion(q), tf2::Vector3(v)), ros::Time::now(), "map");

      tfTransformStamped = tf2::toMsg(ts);
      tfTransformStamped.child_frame_id = "base_link";

      tfBroadcaster.sendTransform(tfTransformStamped);
      //   drawDronePose(tfTransformStamped);
    }

    localPosPub.publish(pose);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
