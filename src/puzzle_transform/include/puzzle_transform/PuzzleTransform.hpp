#if !defined(_PUZZLE_TRANSFORM_H_)
#define _PUZZLE_TRANSFORM_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_broadcaster.h>

using geometry_msgs::PoseStamped;
using geometry_msgs::PoseStampedConstPtr;
using geometry_msgs::TransformStamped;

class PuzzleTransform {
private:
  std::string _vehicleName;
  std::string _mavrosLocalPositionTopic;

  ros::NodeHandle &_nh;
  ros::Subscriber _localPositionSubs;

  TransformStamped _transformMsg;
  tf2_ros::TransformBroadcaster _transformBroadcaster;

  void getLocalPosition(const PoseStampedConstPtr &localPosition);
  void connect();
  bool loadParam();
  void transform(const PoseStampedConstPtr &ps);

public:
  PuzzleTransform(ros::NodeHandle &nh);
  ~PuzzleTransform() {}
};

#endif // _PUZZLE_TRANSFORM_H_
