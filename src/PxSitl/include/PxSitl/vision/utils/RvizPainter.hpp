#if !defined(_PUZLLE_ROS_RVIZ_PAINTER_H_)
#define _PUZLLE_ROS_RVIZ_PAINTER_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

using visualization_msgs::Marker;

class RvizPainter {
private:
  const char *_pubName;
  const char *_frameId;

  ros::NodeHandle &_nh;
  ros::Publisher _markersPublisher;

  Marker marker;

public:
  explicit RvizPainter(ros::NodeHandle &nh, const char* frameId, const char *pubName = "rviz_painter") : _nh(nh), _pubName(pubName) {}
  ~RvizPainter() {
    _markersPublisher.shutdown();
  }

  void setup() {
    if (_markersPublisher.getTopic().empty()) {
      _markersPublisher = _nh.advertise<Marker>(_pubName, 5);
    }
  }

  // draw dir
  // draw pos
  // draw path
};

#endif // _PUZLLE_ROS_RVIZ_PAINTER_H_
