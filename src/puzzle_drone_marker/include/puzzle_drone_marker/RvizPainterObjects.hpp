#if !defined(_PUZZLE_DRONE_MARKER_RVIZ_PAINTER_OBJECTS_H_)
#define _PUZZLE_DRONE_MARKER_RVIZ_PAINTER_OBJECTS_H_

#include <puzzle_common/RvizPainterObjects.hpp>

class RvizPainterObjects
{
private:
  RvizMesh _droneMarker = {"puzzle_drone_marker", "puzzle_drone_marker"};
public:
  RvizPainterObjects() {
    // _droneMarker.setResource("package://puzzle_drone_marker/data/quadrotor.dae");
    _droneMarker.setResource("file:///workspaces/Puzzle/src/puzzle_drone_marker/data/quadrotor.dae");
    tf2::Quaternion orientation;
    orientation.setRPY(3.14,0,1.57);
    _droneMarker.setOrientation(orientation);
  }
  ~RvizPainterObjects() {}

  RvizMesh &getDroneMarker() {
    return _droneMarker;
  }
};

#endif // _PUZZLE_DRONE_MARKER_RVIZ_PAINTER_OBJECTS_H_
