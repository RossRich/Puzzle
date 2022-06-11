#if !defined(_PUZZLE_DRONE_MARKER_RVIZ_PAINTER_OBJECTS_H_)
#define _PUZZLE_DRONE_MARKER_RVIZ_PAINTER_OBJECTS_H_

#include <puzzle_common/RvizPainterObjects.hpp>

class RvizPainterObjects
{
private:
  RvizMesh _droneMArker = {"puzzle_drone_marker", "puzzle_drone_marker"};
public:
  RvizPainterObjects() {
    _droneMArker.setResource("package://puzzle_drone_marker/data/quadrotor.dae");
  }
  ~RvizPainterObjects() {}

  RvizMesh &getDroneMarker() {
    return _droneMArker;
  }
};

#endif // _PUZZLE_DRONE_MARKER_RVIZ_PAINTER_OBJECTS_H_
