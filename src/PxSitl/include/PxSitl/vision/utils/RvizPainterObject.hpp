#if !defined(_PUZZLE_RVIZ_PAINTER_OBJECT_H_)
#define _PUZZLE_RVIZ_PAINTER_OBJECT_H_

#include <puzzle_common/RvizPainterObjects.hpp>

class RvizPainterObject {
private:
  RvizLine _shortAlfaBlueLine = {"short_ablue_line", "map"};
  RvizArrow _redArrow = {"red_arrow", "map"};
  RvizArrow _yellowArrow = {"yellow_arrow", "map"};
  RvizArrow _vecArrowOfReal = {"vec_of_real_arrow", "map"};
  RvizArrow _vecArrowOfPred = {"vec_of_pred_arrow", "map"};
  RvizPoints _predTraj = {"pred_traj", "map"};
  RvizPosition _objFirstPosition = {"obj_first_position", "map"};
  RvizPosition _pointOnTraj = {"point_on_traj", "map"};
  RvizPosition _intersectPosition = {"intersect_position", "map"};
  RvizPoints _realTrajLine = {"real_traj", "map"};
  RvizPoints _trajectoryPlane = {"trajectory_plane", "map"};
  RvizPosition _midPoint = {"mid_point", "map"};
  RvizLine _redLine = {"red_line", "map"};
  RvizLine _yellowLine = {"yellow_line", "map"};

public:
  RvizPainterObject() {
    _redArrow.setColor(RvizVisually::Colors().at(RvizVisually::Color::Red));
    _yellowArrow.setColor(RvizVisually::Colors().at(RvizVisually::Color::Yellow));
    _vecArrowOfReal.setScale(.15f, .01f, .01f);
    _vecArrowOfPred.setColor(RvizVisually::Colors().at(RvizVisually::Color::CyanProcess));
    _vecArrowOfPred.setScale(.15f, .01f, .01f);
    _predTraj.setColor(RvizVisually::Colors().at(RvizVisually::Color::CyanProcess));
    _objFirstPosition.setColor(RvizVisually::Colors().at(RvizVisually::Color::Marigold));
    _shortAlfaBlueLine.setColor(RvizVisually::Colors().at(RvizVisually::Color::AlfaBlue));
    _shortAlfaBlueLine.setScale(0.02f);
    _pointOnTraj.setColor(RvizVisually::Colors().at(RvizVisually::Color::SonicSilver));
    _pointOnTraj.setScale(.05f);
    _midPoint.setScale(.05f);
    _midPoint.setColor(RvizVisually::Colors().at(RvizVisually::Color::CaribbeanGreen));
    _redLine.setColor(RvizVisually::Colors().at(RvizVisually::Color::Red));
    _yellowLine.setColor(RvizVisually::Colors().at(RvizVisually::Color::Yellow));
    _intersectPosition.setColor(RvizVisually::Colors().at(RvizVisually::Color::Red));
    _trajectoryPlane.setColor(RvizVisually::Colors().at(RvizVisually::Color::AlfaBlue));
    _trajectoryPlane.setScale(0.03);
  }

  ~RvizPainterObject() {}

  RvizArrow &getRegArrow() {
    return _redArrow;
  }

  RvizArrow &getYellowArrow() {
    return _yellowArrow;
  }

  RvizArrow &getVecArrowOfReal() {
    return _vecArrowOfReal;
  }

  RvizArrow &getVecArrowOfPred() {
    return _vecArrowOfPred;
  }

  RvizPoints &getRealTrajLine() {
    return _realTrajLine;
  }

  RvizPoints &getPredTrajLine() {
    return _predTraj;
  }

  RvizPosition &getObjFirstPosition() {
    return _objFirstPosition;
  }

  RvizLine &getShortABlueLine() {
    return _shortAlfaBlueLine;
  }

  RvizPosition &getPointOnTraj() {
    return _pointOnTraj;
  }

  RvizPosition &getIntersectPosition() {
    return _intersectPosition;
  }

  RvizPosition &getMidPoint() {
    return _midPoint;
  }

  RvizLine &getRedLine() {
    return _redLine;
  }

  RvizLine &getYellowLine() {
    return _yellowLine;
  }

  RvizPoints &getTrajectoryPlane() {
    return _trajectoryPlane;
  }
};

#endif // _PUZZLE_RVIZ_PAINTER_OBJECT_H_
