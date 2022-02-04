#include "Utils.hpp"
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <map>

using namespace gazebo;
using gazebo::common::PID;
using gazebo::physics::JointPtr;
using gazebo::physics::LinkPtr;
using gazebo::physics::ModelPtr;
using ignition::math::Pose3d;
using ignition::math::Quaterniond;
using ignition::math::Vector3d;
using std::cout;
using std::endl;

class GunPlugin : public ModelPlugin {
private:
  double _maxSpeed = 5.0;

  transport::NodePtr _node;
  transport::PublisherPtr _factoryPub;
  transport::PublisherPtr _targetPub;
  transport::PublisherPtr _visualPub;
  transport::SubscriberPtr _selectObject;
  transport::SubscriberPtr _shootSub;

  physics::WorldPtr _thisWorld;

  ModelPtr _thisModel;
  ModelPtr _target;

  LinkPtr _pitchLink;
  LinkPtr _yawLink;

  JointPtr _yawJoint;
  JointPtr _pitchJoint;

  double _pTerm, _iTerm, _dTerm = 0.0;
  PID _yawPID;
  PID _pitchPID;

  event::ConnectionPtr _updateWorld;

  common::Time loopTimer;
  common::Time lastFrame;
  common::Time t;

public:
  GunPlugin() : ModelPlugin(), _node(new transport::Node()) {}
  ~GunPlugin() {}

  void printPose() {
    gzmsg << "---\n";
    gzmsg << "ModelWorld " << _thisModel->WorldPose()
          << " Relative: " << _thisModel->RelativePose() << endl;
  }

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) {

    _thisWorld = model->GetWorld();
    _thisModel = model;

    _pitchLink = model->GetLink("pitch_1");
    _yawLink = model->GetLink("yaw_1");

    _yawJoint = model->GetJoint("base_yaw_1");
    _pitchJoint = model->GetJoint("yaw_1_pitch_1");

    if (!_yawJoint || !_pitchJoint) {
      gzerr << "Invalid joints\n";
      return;
    }

    if (!sdf->Get<double>("p_term", _pTerm, 1.))
      gzwarn << "No p_term of PID coefficient. Use default 1.0\n";

    if (!sdf->Get<double>("i_term", _iTerm, 0.))
      gzwarn << "No i_term of PID coefficient. Use default 0.0\n";

    if (!sdf->Get<double>("d_term", _dTerm, 2.))
      gzwarn << "No d_term of PID coefficient. Use default 2.0\n";

    _yawPID = PID(_pTerm, _iTerm, _dTerm, 2 * 3.14, -2 * 3.14);
    model->GetJointController()->SetPositionPID(_yawJoint->GetScopedName(),
                                                _yawPID);

    _pitchPID = PID(_pTerm, _iTerm, _dTerm, 2 * 3.14, -2 * 3.14);
    model->GetJointController()->SetPositionPID(_pitchJoint->GetScopedName(),
                                                _pitchPID);

    _node->Init(_thisWorld->Name());
    _factoryPub = _node->Advertise<msgs::Factory>("~/factory", 10, 1);
    _visualPub = _node->Advertise<msgs::Visual>("~/visual");

    _selectObject = _node->Subscribe("~/selection",
                                     &GunPlugin::onSelectObjectCallback, this);

    _updateWorld = event::Events::ConnectWorldUpdateBegin(
        std::bind(&GunPlugin::onWorldUpdate, this, std::placeholders::_1));

    loopTimer = t = _thisWorld->RealTime();
  }

  void onWorldUpdate(const common::UpdateInfo &worldInfo) {
    common::Time dT = _thisWorld->SimTime() - lastFrame;
    if (worldInfo.realTime >= loopTimer) {

      if (_target) {
        Vector3d targetPos = _target->WorldPose().Pos();
        Vector3d targetPosXY(targetPos.X(), targetPos.Y(), 0.f);

        Vector3d yawPose = _yawLink->WorldPose().Pos();
        Vector3d yawXY(yawPose.X(), yawPose.Y(), 0.f);

        Vector3d pitchPose = _pitchLink->WorldPose().Pos();

        Vector3d newDir = targetPosXY - yawXY;
        double len = newDir.Length();
        double dot = Vector3d::UnitX.Dot(newDir.Normalized());

        double rotAngle = acos(dot);

        if (newDir.X() < 0 && newDir.Y() < 0) {
          rotAngle *= -1;
        } else if (newDir.X() > 0 && newDir.Y() < 0) {
          rotAngle *= -1;
        }

        _thisModel->GetJointController()->SetPositionTarget(
            _yawJoint->GetScopedName(), rotAngle);
        gzmsg << "Rot: " << rotAngle << " Dot: " << dot << " dir: " << newDir
              << endl;
      }
      loopTimer += common::Time(0, common::Time::SecToNano(1 / 30));
    }

    lastFrame = _thisWorld->SimTime();
  }

  void onSelectObjectCallback(ConstSelectionPtr &object) {
    if (object->selected() && (object->name() != _thisModel->GetName())) {
      _target = _thisWorld->ModelByName(object->name());
      if (_target)
        gzmsg << _target->GetName() << endl;
    }
  }
};

GZ_REGISTER_MODEL_PLUGIN(GunPlugin);