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
  LinkPtr _baseLink;
  LinkPtr _boxLink;
  LinkPtr _rotLink;
  JointPtr _rotJoint;

  double _pTerm, _iTerm, _dTerm = 0.0;
  PID _rotPID;

  event::ConnectionPtr _updateWorld;

  common::Time loopTimer;
  common::Time lastFrame;
  common::Time t;

public:
  GunPlugin() : ModelPlugin(), _node(new transport::Node()) {}
  ~GunPlugin() {}

  void printPose() {
    gzmsg << "---\n";
    gzmsg << "ModelWorld " << _thisModel->WorldPose() << " Relative: " << _thisModel->RelativePose() << endl;

    gzmsg << "BaseWorld: " << _baseLink->WorldPose() << " Relative: " << _baseLink->RelativePose() << endl;

    gzmsg << "BoxWorld: " << _boxLink->WorldPose() << " Relative: " << _boxLink->RelativePose() << endl;
  }

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) {

    _thisWorld = model->GetWorld();
    _thisModel = model;

    _baseLink = model->GetLink("link");
    // _boxVisual = _baseLink->Get
    _boxLink = model->GetLink("link_1");
    _rotJoint = model->GetJoint("link_0_JOINT_2");

    if (!_rotJoint) {
      gzerr << "Joint not found\n";
      return;
    }

    if (!sdf->Get<double>("p_term", _pTerm, 1.))
      gzwarn << "No p_term of PID coefficient. Use default 1.0\n";

    if (!sdf->Get<double>("i_term", _iTerm, 0.))
      gzwarn << "No i_term of PID coefficient. Use default 0.0\n";

    if (!sdf->Get<double>("d_term", _dTerm, 2.))
      gzwarn << "No d_term of PID coefficient. Use default 2.0\n";

    _rotPID = PID(_pTerm, _iTerm, _dTerm, 2 * 3.14, -2 * 3.14);
    model->GetJointController()->SetPositionPID(_rotJoint->GetScopedName(), _rotPID);

    /* Pose3d originalBasePose = _baseLink->WorldPose();
    Pose3d originalBoxPose = _boxVisual->WorldPose(); */
    Pose3d originalModelPose = _thisModel->WorldPose();

    _node->Init(_thisWorld->Name());
    _factoryPub = _node->Advertise<msgs::Factory>("~/factory", 10, 1);
    _visualPub = _node->Advertise<msgs::Visual>("~/visual");

    _selectObject = _node->Subscribe("~/selection", &GunPlugin::onSelectObjectCallback, this);

    _updateWorld =
        event::Events::ConnectWorldUpdateBegin(std::bind(&GunPlugin::onWorldUpdate, this, std::placeholders::_1));

    loopTimer = t = _thisWorld->RealTime();
  }

  void onWorldUpdate(const common::UpdateInfo &worldInfo) {
    common::Time dT = _thisWorld->SimTime() - lastFrame;
    if (worldInfo.realTime >= loopTimer) {

      if (_target) {
        Quaterniond modelRot = _thisModel->WorldPose().Rot();
        Quaterniond targetRot = _target->WorldPose().Rot();
        Vector3d targetPos = _target->WorldPose().Pos();
        Vector3d modelPos = _thisModel->WorldPose().Pos();

        Vector3d targetPosXY(targetPos.X(), targetPos.Y(), 0.f);

        Vector3d rotPos = _boxLink->RelativePose().Pos();
        Vector3d rotXY(rotPos.X(), rotPos.Y(), 0.f);

        gazebo::physics::Joint_V js = _boxLink->GetParentJoints();
        Pose3d jointPose = js.at(0)->InitialAnchorPose();

        // gzmsg << jointPose.Pos() + rotXY << endl;

        Quaterniond newRot = _boxLink->RelativePose().Rot();
        Quaterniond r = Utils::lookAt(rotXY, targetPosXY) * dT.Double() * _maxSpeed;

        Pose3d p(rotXY, newRot + r);

        Vector3d newDir = targetPosXY - rotXY;
        double len = newDir.Length();
        double dot = Vector3d::UnitX.Dot(newDir.Normalized());

        /*  if (abs(dot - (-1.0f)) < 0.000001f)
           dot = 3.14;

         if (abs(dot - (1.0f)) < 0.000001f)
           dot = 1; */

        double rotAngle = acos(dot);

        if (newDir.X() < 0 && newDir.Y() < 0) {
          rotAngle *= -1;
        } else if (newDir.X() > 0 && newDir.Y() < 0) {
          rotAngle *= -1;
        }

        _thisModel->GetJointController()->SetPositionTarget(_rotJoint->GetScopedName(), rotAngle);
        gzmsg << "Rot: " << rotAngle << " Dot: " << dot << " dir: " << newDir << endl;
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