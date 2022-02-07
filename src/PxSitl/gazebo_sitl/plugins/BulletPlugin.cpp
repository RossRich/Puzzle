#include "Utils.hpp"
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

using namespace gazebo;
using gazebo::physics::LinkPtr;
using gazebo::physics::ModelPtr;
using gazebo::physics::WorldPtr;
using ignition::math::Pose3d;
using ignition::math::Quaterniond;
using ignition::math::Vector3d;
using sdf::ElementPtr;
using transport::Node;
using transport::NodePtr;
using transport::Publisher;
using transport::SubscriberPtr;

class BulletPlugin : public ModelPlugin {
private:
  bool _msgIsAvailable = false;
  bool _isGrounded = true;

  ModelPtr _thisModel;
  WorldPtr _thisWorld;

  Pose3d _targetPose;

  NodePtr _node;
  SubscriberPtr _targetSub;

  event::ConnectionPtr _updateWorld;

  common::Time loopTimer;
  common::Time lastFrame;

  void onTargetCallback(ConstPosePtr &targetPose) {
    _targetPose = msgs::ConvertIgn(*targetPose.get());
    if (!_msgIsAvailable)
      _msgIsAvailable = !_msgIsAvailable;
  }

  void onWorldUpdate(const common::UpdateInfo &worldInfo) {
    common::Time dT = _thisWorld->SimTime() - lastFrame;

    if (worldInfo.realTime >= loopTimer) {

      if (_msgIsAvailable && _isGrounded) {
        Quaterniond modelRot = _thisModel->WorldPose().Rot();
        Quaterniond targetRot = _targetPose.Rot();
        Vector3d targetPos = _targetPose.Pos();
        Vector3d modelPos = _thisModel->WorldPose().Pos();

        Vector3d antiZ = Vector3d::UnitX;
        double angle = acos(antiZ.Dot((targetPos - modelPos).Normalized()));
        gzmsg << "angle: " << angle << std::endl;

        Pose3d p(_thisModel->WorldPose().Pos(), Utils::lookAt(modelPos, targetPos));
        _thisModel->SetWorldPose(p);

        gzmsg << "target pose: " << _targetPose << " model pose: " << modelPos
              << std::endl;

        Vector3d fromTo = targetPos - modelPos;
        Vector3d fromToXZ = Vector3d(fromTo.X(), fromTo.Y(), fromTo.Z());
        // gzmsg << "From To: " << fromTo << " From to XZ: " << fromToXZ <<
        // std::endl;

        float x = fromToXZ.Length();
        float y = fromTo.Z();

        float v2 = (9.8 * x * x) / (2 * (y - tan(angle) * x) * pow(cos(angle), 2));
        float v = sqrt(abs(v2));

        _thisModel->SetGravityMode(true);
        _thisModel->SetLinearVel(fromTo.Normalized() * v);
        _isGrounded = false;
      }

      loopTimer += common::Time(0, common::Time::SecToNano(1 / 30));
    }

    lastFrame = _thisWorld->SimTime();
  }

public:
  BulletPlugin() : ModelPlugin(), _node(new Node()) {}
  ~BulletPlugin() {}

  void Load(ModelPtr model, ElementPtr sdf) {
    _thisModel = model;
    _thisWorld = model->GetWorld();
    _thisModel->SetGravityMode(false);

    _node->Init(model->GetWorld()->Name());
    _targetSub =
        _node->Subscribe("~/target", &BulletPlugin::onTargetCallback, this);
    _updateWorld = event::Events::ConnectWorldUpdateBegin(
        std::bind(&BulletPlugin::onWorldUpdate, this, std::placeholders::_1));

    loopTimer = _thisWorld->RealTime();
  }
};

GZ_REGISTER_MODEL_PLUGIN(BulletPlugin);