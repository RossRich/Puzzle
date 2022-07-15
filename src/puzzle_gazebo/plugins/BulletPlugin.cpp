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
  bool _isFired = false;
  ModelPtr _thisModel;
  WorldPtr _thisWorld;
  Pose3d _targetPose;
  Vector3d _createdPosition;
  NodePtr _node;
  SubscriberPtr _targetSub;
  event::ConnectionPtr _updateWorld;
  common::Time loopTimer;

  float calcVelocity(const Vector3d &from, const Vector3d &to) {
    Vector3d fromTo = to - from;
    Vector3d fromToXY(fromTo.X(), fromTo.Y(), 0.0f);
    float angle = acosf(fromToXY.Normalized().Dot(fromTo.Normalized()));
    float x = fromTo.Length();
    float y = fromTo.Z();
    float num = (9.8f * x * x);
    float vCos = cosf(angle);
    float vTan = tanf(angle);
    float a = y - vTan * x;
    float b = powf(vCos, 2);

    // Protection from inf value
    if (a == 0 || a > 1.0e-3f || a > -1.0e-3f)
      a = 1.0f;
    
    float den = 2.0f * a * b;
    float v2 = num / den;

    return sqrtf(abs(v2));
  }

  float calcVelocity2(const Vector3d &from, const Vector3d &to) {
    Vector3d fromTo = to - from;
    Vector3d fromToXY(fromTo.X(), fromTo.Y(), 0.0);
    float angle = acosf(fromToXY.Normalized().Dot(fromTo.Normalized()));
    float x = fromToXY.Length();
    float y = fromTo.Z();
    float num = (9.8f * x * x);
    return sqrtf(abs(num));
  }

  void calcLinearVel(const Vector3d &from, const Vector3d &to, Vector3d &linearVel) {
#if defined(CALC_VELOCITY_ORIGINAL)
    float v = calcVelocity(from, to);
#else
    float v = calcVelocity2(from, to);
#endif // CALC_VELOCITY_ORIGINAL
    linearVel = (to - from).Normalized() * v;
  }

  void shot() {
    Vector3d modelPosition = _thisModel->WorldPose().Pos();
    Vector3d targetPosition = _targetPose.Pos();
    Vector3d linearVel;
    calcLinearVel(modelPosition, targetPosition, linearVel);
    _thisModel->SetGravityMode(true);
    _thisModel->SetLinearVel(linearVel);
    _isFired = true;
  }

  void onTargetCallback(ConstPosePtr &targetPose) {
    if (_isFired)
      return;

    _targetPose = msgs::ConvertIgn(*targetPose.get());
    if (!_msgIsAvailable)
      _msgIsAvailable = !_msgIsAvailable;
  }

  void onWorldUpdate(const common::UpdateInfo &worldInfo) {
    if (worldInfo.realTime >= loopTimer && !_isFired) {

      if (_msgIsAvailable)
        shot();

      loopTimer += common::Time(0, common::Time::SecToNano(0.033));
    }
  }

public:
  BulletPlugin() : ModelPlugin(), _node(new Node()) {}
  ~BulletPlugin() {}

  void Load(ModelPtr model, ElementPtr sdf) {
    _thisModel = model;
    _createdPosition = model->WorldPose().Pos();
    _thisWorld = model->GetWorld();
    _thisModel->SetGravityMode(false);
    _node->Init(model->GetWorld()->Name());
    _targetSub = _node->Subscribe("~/target", &BulletPlugin::onTargetCallback, this);
    _updateWorld = event::Events::ConnectWorldUpdateBegin(std::bind(&BulletPlugin::onWorldUpdate, this, std::placeholders::_1));

    loopTimer = _thisWorld->RealTime();
  }
};

GZ_REGISTER_MODEL_PLUGIN(BulletPlugin);