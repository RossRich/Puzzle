#include "Utils.hpp"
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/transport/Node.hh>
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
  float rotationF = 0.1;
  uint8_t _bulletNum = 0;
  std::string bulletName = "bullet_";
  std::string bulletFilePath;

  ignition::transport::Node _ignNode;

  transport::NodePtr _node;
  transport::PublisherPtr _factoryPub;
  transport::PublisherPtr _targetPub;
  transport::PublisherPtr _visualPub;
  transport::SubscriberPtr _selectObject;
  transport::SubscriberPtr _shootSub;

  physics::WorldPtr _thisWorld;
  ModelPtr _thisModel;
  ModelPtr _target;

  LinkPtr _bulletSpawnLink;
  LinkPtr _yawLink;
  LinkPtr _pitchLink;

  JointPtr _yawJoint;
  JointPtr _pitchJoint;

  double _pYaw, _iYaw, _dYaw = 0.0;
  double _pPitch, _iPitch, _dPitch = 0.0;
  PID _yawPID;
  PID _pitchPID;

  Vector3d _lastDir = Vector3d::UnitX;
  double _lastYawAngle = .0;
  double _lastAngle2 = .0;

  event::ConnectionPtr _updateWorld;

  common::Time loopTimer;
  common::Time lastFrame;

public:
  GunPlugin() : ModelPlugin(), _node(new transport::Node()), _ignNode() {}
  ~GunPlugin() {}

  bool createBullet(std::string &bulletSrt) {
    sdf::SDFPtr bulletSdf(new sdf::SDF());
    sdf::init(bulletSdf);

    if (!sdf::readFile(bulletFilePath, bulletSdf)) {
      gzerr << "Unable to read sdf file [" << bulletFilePath << "]\n";
      return false;
    }

    sdf::ElementPtr bulletModel = bulletSdf->Root()->GetElement("model");
    sdf::ElementPtr bulletVisual = bulletModel->GetElement("link")->GetElement("visual");

    if (!bulletModel || !bulletModel) {
      gzerr << "Invalid of bullet model\n";
      return false;
    }

    // bulletVisual->GetElement("transparency")->Set<double>(1.0);
    // bulletModel->GetElement("static")->Set<int>(1);
    bulletModel->GetAttribute("name")->Set<std::string>(bulletName + std::to_string(++_bulletNum));

    bulletSrt = bulletSdf->ToString();

    return true;
  }

  void spawnBullet() {
    msgs::Factory factoryMsg;

    std::string bulletSdf;
    if (!createBullet(bulletSdf)) {
      gzerr << "Failed to create sdf model of bullet\n";
      return;
    }

    factoryMsg.set_sdf(bulletSdf);
    msgs::Set(factoryMsg.mutable_pose(), _bulletSpawnLink->WorldPose());
    _factoryPub->Publish(factoryMsg);
  }

  void shootCallback(ConstEmptyPtr &msg) { spawnBullet(); }

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
    _thisWorld = model->GetWorld();
    _thisModel = model;

    _bulletSpawnLink = model->GetLink("bullet_spawner");
    _pitchLink = model->GetLink("pitch_1");
    _yawLink = model->GetLink("yaw_1");

    _yawJoint = model->GetJoint("base_yaw_1");
    _pitchJoint = model->GetJoint("yaw_1_pitch_1");

    if (!_yawJoint || !_pitchJoint) {
      gzerr << "Invalid joints\n";
      return;
    }

    if (!_bulletSpawnLink) {
      gzerr << "Bullet spawner now find\n";
      return;
    }

    if (!sdf->HasElement("model")) {
      gzerr << "Model of bullet not defined\n";
      return;
    }

    // gzmsg << "Link rel \n" << "yaw: " << _yawLink->RelativePose().Pos() << " pitch: " <<
    // _pitchLink->RelativePose().Pos() << endl;

    // gzmsg << "Link world \n" << "yaw: " << _yawLink->WorldPose().Pos() << " pitch: " << _pitchLink->WorldPose().Pos()
    // << endl;

    std::pair modelName = sdf->GetElement("model")->Get<std::string>("name", "");

    if (modelName.first == "" || !modelName.second) {
      gzerr << "Invalid model name for spawn in plugin param\n";
      return;
    }

    if (!sdf->Get<double>("p_yaw", _pYaw, 1.))
      gzwarn << "No p_yaw of PID coefficient. Use default 1.0\n";

    if (!sdf->Get<double>("i_yaw", _iYaw, 0.0))
      gzwarn << "No i_yaw of PID coefficient. Use default 0.0\n";

    if (!sdf->Get<double>("d_yaw", _dYaw, 2.0))
      gzwarn << "No d_yaw of PID coefficient. Use default 2.0\n";

    _yawPID = PID(_pYaw, _iYaw, _dYaw, 3.14, -3.14);
    model->GetJointController()->SetPositionPID(_yawJoint->GetScopedName(), _yawPID);

    if (!sdf->Get<double>("p_pitch", _pPitch, 1.))
      gzwarn << "No p_pitch of PID coefficient. Use default 1.0\n";

    if (!sdf->Get<double>("i_pitch", _iPitch, 0.))
      gzwarn << "No i_pitch of PID coefficient. Use default 0.0\n";

    if (!sdf->Get<double>("d_pitch", _dPitch, 2.))
      gzwarn << "No d_pitch of PID coefficient. Use default 2.0\n";
    _pitchPID = PID(_pPitch, _iPitch, _dPitch, 3.14, -3.14);
    model->GetJointController()->SetPositionPID(_pitchJoint->GetScopedName(), _pitchPID);

    _node->Init(_thisWorld->Name());
    _factoryPub = _node->Advertise<msgs::Factory>("~/factory", 10, 1);
    _visualPub = _node->Advertise<msgs::Visual>("~/visual");
    _targetPub = _node->Advertise<msgs::Pose>("~/target", 5, 1);

    _selectObject = _node->Subscribe("~/selection", &GunPlugin::onSelectObjectCallback, this);
    _shootSub = _node->Subscribe("~/gun_shoot", &GunPlugin::shootCallback, this);

    // _newModelAdded =
    // event::Events::ConnectAddEntity(std::bind(&GunPlugin::onNewModel, this,
    // std::placeholders::_1));

    _updateWorld =
        event::Events::ConnectWorldUpdateBegin(std::bind(&GunPlugin::onWorldUpdate, this, std::placeholders::_1));

    bulletFilePath = common::ModelDatabase::Instance()->GetModelFile(modelName.first.insert(0, "model://"));
    loopTimer = _thisWorld->RealTime();
  }

  /*
   * \brief Calculate a value for shortest rotation
   * \param[out] rotAngle
   * \param[in] diraction
   * \param[in] lastDiraction
   * \param[in] nowAtan2
   * \param[in] lastAtan2
   */
  bool rotation(double &rotAngle, double &lastRotAngle, Vector3d &diraction, Vector3d &lastDiraction) {
    Vector3d nDir = diraction.Normalized();
    Vector3d nLastDir = lastDiraction.Normalized();

    double angleForRot = acos(nLastDir.Dot(nDir));
    double atanForRot = -atan2(nDir.Y(), nDir.X());
    
    if (nLastDir == nDir || angleForRot < 0.1)
      return false;

    double OX = nDir.X();
    double OY = nDir.Y();
    double lastOx = nLastDir.X();
    double lastOy = nLastDir.Y();

    bool isThroughZero = false;
    bool isThroughPI = false;

    /*
     * lastOx > 0 && lastOY < 0 && OY > 0 -> go to up through the zero
     * lastOx > 0 && lastOY > 0 && OY < 0 -> go to down through the zero
     *
     * lastOx < 0 && lastOY < 0 && OY > 0 -> go to up through the PI
     * lastOx < 0 && lastOY > 0 && OY < 0 -> go to down through the PI
     */

    if (lastOx >= 0) {

      if (lastOy < 0 && OY > 0) {
        isThroughZero = true;
        isThroughPI = false;
      } else if (lastOy > 0 && OY < 0) {
        isThroughZero = true;
        isThroughPI = false;
      }

    } else {

      if (lastOy < 0 && OY > 0) {
        isThroughPI = true;
        isThroughZero = false;
      } else if (lastOy > 0 && OY < 0) {
        isThroughPI = true;
        isThroughZero = false;
      }

      return true;
    }

    gzmsg << "Go through PI: " << isThroughPI << " Go through zero: " << isThroughZero << endl;

    /*
     * isThroughPI && OY > 0 -> increment a angle
     * isThroughPI && OY < 0 -> decrement a angle
     *
     * isThroughZero && OY > 0 -> increment a angle
     * isThroughZero && OY < 0 -> decrement a angle
     *
     * else standart logic
     */

    if (isThroughPI && OY > 0) {
      // _lastYawAngle += angleForYaw;

    } else if (isThroughPI && OY < 0) {
      // _lastYawAngle -= angleForYaw;
      rotAngle = -rotAngle;
    } else if (isThroughZero && OY > 0) {
      // _lastYawAngle -= angleForYaw;
      rotAngle = -rotAngle;
    } else if (isThroughZero && OY < 0) {
      // _lastYawAngle += angleForYaw;
    } else {
      double rotDir = atanForRot - lastAtan2;

      if (rotDir > 0) {
        // _lastYawAngle += angleForYaw;
      } else {
        // _lastYawAngle -= angleForYaw;
        rotAngle = -rotAngle;
      }
    }
  }

  void onWorldUpdate(const common::UpdateInfo &worldInfo) {
    common::Time dT = _thisWorld->SimTime() - lastFrame;
    if (worldInfo.realTime >= loopTimer) {

      if (_target) {
        Vector3d targetPos = _target->WorldPose().Pos();

        Vector3d pitchPose = _pitchLink->WorldPose().Pos();
        Vector3d dirForPitch = targetPos - pitchPose;
        Vector3d antiZ = Vector3d::UnitZ * -1;
        double angleForPitch = acos(antiZ.Dot(dirForPitch.Normalized()));

        Vector3d offset = _pitchLink->RelativePose().Pos();
        Vector3d yawPose = _yawLink->WorldPose().Pos() + Vector3d(offset.X(), offset.Y(), 0.);
        Vector3d yawXY(yawPose.X(), yawPose.Y(), 0.);
        Vector3d targetPosXY(targetPos.X(), targetPos.Y(), 0.);
        Vector3d dirForYaw = targetPosXY - yawXY;

        if (rotation(angleForYaw, dirForYaw, _lastDir, angleForYaw2, _lastAngle2)) {
          _lastYawAngle += angleForYaw;

          _lastAngle2 = angleForYaw2;
          _lastDir = dirForYaw.Normalized();
          gzmsg << "atan2: " << angleForYaw2;

          _thisModel->GetJointController()->SetPositionTarget(_pitchJoint->GetScopedName(), angleForPitch);

          double rLastYawAngle = round(_lastYawAngle * 100.0) / 100.0;
          _thisModel->GetJointController()->SetPositionTarget(_yawJoint->GetScopedName(), rLastYawAngle);

          gzmsg << "Yaw: " << angleForYaw;   //<< " Pitch: " << angleForPitch;
          gzmsg << " YawRot: " << dirForYaw; //<< " PitchRow: " << dirForPitch;
          gzmsg << " lastYawAngle: " << _lastYawAngle;
          gzmsg << " target pose: " << targetPos;

          gzmsg << endl;

          ignition::msgs::Marker markerMsg;
          markerMsg.set_ns("default");
          markerMsg.set_id(1);
          markerMsg.set_action(ignition::msgs::Marker_Action::Marker_Action_ADD_MODIFY);
          markerMsg.set_type(ignition::msgs::Marker_Type::Marker_Type_SPHERE);
          ignition::msgs::Material *markerMatMsg = markerMsg.mutable_material();
          markerMatMsg->mutable_script()->set_name("Gazebo/BlueLaser");
          ignition::msgs::Set(markerMsg.mutable_pose(), Pose3d(yawPose, Quaterniond::Identity));
          ignition::msgs::Set(markerMsg.mutable_scale(), Vector3d(0.01, 0.01, 0.01));
          _ignNode.Request("/marker", markerMsg);
        }

        msgs::Pose targetPoseMsg;
        msgs::Set(targetPoseMsg.mutable_orientation(), _target->WorldPose().Rot());
        msgs::Set(targetPoseMsg.mutable_position(), targetPos);

        _targetPub->Publish(targetPoseMsg);
      }
      loopTimer += common::Time(0, common::Time::SecToNano(0.25));
    }

    lastFrame = _thisWorld->SimTime();
  }

  void onSelectObjectCallback(ConstSelectionPtr &object) {
    if (object->selected() && (object->name() != _thisModel->GetName()) && (object->name() != "asphalt_plane")) {
      _target = _thisWorld->ModelByName(object->name());
      if (_target)
        gzmsg << _target->GetName() << endl;
    }
  }
};

GZ_REGISTER_MODEL_PLUGIN(GunPlugin);