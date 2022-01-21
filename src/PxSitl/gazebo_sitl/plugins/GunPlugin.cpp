#include "Utils.hpp"
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <map>

using namespace gazebo;
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

  transport::PublisherPtr _factoryPub;
  transport::NodePtr _node;
  std::map<std::string, physics::ModelPtr> _bullets;
  transport::SubscriberPtr _selectObject;
  transport::SubscriberPtr _shootSub;
  transport::PublisherPtr _visualPub;
  physics::WorldPtr _thisWorld;
  physics::ModelPtr _thisModel;
  physics::ModelPtr _target;
  physics::LinkPtr _bulletSpawnLink;

  event::ConnectionPtr _newModelAdded;
  event::ConnectionPtr _updateWorld;

  common::Time loopTimer;
  common::Time lastFrame;

public:
  GunPlugin() : ModelPlugin(), _node(new transport::Node()) {}
  ~GunPlugin() {}

  bool createBullet(std::string &bulletSrt) {
    sdf::SDFPtr bulletSdf(new sdf::SDF());
    sdf::init(bulletSdf);

    if (!sdf::readFile(bulletFilePath, bulletSdf)) {
      gzerr << "Unable to read sdf file [" << bulletFilePath << "]\n";
      return false;
    }

    sdf::ElementPtr bulletModel = bulletSdf->Root()->GetElement("model");
    sdf::ElementPtr bulletVisual =
        bulletModel->GetElement("link")->GetElement("visual");

    if (!bulletModel || !bulletModel) {
      gzerr << "Invalid of bullet model\n";
      return false;
    }

    // bulletVisual->GetElement("transparency")->Set<double>(1.0);
    // bulletModel->GetElement("static")->Set<int>(1);
    bulletModel->GetAttribute("name")->Set<std::string>(
        bulletName + std::to_string(++_bulletNum));

    bulletSrt = bulletSdf->ToString();

    return true;
  }

  void spawnBullet() {
    msgs::Factory factoryMsg;

    std::string bulletStr;

    if (!createBullet(bulletStr)) {
      gzerr << "Failed to create sdf model of bullet\n";
      return;
    }

    factoryMsg.set_sdf(bulletStr);
    msgs::Set(factoryMsg.mutable_pose(), _bulletSpawnLink->WorldPose());
    _factoryPub->Publish(factoryMsg);
  }

  void shootCallback(ConstEmptyPtr &msg) {

    spawnBullet();

    /* physics::ModelPtr b = _thisWorld->ModelByName("bullet_0");

    if (_bullets.empty()) {
      gzerr << "Gun is empty\n";
      return;
    }

    physics::LinkPtr l = b->GetLink("main");
    msgs::Visual v = l->GetVisualMessage("visual");

    v.set_name("bullet_0::main");
    v.set_parent_name("bullet_0"); */

    // b->SetStatic(false);
    // v.set_transparency(0.5);
    /* _visualPub->Publish(v);

    Vector3d dir = _target->WorldPose().Pos() -
    _bulletSpawnLink->WorldPose().Pos(); l->AddForce(dir.Normalized() * 50);

    _target->GetChildLink("link")->AddForce(Vector3d::UnitX * 50); */

    /* sdf::ElementPtr sdfModel;
    for (auto &&bullet : _bullets) {
      sdfModel = bullet.second->GetSDF();
      if (sdfModel) {
        sdfModel->GetElement("static")->Set<int>(0);
      }
    } */
  }

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) {

    _thisWorld = model->GetWorld();
    _thisModel = model;
    _bulletSpawnLink = model->GetLink("bullet_spawner");

    if (!_bulletSpawnLink) {
      gzerr << "Bullet spawner now find\n";
      return;
    }

    if (!sdf->HasElement("model")) {
      gzerr << "Model of bullet not defined\n";
      return;
    }

    std::pair modelName =
        sdf->GetElement("model")->Get<std::string>("name", "");

    if (modelName.first == "" || !modelName.second) {
      gzerr << "Invalid model name for spawn in plugin param\n";
      return;
    }

    _node->Init("puzzle_world");
    _factoryPub = _node->Advertise<msgs::Factory>("~/factory", 10, 1);
    _selectObject = _node->Subscribe("~/selection",
                                     &GunPlugin::onSelectObjectCallback, this);
    _shootSub =
        _node->Subscribe("~/gun_shoot", &GunPlugin::shootCallback, this);
    _visualPub = _node->Advertise<msgs::Visual>("~/visual");

    _newModelAdded = event::Events::ConnectAddEntity(
        std::bind(&GunPlugin::onNewModel, this, std::placeholders::_1));
    _updateWorld = event::Events::ConnectWorldUpdateBegin(
        std::bind(&GunPlugin::onWorldUpdate, this, std::placeholders::_1));

    bulletFilePath = common::ModelDatabase::Instance()->GetModelFile(
        modelName.first.insert(0, "model://"));
    loopTimer = _thisWorld->RealTime();
  }

  void onNewModel(std::string name) {
    gzmsg << "New model: " << name << endl;

    if (name == "bullet_1") {
      gzmsg << "bullet_1 is found\n";

      /* ModelPtr bm = _thisWorld->ModelByName(name);
      if(bm) {
        gzmsg << bm->GetSDF()->ToString("");
      } */
    }
  }

  void onWorldUpdate(const common::UpdateInfo &worldInfo) {
    common::Time dT = _thisWorld->SimTime() - lastFrame;
    if (worldInfo.realTime >= loopTimer) {

      if (_target) {
        Quaterniond modelRot = _thisModel->WorldPose().Rot();
        Quaterniond targetRot = _target->WorldPose().Rot();
        Vector3d targetPos = _target->WorldPose().Pos();
        Vector3d modelPos = _thisModel->WorldPose().Pos();
        Pose3d p(_thisModel->WorldPose().Pos(),
                 Utils::lookAt(modelPos, targetPos));
        _thisModel->SetWorldPose(p);
      }

      loopTimer += common::Time(0, common::Time::SecToNano(1 / 30));
    }

    lastFrame = _thisWorld->SimTime();
  }

  void onSelectObjectCallback(ConstSelectionPtr &object) {
    if (object->selected()) {
      _target = _thisWorld->ModelByName(object->name());
      if (_target)
        gzmsg << _target->GetName() << endl;
    }
  }
};

GZ_REGISTER_MODEL_PLUGIN(GunPlugin);