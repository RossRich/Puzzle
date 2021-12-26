#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

using namespace gazebo;
using std::cout;
using std::endl;

class GunPlugin : public ModelPlugin {
private:
  transport::PublisherPtr factoryPub;
  transport::NodePtr node;
  physics::Model_V bullets;
  transport::SubscriberPtr factorySub;
  physics::WorldPtr _thisWorld;
  event::ConnectionPtr _newModelAdded;

public:
  GunPlugin() : ModelPlugin(), node(new transport::Node()) {}
  ~GunPlugin() {}

  void factoryCallback(ConstFactoryPtr &msg) {
    // gzmsg << "Total models: " << _thisWorld->ModelCount() << endl;
  }

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) {

    _thisWorld = model->GetWorld();

    node->Init("puzzle_world");

    factoryPub = node->Advertise<msgs::Factory>("~/factory");
    factorySub = node->Subscribe("~/factory", &GunPlugin::factoryCallback, this);

    _newModelAdded = event::Events::ConnectAddEntity(std::bind(&GunPlugin::onNewModel, this));

    auto bulletSpavner = model->GetLink("bullet_spawner");

    if (!bulletSpavner) {
      gzerr << "Bullet spawner now find\n";
      return;
    }

    if (!sdf->HasElement("model")) {
      gzerr << "Model of bullet not defined\n";
      return;
    }

    auto modelName = sdf->GetElement("model")->Get<std::string>("name", "");

    if (modelName.first == "" || !modelName.second) {
      gzerr << "Invalid model name for spawn in plugin param\n";
      return;
    }

    modelName.first.insert(0, "model://");

    auto bSpawnerPose = bulletSpavner->WorldPose();

    physics::WorldPtr world = model->GetWorld();
    std::string filename = common::ModelDatabase::Instance()->GetModelFile(modelName.first);
    gzmsg << filename << endl;

    sdf::SDFPtr sdfOlive(new sdf::SDF());
    sdf::init(sdfOlive);

    if (!sdf::readFile(filename, sdfOlive)) {
      gzerr << "Unable to read sdf file [" << filename << "]\n";
      return;
    }

    sdf::ElementPtr oliveModel = sdfOlive->Root()->GetElement("model");
    sdf::ElementPtr oliveVisual = oliveModel->GetElement("link")->GetElement("visual");

    if (!oliveModel || !oliveModel) {
      gzerr << "Invalid of bullet model\n";
      return;
    }

    oliveVisual->GetElement("transparency")->Set<int>(1);
    oliveModel->GetElement("static")->Set<int>(1);

    msgs::Factory factoryMsg;
    std::string bulletName = "bullet_";

    for (int i = 0; i < 5; i++) {
      oliveModel->GetAttribute("name")->Set<std::string>(bulletName + std::to_string(i));
      bSpawnerPose.Pos() = bSpawnerPose.CoordPositionAdd(ignition::math::Vector3d(0, 0, 0.12));
      factoryMsg.set_sdf(sdfOlive->ToString());
      msgs::Set(factoryMsg.mutable_pose(), bSpawnerPose);
      factoryPub->Publish(factoryMsg);
    }

    gzmsg << "Total models: " << world->ModelCount() << endl;

    for (auto &&model : world->Models()) {
      if (model->GetName().find(bulletName.c_str()) != std::string::npos) {
        bullets.push_back(model);
      }
    }

    gzmsg << "Total bullets: " << bullets.size() << endl;

    for (auto &&bullet : bullets) {
      sdf::ElementPtr visual = bullet->GetSDF()->FindElement("visual");
      gzmsg << visual->ToString("");
      break;
    }
  }

  void onNewModel() {
    
  }
};

GZ_REGISTER_MODEL_PLUGIN(GunPlugin);