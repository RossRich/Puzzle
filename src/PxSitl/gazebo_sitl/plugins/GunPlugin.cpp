#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/transport/transport.hh>

using namespace gazebo;
using std::cout;
using std::endl;

class GunPlugin : public ModelPlugin {
private:
  // std::vector bullets();
  transport::PublicationPtr factoryPub;
  transport::NodePtr node;

public:
  GunPlugin() : ModelPlugin(), node(new transport::Node()) {}
  ~GunPlugin() {}

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
    node->Init(model->GetName());

    factoryPub = node->Advertise<msgs::Factory>("~/spawn_bullet");

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

    oliveVisual->GetElement("transparency")->Set<int>(0);
    oliveModel->GetElement("static")->Set<int>(1);

    world->InsertModelSDF(*sdfOlive.get());
  }
};

GZ_REGISTER_MODEL_PLUGIN(GunPlugin);