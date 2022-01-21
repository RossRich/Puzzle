#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

using namespace gazebo;
using gazebo::physics::LinkPtr;
using gazebo::physics::ModelPtr;
using ignition::math::Pose3d;
using ignition::math::Vector3d;
using sdf::ElementPtr;
using transport::Node;
using transport::NodePtr;
using transport::Publisher;
using transport::SubscriberPtr;

class BulletPlugin : public ModelPlugin {
private:
  ModelPtr _thisModel;
  ModelPtr _thisWorld;

  NodePtr _node;
  SubscriberPtr _selectedSub;

  void onSelectionCallback(ConstSelectionPtr &object) {}

public:
  BulletPlugin() : ModelPlugin(), _node(new Node()) {}
  ~BulletPlugin() {}

  void Load(ModelPtr model, ElementPtr sdf) {
    _thisModel = model;
    _thisWorld = model->GetWorld();

    _node->Init(model->GetWorld()->Name());
    _selectedSub = _node->Subscribe("~/selection", &BulletPlugin::onSelectionCallback, this);
  }
};