#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include "ModelScale.hpp"

namespace gazebo
{
    void Boxscale::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr sdf)
    {
        model_ = _parent;
        world_ = model_->GetWorld();

        node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
        node_->Init(world_->Name());
        pub_visual_ = node_->Advertise<gazebo::msgs::Visual>("~/visual");

        this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&Boxscale::OnUpdate, this));
    }

    void Boxscale::OnUpdate()
    {
        ignition::math::Vector3d initial_scale(3.0, 1.0+accum, 1.0);
        pub_visual_->Publish( SetScale(initial_scale) );
        if (accum < 0) {
            accum = 0;
            speed = speed * -1;
        }
        else if (accum > 1) {
            accum = 1;
            speed = speed * -1;
        }
        accum = speed + accum;
    }

    gazebo::msgs::Visual Boxscale::SetScale(ignition::math::Vector3d scale_)
    {
        // std::cerr << "\nSetting scale: " << scale_;

        std::string visual_name_ = "link_visual";
        std::string linkName = "link";
        physics::LinkPtr link_ = this->model_->GetLink(linkName);
        //sdf::ElementPtr visualSDF = linkSDF->GetElement("visual");
        //visual_name_ = visualSDF->Get<std::string>("name");

        gazebo::msgs::Visual visualMsg = link_->GetVisualMessage(visual_name_);
        gazebo::msgs::Vector3d* scale_factor = new gazebo::msgs::Vector3d{gazebo::msgs::Convert(scale_)};

        visualMsg.set_name(link_->GetScopedName());
        gzmsg << "Name: " << link_->GetScopedName() << std::endl;

        visualMsg.set_parent_name(this->model_->GetScopedName());
        gzmsg << "Parent name: " << this->model_->GetScopedName() << std::endl;

        visualMsg.set_allocated_scale(scale_factor);
        visualMsg.set_transparency(1.0);
        link_->AddRelativeForce(ignition::math::Vector3d(50, 0, 50));

        // Initiate material
        if ((!visualMsg.has_material()) || visualMsg.mutable_material() == NULL) {
            gazebo::msgs::Material *materialMsg = new gazebo::msgs::Material;
            visualMsg.set_allocated_material(materialMsg);
        }


        // Set color
        // gazebo::common::Color newColor();
        // gazebo::msgs::Color *colorMsg = new gazebo::msgs::Color(gazebo::msgs::Convert(newColor));
        // gazebo::msgs::Color *diffuseMsg = new gazebo::msgs::Color(*colorMsg);

        gazebo::msgs::Material *materialMsg = visualMsg.mutable_material();
        if (materialMsg->has_ambient())
        {
          materialMsg->clear_ambient();
        }
        // materialMsg->set_allocated_ambient(colorMsg);
        // if (materialMsg->has_diffuse())
        // {
          // materialMsg->clear_diffuse();
        // }
        // materialMsg->set_allocated_diffuse(diffuseMsg);

        return visualMsg;
    }

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(Boxscale)
}