#ifndef VISUAL_MSG_TEST_PENGUIN_H
#define VISUAL_MSG_TEST_PENGUIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
    class Boxscale : public ModelPlugin
    {
        public: void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);
        public: void OnUpdate();
        public: gazebo::msgs::Visual SetScale(ignition::math::Vector3d scale_);
        private:
            gazebo::rendering::ScenePtr scene_ptr ;
            gazebo::rendering::VisualPtr box_ptr ;
            gazebo::physics::ModelPtr model_;
            gazebo::physics::WorldPtr world_;
            gazebo::event::ConnectionPtr updateConnection;
            gazebo::transport::NodePtr node_;
            gazebo::transport::PublisherPtr pub_visual_;
            std::string visual_name_;
            double accum = 0;
            double speed = 0.001;
    };
}

#endif // VISUAL_MSG_TEST_PENGUIN_H