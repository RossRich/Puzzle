#if !defined(_GAZEBO_SHOOT_PLUGIN_H_)
#define _GAZEBO_SHOOT_PLUGIN_H_

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>

#ifndef Q_MOC_RUN
#include <gazebo/transport/transport.hh>
#endif

using namespace gazebo;

class GAZEBO_VISIBLE GUIShootPlugin : public GUIPlugin {
  
Q_OBJECT

protected slots: void onClickButton();

public:
  GUIShootPlugin();
  ~GUIShootPlugin();

  transport::PublisherPtr _shootPub;
  transport::NodePtr _node;
};

#endif // _GAZEBO_SHOOT_PLUGIN_H_
