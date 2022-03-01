#include "GUIShootPlugin.hpp"
#include <gazebo/msgs/msgs.hh>

GZ_REGISTER_GUI_PLUGIN(GUIShootPlugin);

GUIShootPlugin::GUIShootPlugin() : GUIPlugin() {

  _node = transport::NodePtr(new transport::Node());
  _node->Init();

  _shootPub = _node->Advertise<msgs::Empty>("~/gun_shoot");

  setStyleSheet("QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");
  QHBoxLayout *mainLayout = new QHBoxLayout;

  QFrame *mainFrame = new QFrame();

  QVBoxLayout *frameLayout = new QVBoxLayout();

  QPushButton *button = new QPushButton(tr("Shoot"));
  connect(button, SIGNAL(clicked()), this, SLOT(onClickButton()));

  frameLayout->addWidget(button);
  mainFrame->setLayout(frameLayout);
  mainLayout->addWidget(mainFrame);

  frameLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  setLayout(mainLayout);

  move(10, 10);
  resize(120, 40);
}

GUIShootPlugin::~GUIShootPlugin() {}

void GUIShootPlugin::onClickButton() {
  msgs::Empty shootMsg;
  shootMsg.set_unused(true);
  _shootPub->Publish(shootMsg);
}
