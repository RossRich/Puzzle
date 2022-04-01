// #include "../../include/PxSitl/vision/State.hpp"
// #include "../../include/PxSitl/vision/StateInit.hpp"
// #include "../../include/PxSitl/vision/StateWait.hpp"
#include "../../include/PxSitl/vision/BallTrackingRos.hpp"

BallTrackingRos *BallTrackingRos::getInstanse(ros::NodeHandle &nh) {

  if (_instance != nullptr)
    return _instance;

  int cameraWidth, cameraHeight = 0;
  ros::param::param<int>("/camera/realsense2_camera/color_width", cameraWidth, 0);
  ros::param::param<int>("/camera/realsense2_camera/color_height", cameraHeight, 0);

  if (cameraHeight <= 0 || cameraWidth <= 0) {
    ROS_INFO("No camera resolution in params. Trying to request resolution from CameraInfo...");
    CameraInfoConstPtr cameraInfo;
    try {
      cameraInfo = ros::topic::waitForMessage<CameraInfo>("/camera/color/camera_info", nh);
    } catch (const ros::Exception &e) {
      ROS_ERROR("%s", e.what());
      ROS_ERROR("Failed to get camera info. Exit.");
      return nullptr;
    }

    if (!cameraInfo) {
      ROS_ERROR("Failed to get camera info. Exit.");
      return nullptr;
    }

    cameraWidth = cameraInfo->width;
    cameraHeight = cameraInfo->height;

    if (cameraHeight <= 0 || cameraWidth <= 0) {
      ROS_ERROR("No camera rosolution. Exit.");
      return nullptr;
    }
  }

  ROS_INFO("Camera resolution received. Width = %i, height = %i", cameraWidth, cameraHeight);

  _instance = new BallTrackingRos(nh);
  return _instance;
}

BallTrackingRos::BallTrackingRos(ros::NodeHandle &nh) : _nh(nh) {

  _strategySrv = _nh.advertiseService("strategy_srv", &BallTrackingRos::runSetupSrv, this);
  ROS_INFO("Change strategy server ready");

  _stateInit = new StateInit();
  _stateWait = new StateWait();

  setState(_stateWait);
}

BallTrackingRos::~BallTrackingRos() {
  delete _stateInit;
  delete _stateWait;
}

bool BallTrackingRos::runSetupSrv(std_srvs::EmptyRequest &request, std_srvs::EmptyResponse &response) {
  return true;
}

void BallTrackingRos::shutdown() {
  _nh.shutdown();
  // ros::shutdown();
}

void BallTrackingRos::loop() {
  _state->execute();
}

void BallTrackingRos::setState(State *state) {
  _state = state;
}

/* void BallTrackingRos::tracking() {
  StateContext::tracking();
} */

void BallTrackingRos::wait() {
  _state->wait(this);
}

void BallTrackingRos::init() {
  _state->init(this);
}

/* void StrategyWait::execute() {

  if (ros::Duration(ros::Time::now() - _timer) >= _timeOut) {

    threshold_t thresh;
    if (Utils::readThresholds(_context->getConfFile(), thresh)) {
      ROS_INFO("Data available");
      _context->tracking();
    } else
      ROS_WARN("No data");

    _timer = ros::Time::now();
  }
} */

/* bool StrategyTracking::init() {
  threshold_t threshold;
  if (Utils::readThresholds(_context->getConfFile(), threshold))
    _bt = BallTracking(_vh.getWidth(), _vh.getHeight(), threshold);
  else
    return false;

  if (!_cameraModel.fromCameraInfo(_context->getCameraInfo())) {
    ROS_ERROR("Failed to build model of camera. Exit.");
    return false;
  }

  return true;
} */