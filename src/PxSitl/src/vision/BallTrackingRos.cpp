#include "../../include/PxSitl/vision/BallTrackingRos.hpp"

BallTrackingRos::BallTrackingRos(const ros::NodeHandle &nh, const image_transport::ImageTransport &it)
    : _nh(nh), _it(it) {
  _itRgb.subscribe(_it, "/camera/color/image_raw", 1);
  _itDepth.subscribe(_it, "/camera/aligned_depth_to_color/image_raw", 1);
  _sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(3), _itRgb, _itDepth);
  _sync->registerCallback(boost::bind(&BallTrackingRos::imageSubCb, this, _1, _2));

  int imgWidth;
  int imgHeight;
  _nh.param<int>("/camera/realsense2_camera/color_width", imgWidth, 640);
  _nh.param<int>("/camera/realsense2_camera/color_height", imgHeight, 480);

  ROS_INFO("Ing width: %d * height: %d", imgWidth, imgHeight);

  _bt = BallTracking(imgWidth, imgHeight, cv::Vec<cv::Scalar_<uint8_t>, 2>());
  cv::namedWindow(_winName, cv::WINDOW_AUTOSIZE);
}

BallTrackingRos::~BallTrackingRos() {
  cv::destroyAllWindows();
  delete _sync;
}

void BallTrackingRos::imageSubCb(const ImageConstPtr &rgb, const ImageConstPtr &d) {

  try {
    _color = cv_bridge::toCvCopy(rgb, enc::RGB8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert  from '%s' to 'bgr8'.", _color->encoding.c_str());
  }

  try {
    _depth = cv_bridge::toCvCopy(d, enc::TYPE_16UC1);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert  from '%s' to 'TYPE_16UC1'", _depth->encoding.c_str());
  }
  run();
}

void BallTrackingRos::run() {
  if (!_color->image.empty() && !_depth->image.empty()) {
    Mat frame;

    cv::Point2i pos = _bt.process(_color->image);
    if (pos.x != 0 && pos.y != 0) {

      // std::cout << pos << std::endl;
      // cv::Matx33f dists = _depth->image(cv::Rect2i(int(pos.x) - 1, int(pos.y) - 1, 3, 3)).clone();
      // cv::Matx33f d = cv::Matx33f::all(0.001f);
      // dists = dists.mul(d);
      // std::cout << dists(1,1) << std::endl;
      std::cout <<_depth->image.at<uint16_t>(pos) * 0.001f << std::endl;
      // float point3d [3];
      // rs2_intrinsics 
      // std::cout << rs2_deproject_pixel_to_point(point3d, )
    }

    if (!frame.empty())
      cv::imshow(_winName, frame);
  }
  cv::waitKey(1);
}