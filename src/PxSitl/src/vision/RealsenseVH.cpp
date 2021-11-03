#include "../../include/PxSitl/Vision/RealsenseVH.hpp"

RealsenseVH::RealsenseVH(uint16_t width, uint16_t height, rs2::pipeline &rsPipe)
    : VideoHandler(width, height), _rsPipe(rsPipe) {
  _rsProfile = _rsPipe.get_active_profile();
}

RealsenseVH::~RealsenseVH() {}

void RealsenseVH::read(cv::Mat &frame) {
  _frameset = _rsPipe.wait_for_frames();
  rs2::video_frame color = _frameset.get_color_frame();
  frame = cv::Mat(VideoHandler::getVideoSize(), CV_8UC3, (void *)color.get_data(), cv::Mat::AUTO_STEP);
}
