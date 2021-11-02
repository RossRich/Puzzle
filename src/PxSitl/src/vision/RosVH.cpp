#include "../../include/PxSitl/vision/RosVH.hpp"

RosVH::RosVH(const ros::NodeHandle nh, uint16_t width, uint16_t height)
    : VideoHandler(width, height), _nh(nh) {}

RosVH::~RosVH() {}

void RosVH::read(cv::Mat &frame) {}
