#if !defined(_ROS_VIDEO_HANDLER_H_)
#define _ROS_VIDEO_HANDLER_H_

#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include "VideoHandler.hpp"

class RosVH: public VideoHandler
{
private:
    const ros::NodeHandle &_nh;
public:
    RosVH(const ros::NodeHandle nh, uint16_t width, uint16_t height);
    ~RosVH();

    void read(cv::Mat &frame) override;
};

#endif // _ROS_VIDEO_HANDLER_H_
