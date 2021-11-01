#include "VideoHandler.hpp"
#include <librealsense2/rs.hpp>

class RealsenseVH : public VideoHandler {
private:
  rs2::pipeline &_rsPipe;
  rs2::pipeline_profile _rsProfile;
  rs2::frameset _frameset;
  
public:
  RealsenseVH(uint16_t width, uint16_t height, rs2::pipeline &rsPipe);
  ~RealsenseVH();

  void read(cv::Mat &frame) override; 

  // friend RealsenseVH& operator>>( RealsenseVH&, cv::Mat&);
};