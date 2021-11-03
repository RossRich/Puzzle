#include "../../include/PxSitl/Vision/VideoHandler.hpp"

VideoHandler& operator>>(VideoHandler &vh, cv::Mat &img) {
  vh.read(img);
  return vh;
}