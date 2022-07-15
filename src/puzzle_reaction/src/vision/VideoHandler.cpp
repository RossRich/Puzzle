#include "puzzle_reaction/vision/VideoHandler.hpp"

VideoHandler& operator>>(VideoHandler &vh, cv::Mat &img) {
  vh.readColor(img);
  return vh;
}