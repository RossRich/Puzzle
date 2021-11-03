#if !defined(_VIDEO_HANDLER_H_)
#define _VIDEO_HANDLER_H_

#include <inttypes.h>
#include <opencv2/core/core.hpp>

class VideoHandler {
private:
  uint16_t _width;
  uint16_t _height;

public:
  VideoHandler(uint16_t width, uint16_t height) : _width(width), _height(height) {}
  virtual ~VideoHandler() {}
  virtual void read(cv::Mat &frame) = 0;

  inline cv::Size2i getVideoSize() const {
    return cv::Size2i(_width, _height);
  }

  inline uint16_t getWidth() const { return _width; }
  inline uint16_t getHeight() const  { return _height; }

  friend VideoHandler& operator>> (VideoHandler &, cv::Mat&);
};

#endif // _VIDEO_HANDLER_H_
