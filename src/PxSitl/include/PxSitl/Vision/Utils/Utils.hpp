#if !defined(_VISION_UTILS_H_)
#define _VISION_UTILS_H_

#include "thresholdtype.hpp"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <iostream>

class Utils {
private:
  Utils() {}

public:
  ~Utils() {}

  static bool readThresholds(const char *fileName, threshold_t &threshold) {
    cv::FileStorage conf;
    bool isDataAvalable = false;

    try {
      if (conf.open(fileName, cv::FileStorage::READ)) {
        cv::FileNode thresholdNode = conf["threshold"];
        if (!thresholdNode.empty()) {
          int i = 0;
          for (auto t : thresholdNode)
            thresholdNode[t.name()] >> threshold[i++];

          isDataAvalable = true;
        }
      }

      conf.release();
    } catch (cv::Exception e) {
      std::cerr << e.what() << std::endl;
    }

    if (!isDataAvalable) {
      std::cerr << "No available threshold data in file" << std::endl;
    } else {
      std::cout << "Threshold is available" << std::endl;
    }

    return isDataAvalable;
  }

  static bool writeThreshold(const char *fileName, threshold_t &threshold) {
    cv::FileStorage conf;
    try {
      conf.open(fileName, cv::FileStorage::WRITE);
    } catch (const cv::Exception &e) {
      std::cerr << e.what() << '\n';
      std::cerr << "Failed to write a new threshold to file\n";
      return false;
    }

    if (!conf.isOpened()) {
      std::cerr << "Faild to open the file " << fileName << std::endl;
      return false;
    }

    conf << "threshold"
         << "{";
    conf << "min" << threshold[0];
    conf << "max" << threshold[1];
    conf << "}";

    conf.release();

    std::cout << "New threshold saved\n";

    return true;
  }

  template <typename T> static T fastFilter(T oldVar, T newVar, float gain) {
    return oldVar * gain + (newVar * (1.0f - gain));
  }

  static void fastFilterCvPoint3d(cv::Point3d &point, cv::Point3d &newPoint, float gain) {
    point.x = fastFilter(point.x, newPoint.x, gain);
    point.y = fastFilter(point.y, newPoint.y, gain);
    point.z = fastFilter(point.z, newPoint.z, gain);
  }
};

#endif // _VISION_UTILS_H_
