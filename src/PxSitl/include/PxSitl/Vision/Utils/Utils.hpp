#if !defined(_VISION_UTILS_H_)
#define _VISION_UTILS_H_

#include "thresholdtype.hpp"
#include <opencv2/core/core.hpp>

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
};

#endif // _VISION_UTILS_H_
