#if !defined(_VISION_UTILS_H_)
#define _VISION_UTILS_H_

#include "thresholdtype.hpp"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <visualization_msgs/Marker.h>

class Utils {
private:
  Utils() {}

public:
  ~Utils() {}

  enum Color {
    Black = 0,
    SonicSilver,
    White,
    CyanProcess,
    Blue,
    AlfaBlue,
    Green,
    DarkGreen,
    Red,
    Marigold,
    Yellow
  };

  /**
   * Create vector of colors
   *
   * @return colors link
   */
  static std::vector<std_msgs::ColorRGBA> &createColors() {
    static std::vector<std_msgs::ColorRGBA> colors;
    ///< Black(0) - 33, 33, 33
    colors.push_back(Utils::getColorMsg(33.f / 255.f, 33.f / 255.f, 33.f / 255.f, 0.8f));
    ///< SonicSilver(1) - 117, 117, 117
    colors.push_back(Utils::getColorMsg(117.f / 255.f, 117.f / 255.f, 117.f / 255.f, 1.0f));
    ///< White(2) - 251, 251, 255
    colors.push_back(Utils::getColorMsg(251.f / 255.f, 251.f / 255.f, 255.f / 255.f, 1.0f));
    ///< CyanProcess(3) - 1, 186, 239
    colors.push_back(Utils::getColorMsg(1.f / 255.f, 186.f / 255.f, 239.f / 255.f, 1.0f));
    ///< Blue(4) - 21, 229, 244
    colors.push_back(Utils::getColorMsg(21.f / 255.f, 229.f / 255.f, 244.f / 255.f, 1.0f));
    ///< Blue(5)
    colors.push_back(Utils::getColorMsg(21.f / 255.f, 229.f / 255.f, 244.f / 255.f, 0.65f));
    ///< Green(6) - 19, 205, 177
    colors.push_back(Utils::getColorMsg(19.f / 255.f, 205.f / 255.f, 177.f / 255.f, 1.0f));
    ///< DarkGreen(7) - 32, 191, 85
    colors.push_back(Utils::getColorMsg(32.f / 255.f, 191.f / 255.f, 85.f / 255.f, 1.0f));
    ///< Red(8) - 238, 104, 59
    colors.push_back(Utils::getColorMsg(238.f / 255.f, 104.f / 255.f, 59.f / 255.f, 1.0f));
    ///< Marigold(9) - 229, 165, 36
    colors.push_back(Utils::getColorMsg(229.f / 255.f, 165.f / 255.f, 36.f / 255.f, 1.0f));
    ///< Yellow(10) - 226, 245, 60
    colors.push_back(Utils::getColorMsg(226.f / 255.f, 245.f / 255.f, 60.f / 255.f, 1.0f));

    return colors;
  }

  static std::vector<std_msgs::ColorRGBA> &Colors;

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

  template <typename T>
  static T fastFilter(T oldVar, T newVar, float gain) {
    return oldVar * gain + (newVar * (1.0f - gain));
  }

  static void fastFilterCvPoint3d(cv::Point3d &point, cv::Point3d &newPoint, float gain) {
    point.x = fastFilter(point.x, newPoint.x, gain);
    point.y = fastFilter(point.y, newPoint.y, gain);
    point.z = fastFilter(point.z, newPoint.z, gain);
  }

  static std_msgs::ColorRGBA getColorMsg(float r, float g, float b, float a = 1.0f) {
    std_msgs::ColorRGBA color;
    color.a = a;
    color.r = r;
    color.g = g;
    color.b = b;
    return color;
  }
};

#endif // _VISION_UTILS_H_
