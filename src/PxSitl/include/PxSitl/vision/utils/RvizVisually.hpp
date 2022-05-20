#if !defined(_VISION_RVIZ_VISUALLY)
#define _VISION_RVIZ_VISUALLY

#include <visualization_msgs/Marker.h>

class RvizVisually {
private:
  RvizVisually() {}

public:
  ~RvizVisually() {}
  enum Color { Black = 0, SonicSilver, White, CyanProcess, Blue, AlfaBlue, CaribbeanGreen, DarkGreen, Red, Marigold, Yellow, Green };

  /**
   * Wrapper function for get "color msg" of ros  as "one shot"
   * 
   * @param r red[0 - 1.0f]
   * @param g green[0 - 1.0f]
   * @param b blue[0 - 1.0f]
   * @param a opacity faktor[0 - 1.0f]
   * 
   * @return ROS ColorRGBA
   */
  static std_msgs::ColorRGBA getColorMsg(float r, float g, float b, float a = 1.0f) {
    std_msgs::ColorRGBA color;
    color.a = a;
    color.r = r;
    color.g = g;
    color.b = b;

    return color;
  }

  /**
   * Create vector of ROS ColorRGBA
   *
   * @return colors link
   */
  static std::vector<std_msgs::ColorRGBA> &Colors() {
    static std::vector<std_msgs::ColorRGBA> colors;

    if(colors.size() != 0)
      return colors;

    ///< Black(0) - 33, 33, 33
    colors.push_back(RvizVisually::getColorMsg(33.f / 255.f, 33.f / 255.f, 33.f / 255.f, 0.8f));
    ///< SonicSilver(1) - 117, 117, 117
    colors.push_back(RvizVisually::getColorMsg(117.f / 255.f, 117.f / 255.f, 117.f / 255.f, 1.0f));
    ///< White(2) - 251, 251, 255
    colors.push_back(RvizVisually::getColorMsg(251.f / 255.f, 251.f / 255.f, 255.f / 255.f, 1.0f));
    ///< CyanProcess(3) - 1, 186, 239
    colors.push_back(RvizVisually::getColorMsg(1.f / 255.f, 186.f / 255.f, 239.f / 255.f, 1.0f));
    ///< Blue(4) - 21, 229, 244
    colors.push_back(RvizVisually::getColorMsg(21.f / 255.f, 229.f / 255.f, 244.f / 255.f, 1.0f));
    ///< AlfaBlue(5)
    colors.push_back(RvizVisually::getColorMsg(21.f / 255.f, 229.f / 255.f, 244.f / 255.f, 0.55f));
    ///< CaribbeanGreen(6) - 19, 205, 177
    colors.push_back(RvizVisually::getColorMsg(19.f / 255.f, 205.f / 255.f, 177.f / 255.f, 1.0f));
    ///< DarkGreen(7) - 32, 191, 85
    colors.push_back(RvizVisually::getColorMsg(32.f / 255.f, 191.f / 255.f, 85.f / 255.f, 1.0f));
    ///< Red(8) - 255, 0, 0
    colors.push_back(RvizVisually::getColorMsg(255.f / 255.f, 0.f / 255.f, 0.f / 255.f, 1.0f));
    ///< Marigold(9) - 229, 165, 36
    colors.push_back(RvizVisually::getColorMsg(229.f / 255.f, 165.f / 255.f, 36.f / 255.f, 1.0f));
    ///< Yellow(10) - 226, 245, 60
    colors.push_back(RvizVisually::getColorMsg(226.f / 255.f, 245.f / 255.f, 60.f / 255.f, 1.0f));
    ///< Green(11) - 0, 0, 255
    colors.push_back(RvizVisually::getColorMsg(0.f / 255.f, 255.f / 255.f, 0.f / 255.f, 1.0f));

    return colors;
  }
};
#endif // _VISION_RVIZ_VISUALLY
