#if !defined(_UTILS_H_)
#define _UTILS_H_

#include <iostream>
#include <librealsense2/rs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using cv::Mat;
using std::cout;
using std::endl;

class Utils {
private:
  Utils();

public:
  ~Utils();

  static cv::Vec<cv::Scalar_<uint8_t>, 2> findThreshold(rs2::pipeline &rs, cv::FileStorage &settings) {

    const char *winName = "Do mask";
    cv::namedWindow(winName, cv::WINDOW_AUTOSIZE | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_NORMAL);

    rs2::pipeline_profile rsProfile = rs.get_active_profile();

    uint16_t imWidht = rs2::video_stream_profile(rsProfile.get_stream(RS2_STREAM_COLOR)).width();
    uint16_t imHeight = rs2::video_stream_profile(rsProfile.get_stream(RS2_STREAM_COLOR)).height();
    Mat frame(cv::Size2i(imWidht, imHeight), CV_8UC3);

    cv::Size2i rectSize(30, 30);
    cv::Point2i rectPos(cvRound(imWidht / 2.0) - cvRound(rectSize.width / 2.0),
                        cvRound(imHeight / 2.0) - cvRound(rectSize.height / 2.0));

    char c = ' ';
    cv::Scalar_<uint8_t> max;
    cv::Scalar_<uint8_t> min = {255, 255, 255};

    bool isScan = false;
    rs2::frameset frames;
    while (c != 'q') {
      frames = rs.wait_for_frames();

      frame.data = (uint8_t *)frames.get_color_frame().get_data();

      Mat roi(frame, cv::Rect2i(rectPos, rectSize));

      cv::GaussianBlur(frame, frame, cv::Size2i(9, 9), 1);

      cv::cvtColor(roi, roi, cv::COLOR_RGB2HSV);

      switch (c) {
      case 's':
        isScan = !isScan;
        cout << "Started to scan a color..." << endl;
        break;
      case 'r':
        cout << "Restarted a colors" << endl;
        max = max.all(0);
        min = min.all(255);
        break;
      default:
        break;
      }

      if (isScan) {
        for (size_t i = 0; i < roi.rows; i++) {
          for (size_t j = 0; j < roi.cols; j++) {
            cv::Vec3b row = roi.at<cv::Vec3b>(i, j);
            if (max[0] < row[0]) {
              max[0] = row[0];
            }

            if (row[0] < min[0]) {
              min[0] = row[0];
            }

            if (max[1] < row[1]) {
              max[1] = row[1];
            }

            if (row[1] < min[1]) {
              min[1] = row[1];
            }

            if (max[2] < row[2]) {
              max[2] = row[2];
            }

            if (row[2] < min[2]) {
              min[2] = row[2];
            }
          }
        }
      }

      cv::rectangle(frame, cv::Rect2i(rectPos, rectSize), max, 2);
      cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
      cv::imshow(winName, frame);
      c = cv::waitKey(1);
    }

    settings << "threshold"
             << "{";
    settings << "min" << min;
    settings << "max" << max;
    settings << "}";

    cv::destroyWindow(winName);

    return {min, max};
  }

  
  // https://github.com/IntelRealSense/librealsense/blob/master/wrappers/opencv/cv-helpers.hpp
  // Convert rs2::frame to cv::Mat
  static cv::Mat frame2Mat(const rs2::frame &f) {
    using namespace cv;
    using namespace rs2;

    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    if (f.get_profile().format() == RS2_FORMAT_BGR8) {
      return Mat(Size(w, h), CV_8UC3, (void *)f.get_data(), Mat::AUTO_STEP);
    } else if (f.get_profile().format() == RS2_FORMAT_RGB8) {
      auto r_rgb = Mat(Size(w, h), CV_8UC3, (void *)f.get_data(), Mat::AUTO_STEP);
      Mat r_bgr;
      cvtColor(r_rgb, r_bgr, COLOR_RGB2BGR);
      return r_bgr;
    } else if (f.get_profile().format() == RS2_FORMAT_Z16) {
      return Mat(Size(w, h), CV_16UC1, (void *)f.get_data(), Mat::AUTO_STEP);
    } else if (f.get_profile().format() == RS2_FORMAT_Y8) {
      return Mat(Size(w, h), CV_8UC1, (void *)f.get_data(), Mat::AUTO_STEP);
    } else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32) {
      return Mat(Size(w, h), CV_32FC1, (void *)f.get_data(), Mat::AUTO_STEP);
    }

    throw std::runtime_error("Frame format is not supported yet!");
  }

  // Converts depth frame to a matrix of doubles with distances in meters
  static cv::Mat depthFrame2Meters(const rs2::depth_frame &f) {
    cv::Mat dm = frame2Mat(f);
    dm.convertTo(dm, CV_64F);
    dm = dm * f.get_units();
    return dm;
  }
};

#endif // _UTILS_H_
