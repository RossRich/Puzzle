#include <deque>
#include <librealsense2/rs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include "Utils.hpp"

using cv::Mat;
using std::cerr;
using std::cout;
using std::endl;

enum AlignTo { color, depth };

void alignChangeButtonCallback(int statet, void *data) {
  if (*((uint8_t *)data) == AlignTo::color) {
    *((uint8_t *)data) = AlignTo::depth;
    cout << "Align to depth" << endl;
  } else {
    *((uint8_t *)data) = AlignTo::color;
    cout << "Align to color" << endl;
  }
}

int main(int argc, char *argv[]) {
  rs2::context ctx;
  rs2::device_list devList = ctx.query_devices();

  if (devList.size() == 0) {
    cerr << "No connected realsense" << endl;
    return EXIT_FAILURE;
  }

  rs2::device dev = devList.front();
  cout << dev.get_info(rs2_camera_info::RS2_CAMERA_INFO_NAME) << endl;

  const char *confFile = "/workspaces/Puzzle/src/PxSitl/data/config.yaml";

  cv::FileStorage conf;
  cv::Vec<cv::Scalar_<uint8_t>, 2> threshold;
  bool isDataAvalable = false;

  try {
    if (conf.open(confFile, cv::FileStorage::READ)) {
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
    std::cerr << e.what() << '\n';
  }

  const char *mainWin = "Main";

  // ros::init(argc, argv, "vison");

  // cv::VideoCapture vc;
  // vc.open(0);

  rs2::pipeline rs;

  rs2::pipeline_profile rsProfile = rs.start();

  // if (!vc.isOpened()) {
  //   ROS_ERROR("No video stream");
  //   return -1;
  // }

  // uint16_t imWidth = vc.get(cv::CAP_PROP_FRAME_WIDTH) / 2;
  // uint16_t imHeight = vc.get(cv::CAP_PROP_FRAME_HEIGHT) / 2;

  rs2::video_stream_profile rsVideoProfile = rs2::video_stream_profile(rsProfile.get_stream(RS2_STREAM_COLOR));

  rs2::video_stream_profile rsDepthProfile = rs2::video_stream_profile(rsProfile.get_stream(RS2_STREAM_DEPTH));

  std::stringstream info;
  info << "stream name: " << rsVideoProfile.stream_name() << "\nwidth: " << rsVideoProfile.width()
       << "\nheight: " << rsVideoProfile.height() << "\nfps: " << rsVideoProfile.fps();

  cout << info.str() << endl;
  info.str("");

  info << "stream name: " << rsDepthProfile.stream_name() << "\nwidth: " << rsDepthProfile.width()
       << "\nheight: " << rsDepthProfile.height() << "\nfps: " << rsDepthProfile.fps();

  cout << info.str() << endl;
  info.str("");

  uint16_t imWidth = rsDepthProfile.width();
  uint16_t imHeight = rsDepthProfile.height();

  if (!isDataAvalable) {
    if (!conf.open(confFile, cv::FileStorage::WRITE)) {
      std::cerr << "Faild open file in " << confFile << endl;
      rs.stop();
      return -1;
    }
    threshold = Utils::findThreshold(rs, conf);
  }

  cv::namedWindow(mainWin, cv::WINDOW_AUTOSIZE);

  uint8_t align = AlignTo::depth;
  cv::createButton("Align to color", alignChangeButtonCallback, (void *)&align, cv::QtButtonTypes::QT_RADIOBOX, true);
  cv::createButton("Align to depth", nullptr, (void *)&align, cv::QtButtonTypes::QT_RADIOBOX, false);

  int transparent = 5;
  cv::createTrackbar("Transparent", mainWin, &transparent, 10);

  Mat frame;
  Mat view(cv::Size2i(imWidth * 2, imHeight), CV_8UC3, cv::Scalar::all(255));
  Mat view1 = view(cv::Rect2i(0, 0, imWidth, imHeight));
  Mat view2 = view(cv::Rect2i(imWidth, 0, imWidth, imHeight));

  rs2::frameset rsAlignedFrames;
  rs2::frameset rsFrameset;
  rs2::align alignToDepth(RS2_STREAM_DEPTH);
  rs2::align alignToColor(RS2_STREAM_COLOR);
  rs2::colorizer rsColoredDepth;

  std::vector<std::vector<cv::Point2i>> cnt;

  time_t timerInfo = time(nullptr);
  time_t timerStatusText = time(nullptr);

  while (true) {
    // vc >> frame;

    rsFrameset = rs.wait_for_frames();

    if (!rsFrameset)
      continue;

    if (align == AlignTo::color) {
      rsAlignedFrames = alignToColor.process(rsFrameset);
    } else {
      rsAlignedFrames = alignToDepth.process(rsFrameset);
    }

    auto color = rsAlignedFrames.get_color_frame();
    auto depth = rsAlignedFrames.get_depth_frame();
    auto colored = rsColoredDepth.colorize(depth);

    Mat colorMat(cv::Size2i(color.get_width(), color.get_height()), CV_8UC3, (void *)color.get_data());
    Mat depthMat(cv::Size2i(colored.get_width(), colored.get_height()), CV_8UC3, (void *)colored.get_data());

    if (difftime(time(nullptr), timerInfo) > 5) {
      cout << "Color size: " << color.get_width() << " x " << color.get_height() << endl;
      cout << "Depth size: " << depth.get_width() << " x " << depth.get_height() << endl;
      cout << "Colored depth size: " << colored.get_width() << " x " << colored.get_height() << endl;
      timerInfo = time(nullptr);
    }

    Mat res;
    double kFactor = transparent / 10.0;
    if (align == AlignTo::color) {
      cv::addWeighted(colorMat, kFactor, depthMat, 1.0 - kFactor, 0, res);
    } else {
      cv::addWeighted(depthMat, kFactor, colorMat, 1.0 - kFactor, 0, res);
    }

    frame = Mat(cv::Size2i(rsVideoProfile.width(), rsVideoProfile.height()), CV_8UC3,
                (void *)rsFrameset.get_color_frame().get_data());

    if (frame.empty())
      continue;

    cv::resize(frame, frame, cv::Size2i(imWidth, imHeight));

    Mat mask;
    GaussianBlur(frame, mask, cv::Size2i(11, 11), 0);
    cv::cvtColor(frame, mask, cv::COLOR_RGB2HSV);
    cv::inRange(mask, cv::Scalar(threshold[0][0], threshold[0][1], threshold[0][2]),
                cv::Scalar(threshold[1][0], threshold[1][1], threshold[1][2]), mask);

    cv::erode(mask, mask, Mat(), cv::Point2i(), 5);
    cv::dilate(mask, mask, Mat(), cv::Point2i(), 6);

    cv::findContours(mask.clone(), cnt, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // cv::drawContours(frame, cnt, -1, cv::Scalar::all(255));
    // cout << "Contours count " << cnt.size() << endl;
    float radius = 0.0;
    cv::Point2f center;
    if (cnt.size() > 0) {
      std::vector<cv::Point> cnt0 = cnt[0];
      int maxSiza = cnt[0].size();
      for (auto c : cnt) {
        if (c.size() > maxSiza) {
          cnt0 = c;
          maxSiza = c.size();
        }
      }

      // Mat cc = cv::max(cnt0, cv::contourArea(cnt0));
      cv::minEnclosingCircle(cnt0, center, radius);
      // cv::Moments m = cv::moments(cnt0, true);

      // center.x = m.m10 / m.m00;
      // center.y = m.m01 / m.m00;

      if (difftime(time(nullptr), timerStatusText) > 0.10) {
        cout << "Coordinate of center: " << Mat(center) << endl;
        info.str("");
        info << "Cx: " << center.x << " Cy: " << center.y << " Distance: " << rsFrameset.get_depth_frame().get_distance(center.x, center.y);

        timerStatusText = time(nullptr);
      }

      cv::circle(frame, center, radius + 7.0, cv::Scalar(threshold[1]), 1, cv::LINE_4);
      cv::circle(frame, center, 3, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);
      cv::Point2f textPos = {center.x + radius + 15.0f, center.y + radius + 15.0f};
      cv::putText(frame, info.str(), textPos, cv::FONT_HERSHEY_SIMPLEX, .6, cv::Scalar::all(0), 2);
      cv::displayStatusBar(mainWin, info.str());
    }

    cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR);
    cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
    mask.copyTo(view2);
    frame.copyTo(view1);

    cv::imshow(mainWin, view);

    cv::cvtColor(res, res, cv::COLOR_RGB2BGR);
    cv::imshow("Depth", res);

    char c = cv::waitKey(1);
    if (c == 'q')
      break;
  }

  conf.release();
  rs.stop();
  // vc.release();
  cv::destroyAllWindows();
  return 0;
}
