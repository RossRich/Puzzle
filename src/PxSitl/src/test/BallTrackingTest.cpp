#include "../../include/PxSitl/vision/BallTracking.hpp"
#include "../../include/PxSitl/vision/RealsenseVH.hpp"
#include "../../include/PxSitl/vision/TrackingParam.hpp"
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char const *argv[]) {
  rs2::context cnxt;
  rs2::device_list devsList = cnxt.query_devices();

  if (devsList.size() == 0) {
    std::cerr << "No connected realsense" << std::endl;
    return EXIT_FAILURE;
  }

  for (auto &&dev : devsList)
    std::cout << dev.get_info(rs2_camera_info::RS2_CAMERA_INFO_NAME) << " was found\n";

  rs2::pipeline rsPipe;
  rs2::config cfg;

  uint16_t cameraWidth = 1280;
  uint16_t cameraHeight = 640;

  cfg.enable_stream(rs2_stream::RS2_STREAM_COLOR, cameraWidth, cameraHeight);

  try {
    rsPipe.start();
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  RealsenseVH rsVideoHandler(cameraWidth, cameraHeight, rsPipe);
  cv::Mat frame(rsVideoHandler.getVideoSize(), CV_8UC3);

  TrackingParam tr(rsVideoHandler);

  threshold_t threshold;
  if (!tr.getThreshold(threshold)) {
    Mat mask;
    tr.maskFormGUI(mask);
    if (tr.newThreshold(mask)) {
      cout << "New threshold saved" << endl;
      tr.getThreshold(threshold);
    } else {
      std::cerr << "New threshold not save\n";
      rsPipe.stop();
      cv::destroyAllWindows();
      return EXIT_FAILURE;
    }
  }

  std::cout << threshold[0] << endl;
  std::cout << threshold[1] << endl;

  BallTracking bt(cameraWidth, cameraHeight, threshold);
  
  char c = ' ';

  uint16_t radius = 0;
  Point2i center = {0};

  Mat m;
  while (c != 'q') {
    rsVideoHandler >> frame;

    if (frame.empty())
      break;

    bt.process(frame, m, &center, &radius);

    cv::circle(frame, center, radius + 7.0, cv::Scalar(threshold[1]), 1, cv::LINE_4);
    cv::circle(frame, center, 3, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);
    // cv::Point2f textPos = {center.x + radius + 15.0f, center.y + radius + 15.0f};
    // cv::putText(frame, _info.str(), textPos, cv::FONT_HERSHEY_SIMPLEX, .6, cv::Scalar::all(0), 2);

    cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);

    cv::imshow("TMP", m);

    c = cv::waitKey(1);
  }

  cv::destroyWindow("TMP");
  rsPipe.stop();

  return 0;
}
