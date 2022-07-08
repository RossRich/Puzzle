#include <iostream>
#include <librealsense2/rs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <time.h>

int main(int argc, char const *argv[]) {

  rs2::context ctx;
  auto list = ctx.query_devices();

  if (list.size() == 0) {
    std::cerr << "No realsense device detected.";
    return EXIT_FAILURE;
  }

  rs2::device dev = list.front();

  rs2::pipeline rsPipe;
  rs2::config cfg;
  rs2::colorizer c;
  rs2::rates_printer printer;
  cfg.enable_stream(RS2_STREAM_DEPTH);
  cfg.enable_stream(RS2_STREAM_COLOR);
  rsPipe.start(cfg);

  time_t start, end;

  cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE);

  char ch = ' ';
  while (ch != 'q') {

    rs2::frameset frames = rsPipe.wait_for_frames();

    rs2::depth_frame dFrame = frames.get_depth_frame();
    rs2::video_frame imgColor = frames.get_color_frame();
    rs2::video_frame test = dFrame.apply_filter(c);

    if (dFrame) {
      int w = test.get_width();
      int h = test.get_height();

      cv::Mat depthMat(cv::Size2i(dFrame.get_width(), dFrame.get_height()), CV_16UC1, (void*)dFrame.get_data(), cv::Mat::AUTO_STEP);
      std::cout << "Depth: " << dFrame.get_width() << " * " << dFrame.get_height() << std::endl;

      cv::Mat img(cv::Size2i(w, h), CV_8UC3, (void*)test.get_data(), cv::Mat::AUTO_STEP);
      std::cout << "Img: " << imgColor.get_width() << " * " << imgColor.get_height() << std::endl;
      cv::imshow("Depth", img);
    }

    // std::cout << "Dist to center " << distToCenter << std::endl;

    ch = cv::waitKey(1);
  }

  rsPipe.stop();
  cv::destroyAllWindows();

  return 0;
}
