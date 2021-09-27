#include <deque>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>

using cv::Mat;
using std::cout;
using std::endl;

cv::Vec<cv::Scalar_<uint8_t>, 2> findThreshold(cv::VideoCapture &cap,
                                               cv::FileStorage &settings) {

  const char *winName = "Do mask";

  Mat frame;
  cv::namedWindow(winName, cv::WINDOW_AUTOSIZE | cv::WINDOW_KEEPRATIO |
                               cv::WINDOW_GUI_NORMAL);

  uint16_t imWidht = cap.get(cv::CAP_PROP_FRAME_WIDTH);
  uint16_t imHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT);

  cv::Size2i rectSize(30, 30);
  cv::Point2i rectPos(cvRound(imWidht / 2.0) - cvRound(rectSize.width / 2.0),
                      cvRound(imHeight / 2.0) - cvRound(rectSize.height / 2.0));

  char c = ' ';
  cv::Scalar_<uint8_t> max;
  cv::Scalar_<uint8_t> min = {255, 255, 255};

  bool isScan = false;

  while (c != 'q') {
    cap >> frame;

    Mat roi(frame, cv::Rect2i(rectPos, rectSize));

    cv::cvtColor(roi, roi, cv::COLOR_BGR2HSV);

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
    cv::imshow(winName, frame);
    c = cv::waitKey(5);
  }

  settings << "threshold"
           << "{";
  settings << "min" << min;
  settings << "max" << max;
  settings << "}";

  cv::destroyWindow(winName);

  return {min, max};
}

int main(int argc, char *argv[]) {
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

  cv::VideoCapture vc;
  vc.open(0);

  if (!vc.isOpened()) {
    ROS_ERROR("No video stream");
    return -1;
  }

  uint16_t imWidth = vc.get(cv::CAP_PROP_FRAME_WIDTH) / 2;
  uint16_t imHeight = vc.get(cv::CAP_PROP_FRAME_HEIGHT) / 2;

  ROS_INFO("Width: %i, height: %i", imWidth, imHeight);

  if (!isDataAvalable) {
    if (!conf.open(confFile, cv::FileStorage::WRITE)) {
      std::cerr << "Faild open file in " << confFile << endl;
      vc.release();
      return -1;
    }
    threshold = findThreshold(vc, conf);
  }

  cv::namedWindow(mainWin, cv::WINDOW_AUTOSIZE);

  Mat frame;

  Mat view(cv::Size2i(imWidth * 2, imHeight), CV_8UC3, cv::Scalar::all(255));
  Mat view1 = view(cv::Rect2i(0, 0, imWidth, imHeight));
  Mat view2 = view(cv::Rect2i(imWidth, 0, imWidth, imHeight));

  std::vector<std::vector<cv::Point2i>> cnt;

  while (true) {
    vc >> frame;

    if (frame.empty())
      break;

    cv::resize(frame, frame, cv::Size2i(imWidth, imHeight));

    Mat mask;
    GaussianBlur(frame, mask, cv::Size2i(11, 11), 0);
    cv::cvtColor(frame, mask, cv::COLOR_BGR2HSV);
    cv::inRange(
        mask, cv::Scalar(threshold[0][0], threshold[0][1], threshold[0][2]),
        cv::Scalar(threshold[1][0], threshold[1][1], threshold[1][2]), mask);

    cv::erode(mask, mask, Mat(), cv::Point2i(), 5);
    cv::dilate(mask, mask, Mat(), cv::Point2i(), 6);

    cv::findContours(mask.clone(), cnt, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    // cv::drawContours(frame, cnt, -1, cv::Scalar::all(255));
    // cout << "Contours count " << cnt.size() << endl;
    float radius = 0.0;
    cv::Point2f center;
    if(cnt.size() > 0) {
      std::vector<cv::Point> cnt0 = cnt[0];
      
      // Mat cc = cv::max(cnt0, cv::contourArea(cnt0));
      cv::minEnclosingCircle(cnt0, center, radius);
      // cv::Moments m = cv::moments(cnt0, true);

      // center.x = m.m10 / m.m00;
      // center.y = m.m01 / m.m00;

      cout << "Coordinate of center: " << Mat(center) << endl;

      cv::circle(frame, center, radius + 7.0, cv::Scalar(threshold[1]), 1, cv::LINE_4);
      cv::circle(frame, center, 3, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);

      std::stringstream info;
      info << "Cx: " << center.x << " Cy: " << center.y; 
      cv::Point2f textPos = {center.x + radius + 15.0, center.y + radius + 15.0};
      cv::putText(frame, info.str(), textPos, cv::FONT_HERSHEY_SIMPLEX, .6, cv::Scalar::all(0), 2);
    }

    cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR);
    mask.copyTo(view2);
    frame.copyTo(view1);

    cv::imshow(mainWin, view);

    char c = cv::waitKey(10);
    if (c == 'q')
      break;
  }

  conf.release();
  vc.release();
  cv::destroyAllWindows();
  return 0;
}
