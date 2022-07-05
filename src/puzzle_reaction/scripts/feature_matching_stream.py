import cv2
import numpy as np
# import pyrealsense2 as rs
import time

QUERY_IMG_SIZE = (640, 480)


def feature_matching_detect_obj(query_img, train_img):
    MIN_MATCH_COUNT = 30
    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
    search_params = dict(checks=50)

    query_img = cv2.resize(query_img, dsize=QUERY_IMG_SIZE)
    sift = cv2.xfeatures2d.SIFT_create()
    sift_kp1, sift_des1 = sift.detectAndCompute(query_img, None)
    sift_kp2, sift_des2 = sift.detectAndCompute(train_img, None)
    if (sift_des1 is not None and len(sift_des1) != 0) and \
            (sift_des2 is not None and len(sift_des2) != 0):
        flann = cv2.FlannBasedMatcher(index_params, search_params)
        sift_matches = flann.knnMatch(sift_des1, sift_des2, k=2)

        good = []
        for m, n in sift_matches:
            if m.distance < 0.6 * n.distance:
                good.append(m)

        if len(good) > MIN_MATCH_COUNT:
            print('Good match points:', len(good))
            src_pts = np.float32([sift_kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
            dst_pts = np.float32([sift_kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

            homo_matrix, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 10.0)
            homo_res_polyline_img = train_img
            matches_mask = None
            if homo_matrix is not None:
                matches_mask = mask.ravel().tolist()
                h, w, d = query_img.shape
                pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)
                dst = cv2.perspectiveTransform(pts, homo_matrix)
                homo_res_polyline_img = cv2.polylines(train_img, [np.int32(dst)],
                                                      True, 255, 3, cv2.LINE_AA)
        else:
            print("Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT))
            matches_mask = None
            homo_res_polyline_img = train_img

        draw_params = dict(matchColor=(0, 255, 0),
                           singlePointColor=None,
                           matchesMask=matches_mask,
                           flags=2)
        homo_res_img = cv2.drawMatches(query_img, sift_kp1, homo_res_polyline_img,
                                       sift_kp2, good, None, **draw_params)
    else:
        homo_res_img = cv2.drawMatches(query_img, sift_kp1, train_img, sift_kp2, None,
                                       None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

    return homo_res_img


DEVICE_DEPTH_IMG_SIZE = (1280, 720)
DEVICE_RGB_IMG_SIZE = (1280, 720)

# pipeline = rs.pipeline()
# config = rs.config()
# pipeline_wrapper = rs.pipeline_wrapper(pipeline)
# pipeline_profile = config.resolve(pipeline_wrapper)
# device = pipeline_profile.get_device()
# device_product_line = str(device.get_info(rs.camera_info.product_line))
# config.enable_stream(rs.stream.depth, DEVICE_DEPTH_IMG_SIZE[0], DEVICE_DEPTH_IMG_SIZE[1], rs.format.z16, 30)
# config.enable_stream(rs.stream.color, DEVICE_RGB_IMG_SIZE[0], DEVICE_RGB_IMG_SIZE[1], rs.format.bgr8, 30)

# pipeline.start(config)
# print('Device:', device_product_line)
print('Start streaming ...')

cap = cv2.VideoCapture(0)
if(cap.isOpened() == False):
    print("Unable to read camera feed")

query_img = cv2.imread('/workspaces/Puzzle/src/puzzle_reaction/data/IMG_20210920_201017.jpg', cv2.IMREAD_COLOR)
try:
    while True:
        frames = cap.read()
        # depth_frame = frames.get_depth_frame()
        # color_frame = frames.get_color_frame()
        # if not depth_frame or not color_frame:
            # continue

        # depth_image = np.asanyarray(depth_frame.get_data())
        # color_image = np.asanyarray(color_frame.get_data())

        t = time.time()
        detect_res_img = feature_matching_detect_obj(query_img, frames)
        t = time.time() - t
        print('Inference time: {:.3f}'.format(t))

        cv2.namedWindow('Detector', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Detector', detect_res_img)
        ch = cv2.waitKey(1)
        if ch & 0xFF == ord('q') or ch == 27:
            break

finally:
    print('Stop streaming ...')
    cap.release()
    cv2.destroyAllWindows()


# Недостатки алгоритма обнаружения объекта (куба) на ключевых точках с использованием гомографии:
# 1. Предназначен больше для плоских объектов (тетради, журналы, книги) -> попробовать на кубе
# 2. Неробастный метод - т.е. он пороговый, зависит от условий освещения,
#    и в целом много ложных срабатываний -> для куба подобрать пороги, придумать эвристики
#    для отсева ложных срабатываний (использовать несколько кадров, ...)
# 3. Не может детектить одновременно несколько объектов (только один) -> ?
# 4. Зависит от позы объекта на шаблоне т.е. для детекции куба надо несколько шаблонов с разными ракурсами,
#    а потом это постпроцессить -> постпроцессинг
# 5. Из п.4 следует, что алгоритм нужно прогонять для каждого кадра несколько раз (по количеству шаблонов),
#    вычислительные затраты возрастут в разы (при том что алгоритм сам по себе вычислительно затратный -
#    извлечение фич, knn, гомография) -> можно попробовать что-то оптимизировать


# https://docs.opencv.org/4.5.1/d7/dff/tutorial_feature_homography.html
# https://github.com/mtszkw/matching/blob/master/matching/run_matching.py
# http://amroamroamro.github.io/mexopencv/opencv/feature_homography_demo.html
# https://github.com/GigaFlopsis/image_pose_estimation/tree/master/src
# https://docs.opencv.org/master/dc/d2c/tutorial_real_time_pose.html
# https://docs.opencv.org/master/d7/d53/tutorial_py_pose.html
# 
# solvePnP, solvePnPRansac, projectPoints, findFundamentalMat
#
# https://overcoder.net/q/77430/%D0%BF%D0%BE%D0%BB%D0%BE%D0%B6%D0%B5%D0%BD%D0%B8%D0%B5-%D0%BA%D0%B0%D0%BC%D0%B5%D1%80%D1%8B-%D0%B2-%D0%BC%D0%B8%D1%80%D0%BE%D0%B2%D0%BE%D0%B9-%D0%BA%D0%BE%D0%BE%D1%80%D0%B4%D0%B8%D0%BD%D0%B0%D1%82%D0%B5-%D0%BE%D1%82-cv-solvepnp
# https://learnopencv.com/head-pose-estimation-using-opencv-and-dlib/
#

