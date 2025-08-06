import sys
import time
import threading
import random
import queue

import cv2
import numpy as np
from line_utils import Line, \
    get_fits_by_sliding_windows, \
    get_fits_by_previous_fits, \
    draw_back_onto_the_road
from globals import ym_per_pix, xm_per_pix

from neo import Neo

my_car = Neo()
pan_angle = 0
tilt_angle = -30.0
pan_angle_threshold = int(pan_angle*5 )
tilt_angle_threshold = int(tilt_angle*5)
 
my_car.set_cam_pan(10)
my_car.set_cam_tilt(10)
time.sleep(.5)

my_car.set_cam_pan(pan_angle)
my_car.set_cam_tilt(tilt_angle)

# https://github.com/ndrplz/self-driving-car
# https://markhedleyjones.com/projects/calibration-checkerboard-collection

model = "/usr/share/hailo-models/yolov8s_h8l.hef"
labels = "coco.txt"
score_threshold = 0.5

camera_w = 960
camera_h = 720

# Threshold Adjuster
# ==============================================================

threshold_window = 'Threshold Adjuster'
cv2.namedWindow(threshold_window, cv2.WINDOW_NORMAL)

adaptiveThreshold_max = 255
adaptiveThreshold_boxsize = 99 # must be odd
adaptiveThreshold_C = 20

# black hsv threshold
black_hsv_low = np.array([78, 0, 0])
black_hsv_high = np.array([255, 255, 78])

# black_hsv_low = np.array([0, 76, 0])
# black_hsv_high = np.array([255, 255, 78])

def on_pan_angle(val):
    global pan_angle
    pan_angle = val / 5.0
    my_car.set_cam_pan(pan_angle)
    print("pan_angle: ", pan_angle)

def on_tilt_angle(val):
    global tilt_angle
    tilt_angle = val / 5.0
    my_car.set_cam_tilt(tilt_angle)
    print("tilt_angle: ", tilt_angle)

def on_adativeThreshold_max(val):
    global adaptiveThreshold_max
    adaptiveThreshold_max = val

def on_adativeThreshold_C(val):
    global adaptiveThreshold_C
    adaptiveThreshold_C = val

def on_adativeThreshold_boxsize(val):
    global adaptiveThreshold_boxsize
    if val % 2 == 0:
        adaptiveThreshold_boxsize = val + 1
    else:
        adaptiveThreshold_boxsize = val
    cv2.setTrackbarPos('adaptiveThreshold_boxsize', threshold_window, adaptiveThreshold_boxsize)

cv2.createTrackbar('pan_angle', threshold_window, pan_angle_threshold, 255, on_pan_angle)
cv2.setTrackbarMin('pan_angle', threshold_window, -255)
cv2.createTrackbar('tilt_angle', threshold_window, tilt_angle_threshold, 255, on_tilt_angle)
cv2.setTrackbarMin('tilt_angle', threshold_window, -255)

cv2.createTrackbar('adaptiveThreshold_max', threshold_window, adaptiveThreshold_max, 255, on_adativeThreshold_max)
cv2.createTrackbar('adaptiveThreshold_boxsize', threshold_window, adaptiveThreshold_boxsize, 255, on_adativeThreshold_boxsize)
cv2.createTrackbar('adaptiveThreshold_C', threshold_window, adaptiveThreshold_C, 255, on_adativeThreshold_C)

cv2.imshow(threshold_window, 0)
cv2.moveWindow(threshold_window, 20, 100)

# birdeye
# ==============================================================
def birdeye(img):
    src = np.float32([
        [-1000, 550],   # 左下
        [2000, 550],   # 右下
        [camera_w, 246],   # 右上
        [0, 246]    # 左上
    ])

    dst = np.float32([
        [0, camera_h-50],   # 左下
        [camera_w, camera_h-50],   # 右下
        [camera_w, 0],     # 右上
        [0, 0]      # 左上
    ])

    # 计算透视变换矩阵M
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)

    # 应用变换生成鸟瞰图
    img = cv2.warpPerspective(
        img, 
        M, 
        (camera_w, camera_h),
        flags=cv2.INTER_LINEAR # 双线性插值
        # borderValue=(171, 184, 159)
        )

    return img, M, Minv

# binarize
# ==============================================================
def binarize(img):
    # gray
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # cv2.imshow('Gray', gray)

    # blur
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # adaptive theshold
    _adaptiveThreshold_boxsize = int(adaptiveThreshold_boxsize)
    if _adaptiveThreshold_boxsize % 2 == 0:
        _adaptiveThreshold_boxsize += 1
    _adaptiveThreshold_C = adaptiveThreshold_C
    adaptive_theshold = cv2.adaptiveThreshold(
                                        blur, 
                                        adaptiveThreshold_max, 
                                        cv2.ADAPTIVE_THRESH_MEAN_C,
                                        cv2.THRESH_BINARY,
                                        _adaptiveThreshold_boxsize,
                                        _adaptiveThreshold_C
                                        )
    # cv2.imshow('Adaptive Threshold', adaptive_theshold)

    _line_lt_start = (0, 0)
    _line_lt_end = (343, 720)
    _line_rt_start = (960, 0)
    _line_rt_end = (600, 720)
    _line_thickness = 80
    cv2.line(adaptive_theshold, _line_lt_start, _line_lt_end, 255, _line_thickness)
    cv2.line(adaptive_theshold, _line_rt_start, _line_rt_end, 255, _line_thickness)
    # cv2.imshow('adaptive_theshold2', adaptive_theshold)

    # reverse
    adaptive_theshold = cv2.bitwise_not(adaptive_theshold)
    # cv2.imshow('adaptive_theshold3', adaptive_theshold)

    # open
    kernel = np.ones(15, 15, np.uint8)
    opening = cv2.morphologyEx(adaptive_theshold, cv2.MORPH_OPEN, kernel, iterations=1)
    # cv2.imshow('Opening', opening)

    return opening


def compute_offset_from_center(line_lt, line_rt, frame_width):
    """
    Compute offset from center of the inferred lane.
    The offset from the lane center can be computed under the hypothesis that the camera is fixed
    and mounted in the midpoint of the car roof. In this case, we can approximate the car's deviation
    from the lane center as the distance between the center of the image and the midpoint at the bottom
    of the image of the two lane-lines detected.

    :param line_lt: detected left lane-line
    :param line_rt: detected right lane-line
    :param frame_width: width of the undistorted frame
    :return: inferred offset
    """
    lane_width_px = 600

    if line_lt.detected:
        line_lt_bottom = np.mean(line_lt.all_x[line_lt.all_y > 0.95 * line_lt.all_y.max()])

    if line_rt.detected:
        line_rt_bottom = np.mean(line_rt.all_x[line_rt.all_y > 0.95 * line_rt.all_y.max()])

    if line_lt.detected and line_rt.detected:
        lane_midpoint = (line_lt_bottom + line_rt_bottom) / 2
    elif line_lt.detected:
        lane_midpoint = line_lt_bottom + lane_width_px / 2
    elif line_rt.detected:
        lane_midpoint = line_rt_bottom - lane_width_px / 2
    else:
        return -255, None # No lane detected

    offset_pix = lane_midpoint - frame_width / 2 # 正值偏右，负值偏左
    offset_cm = xm_per_pix * offset_pix

    return offset_cm, int(lane_midpoint)

def prepare_out_blend_frame(blend_on_road, img_binary, img_birdeye, img_fit, line_lt, line_rt, offset_cm):
    """
    Prepare the final pretty pretty output blend, given all intermediate pipeline images

    :param blend_on_road: color image of lane blend onto the road
    :param img_binary: thresholded binary image
    :param img_birdeye: bird's eye view of the thresholded binary image
    :param img_fit: bird's eye view with detected lane-lines highlighted
    :param line_lt: detected left lane-line
    :param line_rt: detected right lane-line
    :param offset_cm: offset from the center of the lane
    :return: pretty blend with all images and stuff stitched
    """
    h, w = blend_on_road.shape[:2]

    thumb_ratio = 0.2
    thumb_h, thumb_w = int(thumb_ratio * h), int(thumb_ratio * w)

    off_x, off_y = 20, 15

    # add a gray rectangle to highlight the upper area
    mask = blend_on_road.copy()
    mask = cv2.rectangle(mask, pt1=(0, 0), pt2=(w, thumb_h+2*off_y), color=(0, 0, 0), thickness=cv2.FILLED)
    blend_on_road = cv2.addWeighted(src1=mask, alpha=0.2, src2=blend_on_road, beta=0.8, gamma=0)

    # add thumbnail of binary image
    thumb_binary = cv2.resize(img_binary, dsize=(thumb_w, thumb_h))
    thumb_binary = np.dstack([thumb_binary, thumb_binary, thumb_binary])
    blend_on_road[off_y:thumb_h+off_y, off_x:off_x+thumb_w, :] = thumb_binary

    # add thumbnail of bird's eye view
    thumb_birdeye = cv2.resize(img_birdeye, dsize=(thumb_w, thumb_h))
    thumb_birdeye = np.dstack([thumb_birdeye, thumb_birdeye, thumb_birdeye])
    blend_on_road[off_y:thumb_h+off_y, 2*off_x+thumb_w:2*(off_x+thumb_w), :] = thumb_birdeye

    # add thumbnail of bird's eye view (lane-line highlighted)
    thumb_img_fit = cv2.resize(img_fit, 
                               dsize=(thumb_w, thumb_h), 
                               interpolation=cv2.INTER_AREA # 区域插值，能更好的保留细节， 但速度比双线性插值慢
                               )
    blend_on_road[off_y:thumb_h+off_y, 3*off_x+2*thumb_w:3*(off_x+thumb_w), :] = thumb_img_fit

    # add text (curvature and offset info) on the upper right of the blend
    mean_curvature_meter = np.mean([line_lt.curvature_meter, line_rt.curvature_meter])
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(blend_on_road, 'radius: {:.02f}cm'.format(mean_curvature_meter), (w-250, 60), font, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(blend_on_road, 'Offset: {:.02f}cm'.format(offset_cm), (w-250, 130), font, 0.8, (255, 255, 255), 2, cv2.LINE_AA)

    return blend_on_road

def create_gray_blur_image(width=500, height=500, gray_value=127, ksize=(15, 15), sigma_x=0):
    """
    创建灰度图像并添加高斯模糊
    
    参数:
        width: 图像宽度（像素）
        height: 图像高度（像素）
        gray_value: 灰度值（0-255，0为纯黑，255为纯白）
        ksize: 高斯核大小，必须是正的奇数元组（如(3,3)、(15,15)），数值越大模糊越明显
        sigma_x: X方向的高斯标准差，0则根据核大小自动计算
    返回:
        处理后的图像
    """
    # 创建单通道灰度图像（初始为指定灰度值）
    gray_img = np.ones((height, width), dtype=np.uint8) * gray_value
    
    # 添加高斯模糊
    blurred_img = cv2.GaussianBlur(gray_img, ksize, sigma_x)

    #
    img = cv2.cvtColor(blurred_img, cv2.COLOR_GRAY2BGR)
    return img


_top_bkg = create_gray_blur_image(width=camera_w, height=174, gray_value=127, ksize=(5, 5), sigma_x=0)

def prepare_out_blend_frame_2(blend_on_road, img_binary, img_birdeye, img_fit, curvature_cm, offset_cm, steer):
    """
    Prepare the final pretty pretty output blend, given all intermediate pipeline images

    :param blend_on_road: color image of lane blend onto the road
    :param img_binary: thresholded binary image
    :param img_birdeye: bird's eye view of the thresholded binary image
    :param img_fit: bird's eye view with detected lane-lines highlighted
    :param line_lt: detected left lane-line
    :param line_rt: detected right lane-line
    :param offset_cm: offset from the center of the lane
    :return: pretty blend with all images and stuff stitched
    """
    h, w = blend_on_road.shape[:2]

    thumb_ratio = 0.2
    thumb_h, thumb_w = int(thumb_ratio * h), int(thumb_ratio * w)

    off_x, off_y = 20, 15


    #######
    # background = create_gray_blur_image(
    #                             width=w, 
    #                             height=int(thumb_ratio*h), 
    #                             gray_value=127, 
    #                             ksize=(5, 5),
    #                             sigma_x=0)
    # # cv2.imshow('bground', background)
    # # add a gray rectangle 
    # blend_on_road = cv2.vconcat([background, blend_on_road])

    ####
    # top_part = blend_on_road[0:thumb_h+2*off_y, 0:w, :]
    # blurred_top = cv2.GaussianBlur(top_part, ksize=(75, 75), sigmaX=0)
    # cv2.imshow('blurred_tops', blurred_top)

    # blend_on_road[0:thumb_h, 0:w, :] = blurred_top
    # blend_on_road = cv2.vconcat([blurred_top, blend_on_road])

    blend_on_road = cv2.vconcat([_top_bkg, blend_on_road])
    

    # add thumbnail of binary image
    # thumb_binary = cv2.resize(img_binary, dsize=(thumb_w, thumb_h))
    # thumb_binary = np.dstack([thumb_binary, thumb_binary, thumb_binary])
    # blend_on_road[off_y:thumb_h+off_y, off_x:off_x+thumb_w, :] = thumb_binary

    # rgb
    thumb_binary = cv2.resize(img_binary, dsize=(thumb_w, thumb_h))
    blend_on_road[off_y:thumb_h+off_y, off_x:off_x+thumb_w, :] = thumb_binary
    # cv2.addWeighted()

    # add thumbnail of bird's eye view
    thumb_birdeye = cv2.resize(img_birdeye, dsize=(thumb_w, thumb_h))
    thumb_birdeye = np.dstack([thumb_birdeye, thumb_birdeye, thumb_birdeye])
    blend_on_road[off_y:thumb_h+off_y, 2*off_x+thumb_w:2*(off_x+thumb_w), :] = thumb_birdeye

    # add thumbnail of bird's eye view (lane-line highlighted)
    thumb_img_fit = cv2.resize(img_fit, 
                               dsize=(thumb_w, thumb_h), 
                               interpolation=cv2.INTER_AREA # 区域插值，能更好的保留细节， 但速度比双线性插值慢
                               )
    blend_on_road[off_y:thumb_h+off_y, 3*off_x+2*thumb_w:3*(off_x+thumb_w), :] = thumb_img_fit

    # add text (curvature and offset info) on the upper right of the blend
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(blend_on_road, 'radius: {:.02f}cm'.format(curvature_cm), (w-250, 60), font, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(blend_on_road, 'Offset: {:.02f}cm'.format(offset_cm), (w-250, 100), font, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(blend_on_road, 'steer: {:.02f}deg'.format(steer), (w-250, 140), font, 0.8, (255, 255, 255), 2, cv2.LINE_AA)

    return blend_on_road

def camera_init(main_size=(camera_w, camera_h), lores_size=(640, 640), hflip=False, vflip=False):
    import os
    # set libcamera2 log level
    os.environ['LIBCAMERA_LOG_LEVELS'] = '*:ERROR'
    from picamera2 import Picamera2
    import libcamera

    picam2 = Picamera2()

    preview_config = picam2.preview_configuration
    preview_config.main = {'size': main_size, 'format': 'XRGB8888'}
    preview_config.lores = {'size': lores_size, 'format': 'RGB888'}
    preview_config.format = 'RGB888'  # 'XRGB8888', 'XBGR8888', 'RGB888', 'BGR888', 'YUV420'
    preview_config.transform = libcamera.Transform(
                                    hflip=hflip,
                                    vflip=vflip
                                )
    preview_config.colour_space = libcamera.ColorSpace.Sycc()
    preview_config.buffer_count = 4
    preview_config.queue = True  
    preview_config.controls = {'FrameRate': 30} # change picam2.capture_array() takes time

    picam2.start()

    return picam2

def hialo_init():
    from picamera2.devices import Hailo

    hailo = Hailo(model)

    return hailo


def read_camera_calibration(file_path):
    import pickle

    with open(file_path, 'rb') as dump_file:
        calibration = pickle.load(dump_file)

    ret, mtx, dist, rvecs, tvecs = calibration
    return ret, mtx, dist, rvecs, tvecs

def extract_detections(hailo_output, w, h, class_names, threshold=0.5):
    """Extract detections from the HailoRT-postprocess output."""
    results = []
    is_safety = True
    if hailo_output is None:
        return results
    for class_id, detections in enumerate(hailo_output):
        for detection in detections:
            score = detection[4]
            if score >= threshold:
                y0, x0, y1, x1 = detection[:4]
                bbox = (int(x0 * w), int(y0 * h), int(x1 * w), int(y1 * h))
                results.append([class_names[class_id], bbox, score])
                
                if class_names[class_id] == 'stop sign':
                    is_safety = False
                    # area = (x1-x0)*w*(y1-y0)*h
                    # if area > 4500:
                    #     is_safety = False
                if class_names[class_id] == 'person':
                    is_safety = False
                    # area = (x1-x0)*w*(y1-y0)*h
                    # if area > 8000:
                    #     is_safety = False

                if class_names[class_id] == 'traffic light':
                    is_safety = False
                    # area = (x1-x0)*w*(y1-y0)*h
                    # if area > 4000:
                    #     is_safety = False


    return results, is_safety

def draw_objects(frame, detections):
    if detections:
        for class_name, bbox, score in detections:
            x0, y0, x1, y1 = bbox
            label = f"{class_name} %{int(score * 100)}"
            cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 255, 0, 0), 2)
            cv2.putText(frame, label, (x0 + 5, y0 + 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0, 0), 1, cv2.LINE_AA)

    return frame


move_status = 'stop' # run
move_running = False
curvature = 0
offset = 0 # cm
steer = 0
KP = 2
KI = 0.00
KD = 13.5
OUT_MAX_ANGLE = 35
POWER = 45
OFFSET_R = 1.68


thread_lock = threading.Lock()

def move_hander():
    from pid import PID

    global move_status, curvature, offset, steer

    pid = PID(KP, KI, KD, OUT_MAX_ANGLE)

    rot = 0
    _status = 'stop'
    _curvature = 0
    _offset = 0

    while move_running:
        with thread_lock:
            _status = move_status
            _curvature = curvature
            _offset = offset

        if _status == 'run':

            if _offset == -255:
                my_car.stop()
                time.sleep(0.1)
                continue
            # radius < 30cm 时，转弯达到最大？
            # _kk = 30
            # if _curvature == 0:
            #     _steer = 0
            # else:
            #     _steer = OUT_MAX_ANGLE * _kk* (1/_curvature) 
            # steer = pid.update(_steer)

            steer = pid.update(_offset)

            # my_car.move(0, POWER, -steer, drift=False)

        elif _status == 'stop':
            steer = 0
            my_car.stop()
        time.sleep(0.1)

    my_car.stop()
    time.sleep(0.1)


inference_queue = queue.Queue(maxsize=1)  # 限制队列大小，避免内存堆积
result_queue = queue.Queue(maxsize=1)


def async_inference_worker():
    """单线程消费队列，按顺序处理推理"""
    while True:
        try:
            frame = inference_queue.get(timeout=0.1)  # 取出带序号的帧
            results = hailo.run(frame)
            result_queue.put(results)  # 结果带序号返回
            inference_queue.task_done()
        except queue.Empty:
            continue
        except Exception as e:
            print(f"Error during inference: {e}")
            # break
            continue

picam2 = None
hailo = None
detections = None
class_names = None
move_thread = None
mode = 'img'

final_output_window = 'Final Output'
cv2.namedWindow(final_output_window, cv2.WINDOW_NORMAL) 
cv2.resizeWindow(final_output_window, 960, 895)
cv2.moveWindow(final_output_window, 600, 100)

def main():
    global move_running, move_status, curvature, offset, thread_lock
    global picam2, hailo, detections, class_names, move_thread, mode
    global steer

    # -------- init --------
    if len(sys.argv) > 1:
        mode = 'img'
        img_path = sys.argv[1]
    else:
        mode = 'live'

        hailo = hialo_init()
        inference_thread = threading.Thread(target=async_inference_worker, daemon=True)
        inference_thread.start()

        model_h, model_w, _ = hailo.get_input_shape()
        picam2 = camera_init(
                        main_size=(camera_w, camera_h),
                        lores_size=(model_w, model_h),
                        hflip=True,
                        vflip=True
                        )
        with open(labels, 'r', encoding="utf-8") as f:
            class_names = f.read().splitlines()
        
        move_thread = threading.Thread(target=move_hander, daemon=True)
        move_running = True
        move_thread.start()

    line_lt, line_rt = Line(buffer_len=8), Line(buffer_len=8)

    st = time.time()
    fps_count = 0
    fps = 0
    results = None

    # ------ Initiate the object detection inference for the first frame ---
    if mode == 'live':
        # lores = picam2.capture_array('lores')

        img_ori = picam2.capture_array('main')
        lores = cv2.resize(img_ori, (model_w, model_h))

        inference_queue.put(lores)

    while True:
        # ------ image input ---------
        if mode == 'img':
            img_path = sys.argv[1]
            img_ori = cv2.imread(img_path)
        elif mode == 'live':
            img_ori = picam2.capture_array('main')
            # lores = picam2.capture_array('lores')
            # lores = cv2.resize(img_ori, (model_w, model_h))

        # cv2.imshow('img_ori', img_ori)

        # ------ undistort the image ------
        # ret, mtx, dist, rvecs, tvecs = read_camera_calibration('cali/calibration.pckl')
        # img = undistort(img_ori, mtx, dist)

        # ------ birdeye ------
        img_birdeye_ori,  M, Minv  = birdeye(img_ori)
        # cv2.imshow('birdeye_ori', img_birdeye_ori)


        # ------ binarize ------
        img_birdeye_binary = binarize(img_birdeye_ori)
        # cv2.imshow('birdeye_binary', img_birdeye_binary)


        # ------ find and fit the lane ------
        try:
            # if line_lt.detected and line_rt.detected:
            #     line_lt, line_rt, img_fit = get_fits_by_previous_fits(img_birdeye_binary, line_lt, line_rt, verbose=False)
            # else:
            #     line_lt, line_rt, img_fit = get_fits_by_sliding_windows(img_birdeye_binary, line_lt, line_rt, n_windows=10, verbose=False)
   
            line_lt, line_rt, img_fit = get_fits_by_sliding_windows(img_birdeye_binary, line_lt, line_rt, n_windows=10, verbose=True)
            # cv2.imshow('img_fit', img_fit)

            # # compute curvature and offset
            curvature_cm = np.mean([line_lt.curvature_meter, line_rt.curvature_meter])
            offset_cm, lane_midpoint = compute_offset_from_center(line_lt, line_rt, frame_width=img_ori.shape[1])



            # ------ draw lane lines on the original image ------
            blend_on_road = draw_back_onto_the_road(img_ori, Minv, line_lt, line_rt, keep_state=True)
            # cv2.imshow('blend_on_road', blend_on_road)

        except:
            img_fit = np.dstack((img_birdeye_binary, img_birdeye_binary, img_birdeye_binary)) * 255
            blend_on_road = img_ori
            offset_cm = -255
            curvature_cm = -255
    
        # ------ object detection (block) ------
        # results = hailo.run(lores)
        # detections, is_safety = extract_detections(results, camera_w, camera_h, class_names, score_threshold)

        # img_hailo = draw_objects(blend_on_road, detections)
        # cv2.imshow('img_hailo', img_hailo)

        # ------ object detection (async) ------
        if not result_queue.empty():
            results = result_queue.get()
            result_queue.task_done()
        else:
            # use the previous frame's results
            pass

        if not inference_queue.full():
            # lores = picam2.capture_array('lores')
            lores = cv2.resize(img_ori, (model_w, model_h))
            inference_queue.put(lores)

        detections, is_safety = extract_detections(results, camera_w, camera_h, class_names, score_threshold)
        img_hailo = draw_objects(blend_on_road, detections)

        # ------ move ------
        with thread_lock:
            if is_safety:
                move_status = 'run'
            else:
                move_status = 'stop'
            curvature = curvature_cm
            offset = offset_cm

        # ------ blend ------
        blend_output = prepare_out_blend_frame_2(img_hailo, img_birdeye_ori, img_birdeye_binary, img_fit, curvature_cm, offset_cm, steer)
        # cv2.imshow('blend_output', blend_output)  

        # ------ count fps ------
        fps_count += 1
        if time.time() - st >= 1:
            fps =  int(fps_count / (time.time() - st))
            fps_count = 0
            st = time.time()
            # print(f'fps: {fps}') 

        cv2.putText(blend_output, f'fps: {fps}', (camera_w - 120, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.imshow(final_output_window, blend_output)

        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Interrupted')
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if mode == 'live':
            print('stop')
            move_running = False
            move_thread.join()
            picam2.close()


            

