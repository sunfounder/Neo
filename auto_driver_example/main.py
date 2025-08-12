import sys
import time
import traceback

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

import mediapipe as mp

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

model = "../vision_models/efficientdet_lite0.tflite"
labels = "../vision_models/object_detection_labelmap.txt"
SCORE_THRESHOLD = 0.5
MAX_RESULTS = 15

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
        [-1000, 550],       # left bottom
        [2000, 550],        # right bottom
        [camera_w, 246],    # right top
        [0, 246]            # left top
    ])

    dst = np.float32([
        [0, camera_h-50],          # left bottom
        [camera_w, camera_h-50],   # right bottom
        [camera_w, 0],             # right top
        [0, 0]                     # left top
    ])

    # Calculate the perspective transformation matrix M
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)

    # Generate a bird's-eye view
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
    # adaptive theshold must be odd
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
    kernel = np.ones((15, 15), np.uint8)
    opening = cv2.morphologyEx(adaptive_theshold, cv2.MORPH_OPEN, kernel, iterations=1)
    # cv2.imshow('Opening', opening)

    return opening

# compute_offset_from_center
# ==============================================================
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

    offset_pix = lane_midpoint - frame_width / 2 # Positive - right, Negative - left
    offset_cm = xm_per_pix * offset_pix

    return offset_cm, int(lane_midpoint)

# blend images to output the final image
# ==============================================================
def create_gray_blur_image(width=500, height=500, gray_value=127, ksize=(15, 15), sigma_x=0):
    """
    Create a grayscale image and add Gaussian blur
    
    Parameters:
        width: Image width in pixels
        height: Image height in pixels
        gray_value: Grayscale value (0-255, where 0 is pure black and 255 is pure white)
        ksize: Gaussian kernel size, must be a tuple of positive odd numbers (e.g., (3,3), (15,15)). Larger values result in more blur
        sigma_x: Gaussian standard deviation in X direction. If 0, it's calculated automatically based on kernel size
    Returns:
        The processed image
    """
    # Create a single-channel grayscale image (initialized with specified grayscale value)
    gray_img = np.ones((height, width), dtype=np.uint8) * gray_value
    # Gaussian blur
    blurred_img = cv2.GaussianBlur(gray_img, ksize, sigma_x)

    #
    img = cv2.cvtColor(blurred_img, cv2.COLOR_GRAY2BGR)
    return img

_top_bkg = create_gray_blur_image(width=camera_w, height=174, gray_value=127, ksize=(5, 5), sigma_x=0)

def prepare_out_blend_frame(blend_on_road, img_binary, img_birdeye, img_fit, curvature_cm, offset_cm, steer):
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

    # add _top_bkg
    blend_on_road = cv2.vconcat([_top_bkg, blend_on_road])
    
    # rgb
    thumb_binary = cv2.resize(img_binary, dsize=(thumb_w, thumb_h))
    blend_on_road[off_y:thumb_h+off_y, off_x:off_x+thumb_w, :] = thumb_binary

    # add thumbnail of bird's eye view
    thumb_birdeye = cv2.resize(img_birdeye, dsize=(thumb_w, thumb_h))
    thumb_birdeye = np.dstack([thumb_birdeye, thumb_birdeye, thumb_birdeye])
    blend_on_road[off_y:thumb_h+off_y, 2*off_x+thumb_w:2*(off_x+thumb_w), :] = thumb_birdeye

    # add thumbnail of bird's eye view (lane-line highlighted)
    thumb_img_fit = cv2.resize(img_fit, 
                               dsize=(thumb_w, thumb_h), 
                               interpolation=cv2.INTER_AREA # Area interpolation, details better but slower than bilinear interpolation
                               )
    blend_on_road[off_y:thumb_h+off_y, 3*off_x+2*thumb_w:3*(off_x+thumb_w), :] = thumb_img_fit

    # add text (curvature and offset info) on the upper right of the blend
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(blend_on_road, 'radius: {:.02f}cm'.format(curvature_cm), (w-250, 60), font, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(blend_on_road, 'Offset: {:.02f}cm'.format(offset_cm), (w-250, 100), font, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(blend_on_road, 'steer: {:.02f}deg'.format(steer), (w-250, 140), font, 0.8, (255, 255, 255), 2, cv2.LINE_AA)

    return blend_on_road


# init camera
# ==============================================================
def camera_init(main_size=(camera_w, camera_h), lores_size=None, hflip=False, vflip=False):
    import os
    # set libcamera2 log level
    os.environ['LIBCAMERA_LOG_LEVELS'] = '*:ERROR'
    from picamera2 import Picamera2
    import libcamera

    picam2 = Picamera2()

    preview_config = picam2.preview_configuration
    preview_config.main = {'size': main_size, 'format': 'XRGB8888'}
    if lores_size is not None:
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

def read_camera_calibration(file_path):
    import pickle

    with open(file_path, 'rb') as dump_file:
        calibration = pickle.load(dump_file)

    ret, mtx, dist, rvecs, tvecs = calibration
    return ret, mtx, dist, rvecs, tvecs

# init mediapipe object detector
# ==============================================================
detection_result_list = []
obj_det_fps = 0
obj_det_frame_count = 0
obj_det_fps_start_time = time.time()

def mediapipe_objiect_detector_init():
    from mediapipe.tasks import python
    from mediapipe.tasks.python import vision

    def save_result(result: vision.ObjectDetectorResult, output_image: mp.Image=None, timestamp_ms: int=None):
        global detection_result_list
        global obj_det_fps, obj_det_frame_count, obj_det_fps_start_time

        if obj_det_frame_count % 10 == 0:
            obj_det_fps = int(10 / (time.time() - obj_det_fps_start_time))
            obj_det_fps_start_time = time.time()

        detection_result_list.append(result)
        obj_det_frame_count += 1

    options = vision.ObjectDetectorOptions(
        base_options=python.BaseOptions(model_asset_path=model),
        running_mode=vision.RunningMode.LIVE_STREAM,
        max_results=MAX_RESULTS,
        score_threshold=SCORE_THRESHOLD,
        result_callback=save_result,
    )

    detector = vision.ObjectDetector.create_from_options(options)

    return detector


colors = [(0,255,255),(255,0,0),(0,255,64),(255,255,0),
        (255,128,64),(128,128,255),(255,128,255),(255,128,128)]
MARGIN = 10
ROW_SIZE = 15
FONT_SIZE = 0.8
FONT_THICKNESS = 1

def extract_draw_objects(frame, detections):

    is_safety = True
    if detections is None:
        return frame, is_safety

    i = 0
    for detection in detections:
        # Draw bounding_box
        bbox = detection.bounding_box
        start_point = bbox.origin_x, bbox.origin_y
        end_point = bbox.origin_x + bbox.width, bbox.origin_y + bbox.height
        color = colors[i % len(colors)]
        i += 1
        cv2.rectangle(frame, start_point, end_point, color, 2)

        # Draw label and score
        label = detection.categories[0].category_name
        score = detection.categories[0].score
        label_text = f'{label} {int(score * 100)}%'
        cv2.putText(frame,
                    label_text,
                    (MARGIN + bbox.origin_x, MARGIN + ROW_SIZE + bbox.origin_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    FONT_SIZE,
                    color,
                    FONT_THICKNESS,
                    cv2.LINE_AA)
        
        if label == 'stop sign':
            is_safety = False
            # area = bbox.width * bbox.height
            # if area > 4500:
            #     is_safety = False

        if label == 'person':
            is_safety = False
            # area = bbox.width * bbox.height
            # if area > 8000:
            #     is_safety = False

        if label == 'traffic light':
            is_safety = False
            # area = bbox.width * bbox.height
            # if area > 4000:
            #     is_safety = False

    return frame, is_safety

# move handler
# ==============================================================
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

            # _kk = 30
            # if _curvature == 0:
            #     _steer = 0
            # else:
            #     _steer = OUT_MAX_ANGLE * _kk* (1/_curvature) 
            # steer = pid.update(_steer)

            steer = pid.update(_offset)

            my_car.move(0, POWER, -steer, drift=False)

        elif _status == 'stop':
            steer = 0
            my_car.stop()
        time.sleep(0.1)

    my_car.stop()
    time.sleep(0.1)

# main
# ==============================================================
picam2 = None
move_thread = None
mode = 'live'

final_output_window = 'Final Output'
cv2.namedWindow(final_output_window, cv2.WINDOW_NORMAL) 
cv2.resizeWindow(final_output_window, 960, 895)
cv2.moveWindow(final_output_window, 600, 100)

def main():
    global move_running, move_status, curvature, offset, thread_lock
    global picam2, move_thread, mode
    global steer

    # -------- init --------
    if len(sys.argv) > 1:
        mode = 'img'
        img_path = sys.argv[1]
    else:
        mode = 'live'

        obj_detector = mediapipe_objiect_detector_init()

        picam2 = camera_init(
                        main_size=(camera_w, camera_h),
                        lores_size=None,
                        hflip=True,
                        vflip=True
                        )
        
        move_thread = threading.Thread(target=move_hander, daemon=True)
        move_running = True
        move_thread.start()

    line_lt, line_rt = Line(buffer_len=8), Line(buffer_len=8)

    st = time.time()
    fps_count = 0
    fps = 0
    results = None

    while True:
        # ------ image input ---------
        if mode == 'img':
            img_path = sys.argv[1]
            img_ori = cv2.imread(img_path)
        elif mode == 'live':
            img_ori = picam2.capture_array('main')

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

        # ------ object detection (async) ------
        rgb_image = cv2.cvtColor(img_ori, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)

        # if Object Detector task is busy, the new input frame will be ignored
        obj_detector.detect_async(mp_image, time.time_ns() // 1_000_000)

        if detection_result_list:
            results = detection_result_list[0].detections
            detection_result_list.clear()
        else:
            # use the previous frame's results
            pass

        img_hailo, is_safety = extract_draw_objects(blend_on_road, results)

        # ------ move ------
        with thread_lock:
            if is_safety:
                move_status = 'run'
            else:
                move_status = 'stop'
            curvature = curvature_cm
            offset = offset_cm

        # ------ blend ------
        blend_output = prepare_out_blend_frame(img_hailo, img_birdeye_ori, img_birdeye_binary, img_fit, curvature_cm, offset_cm, steer)
        # cv2.imshow('blend_output', blend_output)  

        # ------ count fps ------
        fps_count += 1
        if time.time() - st >= 1:
            fps =  int(fps_count / (time.time() - st))
            fps_count = 0
            st = time.time()
            # print(f'fps: {fps}') 


        cv2.putText(blend_output, f'fps: {fps}', (camera_w - 120, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(blend_output, f'obj detector fps: {obj_det_fps}', (camera_w - 320, 180), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        # ------ show ------
        try:
            prop = cv2.getWindowProperty(final_output_window, cv2.WND_PROP_VISIBLE)
            if prop < 1:
                break
        except:
            pass

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
        print(traceback.format_exc())
    finally:
        if mode == 'live':
            print('stop')
            move_running = False
            move_thread.join()
            picam2.close()


            

