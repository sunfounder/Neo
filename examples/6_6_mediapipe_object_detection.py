'''
https://ai.google.dev/edge/mediapipe/solutions/vision/object_detector/python

'''

import cv2
import numpy as np
import time

from picamera2 import Picamera2
import libcamera

import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

# ---------------------------------------------------------------------------
def camera_init(main_size=(800, 600), lores_size=None, hflip=False, vflip=False):
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

# ---------------------------------------------------------------------------
MARGIN = 10  # pixels
ROW_SIZE = 30  # pixels
FONT_SIZE = 1
FONT_THICKNESS = 1

colors = [(0,255,255),(255,0,0),(0,255,64),(255,255,0),
        (255,128,64),(128,128,255),(255,128,255),(255,128,128)]

def visualize(
    image,
    detection_result
) -> np.ndarray:
  """Draws bounding boxes on the input image and return it.
  Args:
    image: The input RGB image.
    detection_result: The list of all "Detection" entities to be visualized.
  Returns:
    Image with bounding boxes.
  """
  i = 0
  for detection in detection_result.detections:
    # Draw bounding_box
    bbox = detection.bounding_box
    start_point = bbox.origin_x, bbox.origin_y
    end_point = bbox.origin_x + bbox.width, bbox.origin_y + bbox.height
    # Use the orange color for high visibility.
    # color = np.random.randint(0, 255, 3, dtype=np.int32)
    color = colors[i % len(colors)]
    i += 1
    cv2.rectangle(image, start_point, end_point, color, 3)

    # Draw label and score
    category = detection.categories[0]
    category_name = category.category_name
    probability = round(category.score, 2)
    result_text = category_name + ' (' + str(probability) + ')'
    text_location = (MARGIN + bbox.origin_x,
                     MARGIN + ROW_SIZE + bbox.origin_y)
    cv2.putText(image, result_text, text_location, cv2.FONT_HERSHEY_DUPLEX,
                FONT_SIZE, color, FONT_THICKNESS, cv2.LINE_AA)

  return image

# ---------------------------------------------------------------------------
fps = 0
frame_count = 0
start_time = time.time()
FPS_AVG_FRAME_COUNT = 10

detection_frame = None
detection_result_list = []

def save_result(result: vision.ObjectDetectorResult, output_image: mp.Image=None, timestamp_ms: int=None):
    global fps, frame_count, start_time
    global detection_frame, detection_result_list

    if frame_count % FPS_AVG_FRAME_COUNT == 0:
        fps = FPS_AVG_FRAME_COUNT / (time.time() - start_time)
        start_time = time.time()

    detection_result_list.append(result)
    frame_count += 1

# ---------------------------------------------------------------------------
model = "../vision_models/efficientdet_lite0.tflite"
labels = "../vision_models/object_detection_labelmap.txt"

CAMERA_WIDTH = 800
CAMERA_HEIGHT = 600
HFLIP = True
VFLIP = True

IS_ASYNC = True

SCORE_THRESHOLD = 0.35
MAX_RESULTS = 15

# fps display
row_size = 38  # pixels
left_margin = CAMERA_WIDTH - 164  # pixels
text_color = (255, 255, 255)  # white
font_size = 1
font_thickness = 1

def main():
    global detection_frame, detection_result_list

    print(f'ASYNC:{IS_ASYNC}')
    
    # Initialize Picamera2.
    picam2 = camera_init(main_size=(CAMERA_WIDTH, CAMERA_HEIGHT), hflip=HFLIP, vflip=VFLIP)

    # Initialize MediaPipe Objectron.
    if IS_ASYNC:
        # async
        options = vision.ObjectDetectorOptions(
            base_options=python.BaseOptions(model_asset_path=model),
            running_mode=vision.RunningMode.LIVE_STREAM,
            max_results=MAX_RESULTS,
            score_threshold=SCORE_THRESHOLD,
            result_callback=save_result,
        )
    else:
        # sync
        options = vision.ObjectDetectorOptions(
            base_options=python.BaseOptions(model_asset_path=model),
            running_mode=vision.RunningMode.IMAGE,
            max_results=MAX_RESULTS,
            score_threshold=SCORE_THRESHOLD
        )
        
    detector = vision.ObjectDetector.create_from_options(options)

    # loop.
    while True:
        # Capture image
        image = picam2.capture_array()

        # Convert the image from BGR to RGB as required by the TFLite model.
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)

        # Run object detection.
        if IS_ASYNC:
            # async
            # if Object Detector task is busy, the new input frame will be ignored
            detector.detect_async(mp_image, time.time_ns() // 1_000_000)
        else:
            # block
            detection_result = detector.detect(mp_image)
            save_result(detection_result)

        # Show the FPS
        fps_text = f'FPS:{fps:.1f}'
        text_location = (left_margin, row_size)
        cv2.putText(image,
                    fps_text,
                    text_location,
                    cv2.FONT_HERSHEY_DUPLEX,
                    font_size,
                    text_color,
                    font_thickness,
                    cv2.LINE_AA
                    )

        # Draw bounding boxes on the input image and return it.
        if detection_result_list:
            # print(detection_result_list)
            image = visualize(image, detection_result_list[0])
            detection_result_list.clear()

        
        # Display image
        try:
            prop = cv2.getWindowProperty("Object Detection", cv2.WND_PROP_VISIBLE)
            if prop < 1:
                break
        except:
            pass
            
        cv2.imshow('Object Detection', image)
        if cv2.waitKey(1) == ord('q'):
            break

    detector.close()
    picam2.stop()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()