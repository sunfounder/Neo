import numpy as np
import cv2
import glob
import collections
import matplotlib.pyplot as plt
# from calibration_utils import calibrate_camera, undistort
# from binarization_utils import binarize
# from perspective_utils import birdeye
from globals import ym_per_pix, xm_per_pix

# https://blog.csdn.net/qq_28087491/article/details/113724322

height = 720
ploty = np.linspace(0, height - 1, height, dtype=np.float32)


class Line:
    """
    Class to model a lane-line.
    """
    def __init__(self, buffer_len=10):

        # flag to mark if the line was detected the last iteration
        self.detected = False

        # polynomial coefficients fitted on the last iteration
        self.last_fit_pixel = None
        self.last_fit_meter = None

        # list of polynomial coefficients of the last N iterations
        self.recent_fits_pixel = collections.deque(maxlen=buffer_len)
        self.recent_fits_meter = collections.deque(maxlen=2 * buffer_len)

        self.radius_of_curvature = None

        # store all pixels coords (x, y) of line detected
        self.all_x = None
        self.all_y = None

    def update_line(self, new_fit_pixel, new_fit_meter, detected, clear_buffer=False):
        """
        Update Line with new fitted coefficients.

        :param new_fit_pixel: new polynomial coefficients (pixel)
        :param new_fit_meter: new polynomial coefficients (meter)
        :param detected: if the Line was detected or inferred
        :param clear_buffer: if True, reset state
        :return: None
        """
        self.detected = detected

        if clear_buffer:
            self.recent_fits_pixel = []
            self.recent_fits_meter = []

        self.last_fit_pixel = new_fit_pixel
        self.last_fit_meter = new_fit_meter

        self.recent_fits_pixel.append(self.last_fit_pixel)
        self.recent_fits_meter.append(self.last_fit_meter)

    def draw(self, mask, color=(255, 0, 0), line_width=50, average=False):
        """
        Draw the line on a color mask image.
        """
        h, w, c = mask.shape
        coeffs = self.average_fit if average else self.last_fit_pixel

        line_center = coeffs[0] * ploty ** 2 + coeffs[1] * ploty + coeffs[2]
        line_left_side = line_center - line_width // 2
        line_right_side = line_center + line_width // 2

        # Some magic here to recast the x and y points into usable format for cv2.fillPoly()
        pts_left = np.array(list(zip(line_left_side, ploty)))
        pts_right = np.array(np.flipud(list(zip(line_right_side, ploty))))
        pts = np.vstack([pts_left, pts_right])

        # Draw the lane onto the warped blank image
        return cv2.fillPoly(mask, [np.int32(pts)], color)

    @property
    # average of polynomial coefficients of the last N iterations
    def average_fit(self):
        return np.mean(self.recent_fits_pixel, axis=0)

    @property
    # radius of curvature of the line (averaged)
    def curvature(self):
        y_eval = 0
        coeffs = self.average_fit
        return ((1 + (2 * coeffs[0] * y_eval + coeffs[1]) ** 2) ** 1.5) / np.absolute(2 * coeffs[0])

    # @property
    # # radius of curvature of the line (averaged)
    # def curvature_meter(self):
    #     y_eval = 0
    #     coeffs = np.mean(self.recent_fits_meter, axis=0)
    #     print('coeffs', coeffs)
    #     return ((1 + (2 * coeffs[0] * y_eval + coeffs[1]) ** 2) ** 1.5) / np.absolute(2 * coeffs[0])

    @property
    # radius of curvature of the line (averaged)
    def curvature_meter(self):
        y_eval = 0
        coeffs = np.mean(self.recent_fits_meter, axis=0)

        A = coeffs[0]
        B = coeffs[1]

        # print('A', A)

        if abs(A) < 1e-2:  # 阈值可根据实际场景调整（如1e-6 ~ 1e-5）
            return 0.0  # 直线曲率为0
        
        numerator = (1 + (2 * A * y_eval + B) **2)** 1.5
        # denominator = abs(2 * A)
        denominator = 2 * A # 保留符号以区分方向（左/右弯）
        return numerator / denominator
    
def get_fits_by_sliding_windows(birdeye_binary, line_lt, line_rt, n_windows=9, verbose=False):
    """
    Get polynomial coefficients for lane-lines detected in an binary image.

    :param birdeye_binary: input bird's eye view binary image
    :param line_lt: left lane-line previously detected
    :param line_rt: left lane-line previously detected
    :param n_windows: number of sliding windows used to search for the lines
    :param verbose: if True, display intermediate output
    :return: updated lane lines and output image
    """
    height, width = birdeye_binary.shape
   
    # 直方图-统计每一列的像素
    histogram = np.sum(birdeye_binary[30:height//2, :], axis=0)

    # Create an output image to draw on and  visualize the result
    out_img = np.dstack((birdeye_binary, birdeye_binary, birdeye_binary)) * 255

    # 寻找左边和右边像素最多的列, 作为找线的起点
    # Find the peak of the left and right halves of the histogram
    # These will be the starting point for the left and right lines
    midpoint = len(histogram) // 2
    leftx_base = np.argmax(histogram[:midpoint]) 
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    # Set height of windows
    window_height = np.int32(height / n_windows)

    # 寻找所有非零像素, 帮助快速筛选
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = birdeye_binary.nonzero()
    nonzero_y = np.array(nonzero[0])
    nonzero_x = np.array(nonzero[1])

    # Current positions to be updated for each window
    leftx_current = leftx_base
    rightx_current = rightx_base

    lane_width_px = 600 # lane width

    # 滑动窗口宽度/2
    margin = 150 # width of the windows +/- margin
    # 窗口有效阈值-窗口内像素数 >= minpix 才有效
    minpix = 50   # minimum number of pixels found to recenter window
                  # minpix
    
    min_valid_windows = 4  # 至少4个窗口有效, 才认为检测到线

    left_valid_windows = 0
    right_valid_windows = 0

    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []

    # Step through the windows one by one
    n  = 0
    for window in range(n_windows):
        # Identify window boundaries in x and y (and right and left)
        # y
        # 由下到上
        # win_y_low = height - (window + 1) * window_height
        # win_y_high = height - window * window_height

        # 由上到下
        win_y_low = window * window_height
        win_y_high = (window + 1) * window_height
        
        # x
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin

        # Draw the windows on the visualization image
        if n == 0:
            cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (255, 255, 0), 2)
            cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 255), 2)
        else:
            cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 2)
            cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 2)

        # 从所有非零像素中,找出属于当前窗口的像素
        # Identify the nonzero pixels in x and y within the window
        good_left_inds = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) & (nonzero_x >= win_xleft_low)
                          & (nonzero_x < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) & (nonzero_x >= win_xright_low)
                           & (nonzero_x < win_xright_high)).nonzero()[0]

        # Append these indices to the lists
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)

        # 如窗口有效, 根据有效像素的x轴平均坐标, 作为下一个窗口x坐标
        # If you found > minpix pixels, recenter next window on their mean position
        if len(good_left_inds) > minpix:
            leftx_current = np.int32(np.mean(nonzero_x[good_left_inds]))
            left_valid_windows += 1
        if len(good_right_inds) > minpix:
            rightx_current = np.int32(np.mean(nonzero_x[good_right_inds]))
            right_valid_windows += 1

        n += 1
    # Concatenate the arrays of indices
    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    # Extract left and right line pixel positions
    line_lt.all_x, line_lt.all_y = nonzero_x[left_lane_inds], nonzero_y[left_lane_inds]
    line_rt.all_x, line_rt.all_y = nonzero_x[right_lane_inds], nonzero_y[right_lane_inds]


    mid_x = int(width // 2)
    mid_y = int(height // 2)
    left_detected = False
    right_detected = False

    cv2.line(out_img, (mid_x, 0), (mid_x, height), (255, 255, 255), 2)

    # 有效窗口 >= min_valid_windows, 则认为检测到线
    if left_valid_windows >= min_valid_windows:
        # 判断最下边的窗口的像素和的平均x左边，平均x位于于mid_x左边，则认为检测到左线有效
        line_lt_bottom_y = line_lt.all_y.max()
        line_lt_bottom_x = np.mean(line_lt.all_x[line_lt.all_y > 0.95 * line_lt_bottom_y])
        if line_lt_bottom_x > mid_x:
            left_detected = False
        else:
            # left_pixels_left = np.sum(line_lt.all_x < mid_x)
            # left_pixels_right = len(line_lt.all_x) - left_pixels_left
            # if left_pixels_left > left_pixels_right:
            #     left_detected = True
            left_detected = True

    # 有效窗口 >= min_valid_windows, 则认为检测到线
    if right_valid_windows >= min_valid_windows:
        # 判断最下边的窗口的像素和的平均x左边，平均x位于于mid_x右边，则认为检测到右线有效
        line_rt_bottom_y = line_rt.all_y.max()
        line_rt_bottom_x = np.mean(line_rt.all_x[line_rt.all_y > 0.95 * line_rt_bottom_y])
        if line_rt_bottom_x < mid_x:
            right_detected = False
        else:
            # right_pixels_left = np.sum(line_rt.all_x < mid_x)
            # right_pixels_right = len(line_rt.all_x) - right_pixels_left
            # if right_pixels_right > right_pixels_left:
            #     right_detected = True
            right_detected = True

    if left_detected:
        left_fit_pixel = np.polyfit(line_lt.all_y, line_lt.all_x, 2)
        left_fit_meter = np.polyfit(line_lt.all_y * ym_per_pix, line_lt.all_x * xm_per_pix, 2)

    if right_detected:
        right_fit_pixel = np.polyfit(line_rt.all_y, line_rt.all_x, 2)
        right_fit_meter = np.polyfit(line_rt.all_y * ym_per_pix, line_rt.all_x * xm_per_pix, 2)

    if not left_detected:
        if right_detected: # 如果左线没有检测到,但是右线检测到了,则使用右线拟合的参数作为左线拟合的参数
            left_fit_pixel = [
                right_fit_pixel[0],  # A系数保持一致（曲率相同）
                right_fit_pixel[1],  # B系数保持一致（斜率相同）
                right_fit_pixel[2] - lane_width_px  # C系数减去车道宽度（水平偏移）
            ]
            left_fit_meter = [
                right_fit_meter[0],
                right_fit_meter[1],
                right_fit_meter[2] - lane_width_px*xm_per_pix
            ]
        else: # 如果左右线都没有检测到,则使用上一次拟合的参数作为本次拟合的参数
            left_fit_pixel = line_lt.last_fit_pixel
            left_fit_meter = line_lt.last_fit_meter

    if not right_detected:
        if left_detected: # 如果右线没有检测到,但是左线检测到了,则使用左线拟合的参数作为右线拟合的参数
            right_fit_pixel = [
                left_fit_pixel[0],
                left_fit_pixel[1],
                left_fit_pixel[2] + lane_width_px
            ]
            right_fit_meter = [
                left_fit_meter[0],
                left_fit_meter[1],
                left_fit_meter[2] + lane_width_px*xm_per_pix
            ]
        else:
            right_fit_pixel = line_rt.last_fit_pixel
            right_fit_meter = line_rt.last_fit_meter


    line_lt.update_line(left_fit_pixel, left_fit_meter, detected=left_detected)
    line_rt.update_line(right_fit_pixel, right_fit_meter, detected=right_detected)

    # Generate x and y values for plotting
    # ploty = np.linspace(0, height - 1, height)
    left_fitx = left_fit_pixel[0] * ploty ** 2 + left_fit_pixel[1] * ploty + left_fit_pixel[2]
    right_fitx = right_fit_pixel[0] * ploty ** 2 + right_fit_pixel[1] * ploty + right_fit_pixel[2]

    # 标记检测到的像素点
    out_img[nonzero_y[left_lane_inds], nonzero_x[left_lane_inds]] = [255, 0, 0]
    out_img[nonzero_y[right_lane_inds], nonzero_x[right_lane_inds]] = [0, 0, 255]

    # 绘制拟合线
    if verbose:
        points = np.column_stack((left_fitx, ploty)).astype(np.int32)
        cv2.polylines(out_img, [points], isClosed=False, color=(255, 255, 0), thickness=2)
        points = np.column_stack((right_fitx, ploty)).astype(np.int32)
        cv2.polylines(out_img, [points], isClosed=False, color=(0, 255, 255), thickness=2)

    return line_lt, line_rt, out_img


def get_fits_by_previous_fits(birdeye_binary, line_lt, line_rt, verbose=False):
    """
    Get polynomial coefficients for lane-lines detected in an binary image.
    This function starts from previously detected lane-lines to speed-up the search of lane-lines in the current frame.

    :param birdeye_binary: input bird's eye view binary image
    :param line_lt: left lane-line previously detected
    :param line_rt: left lane-line previously detected
    :param verbose: if True, display intermediate output
    :return: updated lane lines and output image
    """

    height, width = birdeye_binary.shape

    left_fit_pixel = line_lt.last_fit_pixel
    right_fit_pixel = line_rt.last_fit_pixel

    nonzero = birdeye_binary.nonzero()
    nonzero_y = np.array(nonzero[0])
    nonzero_x = np.array(nonzero[1])
    margin = 80
    lane_width_px = 600

    # 筛选落在左右线拟合曲线区域上的像素点
    left_lane_inds = (
    (nonzero_x > (left_fit_pixel[0] * (nonzero_y ** 2) + left_fit_pixel[1] * nonzero_y + left_fit_pixel[2] - margin)) & (
    nonzero_x < (left_fit_pixel[0] * (nonzero_y ** 2) + left_fit_pixel[1] * nonzero_y + left_fit_pixel[2] + margin)))
    right_lane_inds = (
    (nonzero_x > (right_fit_pixel[0] * (nonzero_y ** 2) + right_fit_pixel[1] * nonzero_y + right_fit_pixel[2] - margin)) & (
    nonzero_x < (right_fit_pixel[0] * (nonzero_y ** 2) + right_fit_pixel[1] * nonzero_y + right_fit_pixel[2] + margin)))

    # Extract left and right line pixel positions
    line_lt.all_x, line_lt.all_y = nonzero_x[left_lane_inds], nonzero_y[left_lane_inds]
    line_rt.all_x, line_rt.all_y = nonzero_x[right_lane_inds], nonzero_y[right_lane_inds]

    mid_x = int(width // 2)
    mid_y = int(height // 2)
    left_detected = False
    right_detected = False

    if list(line_lt.all_x):
        line_lt_bottom_y = line_lt.all_y.max()
        line_lt_bottom_x = np.mean(line_lt.all_x[line_lt.all_y > 0.95 * line_lt_bottom_y])
        if line_lt_bottom_x > mid_x:
            left_detected = False
        else:
            left_detected = True

    if list(line_rt.all_x):
        line_rt_bottom_y = line_rt.all_y.max()
        line_rt_bottom_x = np.mean(line_rt.all_x[line_rt.all_y > 0.95 * line_rt_bottom_y])
        if line_rt_bottom_x < mid_x:
            right_detected = False
        else:
            right_detected = True

    if left_detected:
        left_fit_pixel = np.polyfit(line_lt.all_y, line_lt.all_x, 2)
        left_fit_meter = np.polyfit(line_lt.all_y * ym_per_pix, line_lt.all_x * xm_per_pix, 2)

    if right_detected:
        right_fit_pixel = np.polyfit(line_rt.all_y, line_rt.all_x, 2)
        right_fit_meter = np.polyfit(line_rt.all_y * ym_per_pix, line_rt.all_x * xm_per_pix, 2)

    if not left_detected:
        if right_detected: # 如果左线没有检测到,但是右线检测到了,则使用右线拟合的参数作为左线拟合的参数
            left_fit_pixel = [
                right_fit_pixel[0],  # A系数保持一致（曲率相同）
                right_fit_pixel[1],  # B系数保持一致（斜率相同）
                right_fit_pixel[2] - lane_width_px  # C系数减去车道宽度（水平偏移）
            ]
            left_fit_meter = [
                right_fit_meter[0],
                right_fit_meter[1],
                right_fit_meter[2] - lane_width_px*xm_per_pix
            ]
        else: # 如果左右线都没有检测到,则使用上一次拟合的参数作为本次拟合的参数
            left_fit_pixel = line_lt.last_fit_pixel
            left_fit_meter = line_lt.last_fit_meter

    if not right_detected:
        if left_detected: # 如果右线没有检测到,但是左线检测到了,则使用左线拟合的参数作为右线拟合的参数
            right_fit_pixel = [
                left_fit_pixel[0],
                left_fit_pixel[1],
                left_fit_pixel[2] + lane_width_px
            ]
            right_fit_meter = [
                left_fit_meter[0],
                left_fit_meter[1],
                left_fit_meter[2] + lane_width_px*xm_per_pix
            ]
        else:
            right_fit_pixel = line_rt.last_fit_pixel
            right_fit_meter = line_rt.last_fit_meter

    line_lt.update_line(left_fit_pixel, left_fit_meter, detected=left_detected)
    line_rt.update_line(right_fit_pixel, right_fit_meter, detected=right_detected)

    # Generate x and y values for plotting
    # ploty = np.linspace(0, height - 1, height)
    left_fitx = left_fit_pixel[0] * ploty ** 2 + left_fit_pixel[1] * ploty + left_fit_pixel[2]
    right_fitx = right_fit_pixel[0] * ploty ** 2 + right_fit_pixel[1] * ploty + right_fit_pixel[2]

    # Create an image to draw on and an image to show the selection window
    img_fit = np.dstack((birdeye_binary, birdeye_binary, birdeye_binary)) * 255
    window_img = np.zeros_like(img_fit)

    # Color in left and right line pixels
    img_fit[nonzero_y[left_lane_inds], nonzero_x[left_lane_inds]] = [255, 0, 0]
    img_fit[nonzero_y[right_lane_inds], nonzero_x[right_lane_inds]] = [0, 0, 255]

    # Generate a polygon to illustrate the search window area
    # And recast the x and y points into usable format for cv2.fillPoly()
    left_line_window1 = np.array([np.transpose(np.vstack([left_fitx - margin, ploty]))])
    left_line_window2 = np.array([np.flipud(np.transpose(np.vstack([left_fitx + margin, ploty])))])
    left_line_pts = np.hstack((left_line_window1, left_line_window2))
    right_line_window1 = np.array([np.transpose(np.vstack([right_fitx - margin, ploty]))])
    right_line_window2 = np.array([np.flipud(np.transpose(np.vstack([right_fitx + margin, ploty])))])
    right_line_pts = np.hstack((right_line_window1, right_line_window2))

    # Draw the lane onto the warped blank image
    cv2.fillPoly(window_img, np.int_([left_line_pts]), (0, 255, 0))
    cv2.fillPoly(window_img, np.int_([right_line_pts]), (0, 255, 0))
    result = cv2.addWeighted(img_fit, 1, window_img, 0.3, 0)

    if verbose:
        points = np.column_stack((left_fitx, ploty)).astype(np.int32)
        cv2.polylines(result, [points], isClosed=False, color=(255, 255, 0), thickness=2)
        points = np.column_stack((right_fitx, ploty)).astype(np.int32)
        cv2.polylines(result, [points], isClosed=False, color=(0, 255, 255), thickness=2)

    return line_lt, line_rt, result

def draw_back_onto_the_road(img_undistorted, zoom_out, Minv, line_lt, line_rt, keep_state):
    """
    Draw both the drivable lane area and the detected lane-lines onto the original (undistorted) frame.
    :param img_undistorted: original undistorted color frame
    :param Minv: (inverse) perspective transform matrix used to re-project on original frame
    :param line_lt: left lane-line previously detected
    :param line_rt: right lane-line previously detected
    :param keep_state: if True, line state is maintained
    :return: color blend
    """
    height, width, _ = img_undistorted.shape

    left_fit = line_lt.average_fit if keep_state else line_lt.last_fit_pixel
    right_fit = line_rt.average_fit if keep_state else line_rt.last_fit_pixel

    left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
    right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]

    # now separately draw solid lines to highlight them
    warp = np.zeros_like(img_undistorted, dtype=np.uint8)
    warp = line_lt.draw(warp, color=(255, 0, 0), average=keep_state)
    warp = line_rt.draw(warp, color=(0, 0, 255), average=keep_state)

    # draw road as green polygon on original frame
    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    pts = np.hstack((pts_left, pts_right))

    if line_lt.detected or line_rt.detected:
        cv2.fillPoly(warp, np.int_([pts]), (0, 255, 0))
    else:
        cv2.fillPoly(warp, np.int_([pts]), (0, 0, 255))

    warp = cv2.warpPerspective(
        warp, Minv, (width, height),
        flags=cv2.INTER_NEAREST
        )
    
    blend_onto_road = cv2.addWeighted(
                        src1=img_undistorted,
                        alpha=1.,
                        src2=warp,
                        beta=0.3,
                        gamma=0.
                        )

    return blend_onto_road

