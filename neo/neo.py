
from robot_hat import Pin, ADC, PWM, Servo, Motor, Music
from robot_hat import Grayscale_Module, Ultrasonic, utils
from robot_hat import Config
from .rgb_strip import RGB_Strip
from .sh3001 import SH3001
from .compass import Compass
from .pid import PID
from .utils import *

import os
from math import pi, sqrt, sin, cos
import time
import ast
import traceback

import imufusion   # https://github.com/xioTechnologies/Fusion
import numpy as np

from multiprocessing import Process, Value, Array


class Neo():

    CONFIG = f'/opt/zeus-pi/zeus-pi.config'

    CONFIG_DESCRIPTION = '''
  This the offset file is used to calibrate the Neo robot.
'''

    '''
    M3 --!!!-- M0
       |     |
       |     |
    M2 ------- M1

    '''
    M0_A_PIN = 'P12'
    M0_B_PIN = 'P13'
    M1_A_PIN = 'P14'
    M1_B_PIN = 'P15'
    M2_A_PIN = 'P16'
    M2_B_PIN = 'P17'
    M3_A_PIN = 'P18'
    M3_B_PIN = 'P19'

    CAM_PAN_PIN = 'P0'
    CAM_TILT_PIN = 'P1'

    GRAYSCALE_L_PIN = 'A0'
    GRAYSCALE_M_PIN = 'A1'
    GRAYSCALE_R_PIN = 'A2'

    ULTRASONIC_TRIG_PIN = 'D0'
    ULTRASONIC_ECHO_PIN = 'D1'

    IR_OBSTACLE_L_PIN = 'D2'
    IR_OBSTACLE_R_PIN = 'D3'

    COMPASS_PLACEMENT = ['x', 'y', 'z']

    ROTATE_RATIO = 0.5

    RGB_LEDS_NUM = 16

    DEFAULT_MOTORS_DIRECTION = [1, 1, 0, 0] # Note: in python, 0 and None is False, 1, -1 etc. is True

    DEFAULT_LINE_REFERENCE = [1000, 1000, 1000]
    DEFAULT_CLIFF_REFERENCE = [200, 200, 200]

    CAM_PAN_DIR= -1
    CAM_TILT_DIR = 1
    CAM_PAN_MIN = -90
    CAM_PAN_MAX = 90
    CAM_TILT_MIN = -55
    CAM_TILT_MAX = 90

    MOVE_PID_KP = 0.8
    MOVE_PID_KI = 0.0
    MOVE_PID_KD = 20.0
    MOVE_PID_OUTPUT_MAX = 100

    MOVE_ERROR_IGNORE = 1 # degrees

    ACC_RANGE = 2 # 2G, 4G, 8G, 16G
    GROYTY_RANGE = 2000 # 125dps, 250dps, 500dps, 1000dps, 2000dps

    IMU_FUSION_SAMPLE_RATE = 200 # Hz
    IMU_FUSION_SETTINGS = [
        imufusion.CONVENTION_NWU,  # convention
        0.5,  # gain
        2000,  # gyroscope range
        0,  # acceleration rejection
        0,  # magnetic rejection
        2* IMU_FUSION_SAMPLE_RATE,  # recovery trigger period = 2 seconds
    ]

    def __init__(self,
                motor_pins:list=[M0_A_PIN, M0_B_PIN, M1_A_PIN, M1_B_PIN, M2_A_PIN, M2_B_PIN, M3_A_PIN, M3_B_PIN],
                servo_pins:list=[CAM_PAN_PIN, CAM_TILT_PIN],
                grayscale_pins:list=[GRAYSCALE_L_PIN, GRAYSCALE_M_PIN, GRAYSCALE_R_PIN],
                ultrasonic_pins:list=[ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN],
                ir_obstacle_pins:list=[IR_OBSTACLE_L_PIN, IR_OBSTACLE_R_PIN],
                compass_placement:list=COMPASS_PLACEMENT,
                config:str=CONFIG,
                ):

        # reset robot_hat
        utils.reset_mcu()
        time.sleep(0.2)

        # --------- variables ---------
        self.motors_speed = [0, 0, 0, 0]
        self.origin_heading = 0

        self.imu_fusion_process = None
        self.imu_fusion_run = False
        self.gyro_raw = Array('d', 3)
        self.mag_raw = Array('d', 3)
        self.acc = Array('d', 3)
        self.gyro = Array('d', 3)
        self.mag = Array('d', 3)
        self._imufusion_timeout_cnt = Value('I', 0) # unsigned int
        self._imufusion_take = Value('d', 0)
        self.roll = Value('f', 0.0) # float
        self.pitch = Value('f', 0.0)
        self.yaw = Value('f', 0.0)

        # --------- config_flie ---------
        self.config = Config(path=config,
                             mode=0o754,
                             owner=os.getlogin(),
                             description=self.CONFIG_DESCRIPTION
                             )
        self.motors_direction = ast.literal_eval(
                self.config.get('motors', 'direction', self.DEFAULT_MOTORS_DIRECTION))
        self.cam_pan_offset, self.cam_tilt_offset = ast.literal_eval(
                self.config.get('servos', 'offset', '[0, 0]'))
        self.line_reference = ast.literal_eval(
                self.config.get('grayscale', 'line_reference', self.DEFAULT_LINE_REFERENCE))
        self.cliff_reference = ast.literal_eval(
                self.config.get('grayscale', 'cliff_reference', self.DEFAULT_CLIFF_REFERENCE))
        self.compass_offset = ast.literal_eval(
                self.config.get('compass', 'offset', f'{[0 for _ in range(6)]}'))
        self.magnetic_declination = str(self.config.get('compass', 'magnetic_declination', f"0°0'E"))

        # write default config if not exist
        self.config.write()
        #
        debug(f'motors_direction: {self.motors_direction}')
        debug(f'servos_offset: {self.cam_pan_offset, self.cam_tilt_offset}')
        debug(f'line_reference: {self.line_reference}')
        debug(f'cliff_reference: {self.cliff_reference}')
        debug(f'compass_offset: {self.compass_offset}')
        debug(f'magnetic_declination: {self.magnetic_declination}')

        # --------- motors init ---------
        try:
            debug("motors init ... ", end='', flush=True)
            # init
            self.motor_0 = Motor(PWM(motor_pins[0]), PWM(motor_pins[1]), is_reversed=self.motors_direction[0])
            self.motor_1 = Motor(PWM(motor_pins[2]), PWM(motor_pins[3]), is_reversed=self.motors_direction[1])
            self.motor_2 = Motor(PWM(motor_pins[4]), PWM(motor_pins[5]), is_reversed=self.motors_direction[2])
            self.motor_3 = Motor(PWM(motor_pins[6]), PWM(motor_pins[7]), is_reversed=self.motors_direction[3])
            # done
            debug("ok")
        except Exception as e:
            error('fail')
            error(e)

        # --------- servos init ---------
        try:
            debug("servos init ... ", end='', flush=True)
            # init
            self.cam_pan = Servo(servo_pins[0])
            self.cam_tilt = Servo(servo_pins[1])
            # done
            debug("ok")
        except Exception as e:
            error('fail')
            error(e)

        # --------- grayscale module init ---------
        try:
            debug("grayscale module init ... ", end='', flush=True)
            # init
            self.adc0, self.adc1, self.adc2 = [ADC(pin) for pin in grayscale_pins]
            self.grayscale = Grayscale_Module(self.adc0, self.adc1, self.adc2, reference=None)
            # done
            debug("ok")
        except Exception as e:
            error('fail')
            error(e)

        # --------- ultrasonic init ---------
        try:
            debug("ultrasonic init ... ", end='', flush=True)
            trig = Pin(ultrasonic_pins[0], mode=Pin.OUT)
            echo = Pin(ultrasonic_pins[1], mode=Pin.IN, pull=Pin.PULL_DOWN)
            self.ultrasonic = Ultrasonic(trig, echo)
            debug("ok")
        except Exception as e:
            error("fail")
            error(e)

        # --------- ir obstacle init ---------
        try:
            debug("ir obstacle init ... ", end='', flush=True)
            self.ir_obstacle_left = Pin(ir_obstacle_pins[0], mode=Pin.IN, pull=Pin.PULL_NONE, active_state=True)
            self.ir_obstacle_right = Pin(ir_obstacle_pins[1], mode=Pin.IN, pull=Pin.PULL_NONE, active_state=True)
            debug("ok")
        except Exception as e:
            error("fail")
            error(e)

        # --------- imu sh3001 init ---------
        try:
            debug("imu sh3001 init ... ", end='', flush=True)
            self.acc_raw = Array('f', 3)
            self.gyro_raw = Array('f', 3)
            self.imu = SH3001(acc_range=self.ACC_RANGE, gryo_range=self.GROYTY_RANGE, db=config)
            debug("ok")
            debug(f"acc_offset: {self.imu.acc_offset}")
            debug(f"gyro_offset: {self.imu.gyro_offset}")
        except Exception as e:
            error("fail")
            error(e)

        # --------- geomagnetism qmc6310 init ---------
        try:
            debug("compass qmc6310 init ... ", end='', flush=True)
            self.compass_placement = compass_placement
            self.compass = Compass(placement=self.compass_placement,
                                   offset=self.compass_offset,
                                   declination=self.magnetic_declination
                                   )
            debug("ok")
            debug(f"compass offset: {self.compass.x_offset} {self.compass.y_offset} {self.compass.z_offset}"
                  f" scale: {self.compass.x_scale} {self.compass.y_scale} {self.compass.z_scale}")
        except Exception as e:
            error("fail")
            _track = traceback.format_exc()
            error(_track)

        # --------- ws2812 rgb_LEDs init ---------
        try:
            debug("ws2812 rgb_LEDs init ... ", end='', flush=True)
            self.rgbs = RGB_Strip(self.RGB_LEDS_NUM)
            debug("ok")
        except Exception as e:
            error("fail")
            error(e)

        # --------- speaker init ---------
        try:
            debug("speaker init ... ", end='', flush=True)
            self.speaker = Music()
            self.enable_speaker()
            debug("ok")
        except Exception as e:
            error("fail")
            error(e)

        # --------- microphone check ---------
        try:
            debug("microphone check ... ", end='', flush=True)
            result = utils.run_command("arecord -l |grep sndrpigooglevoi")
            if result != '':
                debug("ok")
            else:
                debug("fail")
        except Exception as e:
            error("fail")
            error(e)

        # --------- pid controller init ---------
        self.move_pid = PID(kp=self.MOVE_PID_KP,
                               ki=self.MOVE_PID_KI,
                               kd=self.MOVE_PID_KD,
                               out_max=self.MOVE_PID_OUTPUT_MAX
                               )

        # self.reset_heading()
        # --- reset motors and servos ---
        self.reset()


    # motors
    # ===============================================================================
    def set_motors_direction(self, motors_direction):
        self.config['motors']['direction'] = list.copy(motors_direction)
        self.motors_direction = list.copy(motors_direction)
        self.motor_0.set_is_reverse(motors_direction[0])
        self.motor_1.set_is_reverse(motors_direction[1])
        self.motor_2.set_is_reverse(motors_direction[2])
        self.motor_3.set_is_reverse(motors_direction[3])

    def set_motors(self, m0_power, m1_power, m2_power, m3_power):
        #
        _ERROR = 20
        _STEP = 10
        _DEALY = 0.005

        # TODO: start power
        last_m0_power, last_m1_power, last_m2_power, last_m3_power = self.motors_speed

        # ---- m0 ----
        if m0_power - last_m0_power > _ERROR:
            for i in range(round(last_m0_power), round(m0_power), _STEP):
                self.motor_0.speed(i)
                time.sleep(_DEALY)
        elif m0_power - last_m0_power < -_ERROR:
            for i in range(round(last_m0_power), round(m0_power), -_STEP):
                self.motor_0.speed(i)
                time.sleep(_DEALY)
        # ---- m1 ----
        if m1_power - last_m1_power > _ERROR:
            for i in range(round(last_m1_power), round(m1_power), _STEP):
                self.motor_1.speed(i)
                time.sleep(_DEALY)
        elif m1_power - last_m1_power < -_ERROR:
            for i in range(round(last_m1_power), round(m1_power), -_STEP):
                self.motor_1.speed(i)
                time.sleep(_DEALY)
        # ---- m2 ----
        if m2_power - last_m2_power > _ERROR:
            for i in range(round(last_m2_power), round(m2_power), _STEP):
                self.motor_2.speed(i)
                time.sleep(_DEALY)
        elif m2_power - last_m2_power < -_ERROR:
            for i in range(round(last_m2_power), round(m2_power), -_STEP):
                self.motor_2.speed(i)
                time.sleep(_DEALY)
        # ---- m3 ----
        if m3_power - last_m3_power > _ERROR:
            for i in range(round(last_m3_power), round(m3_power), _STEP):
                self.motor_3.speed(i)
                time.sleep(_DEALY)
        elif m3_power - last_m3_power < -_ERROR:
            for i in range(round(last_m3_power), round(m3_power), -_STEP):
                self.motor_3.speed(i)
                time.sleep(_DEALY)

        self.motor_0.speed(m0_power)
        self.motor_1.speed(m1_power)
        self.motor_2.speed(m2_power)
        self.motor_3.speed(m3_power)

        self.motors_speed = [m0_power, m1_power, m2_power, m3_power]

    def move(self, angle, move_power, rotate_power=0, drift=False):
        # offset angle as 0 to the front
        angle += 90
        #
        rad = angle * pi / 180

        rotate_power = rotate_power * self.ROTATE_RATIO
        move_power = move_power  / (sqrt(2))
        move_power = move_power * (1 - self.ROTATE_RATIO)

        if drift:
            power_0 = (sin(rad) + cos(rad)) * move_power
            power_1 = (sin(rad) - cos(rad)) * move_power + rotate_power * 2
            power_2 = (sin(rad) + cos(rad)) * move_power - rotate_power * 2
            power_3 = (sin(rad) - cos(rad)) * move_power
        else:
            power_0 = (sin(rad) + cos(rad)) * move_power + rotate_power
            power_1 = (sin(rad) - cos(rad)) * move_power + rotate_power
            power_2 = (sin(rad) + cos(rad)) * move_power - rotate_power
            power_3 = (sin(rad) - cos(rad)) * move_power - rotate_power

        self.set_motors(power_0, power_1, power_2, power_3)

    # TODO:
    # def move(self, angle, move_power, rotate_power, drift=false)
    # def move_field_centric(self, angle, move_power,heading, drift=false, angFlag)
    # rotation direction 

    def stop_motors(self):
        self.motor_0.speed(0)
        self.motor_1.speed(0)
        self.motor_2.speed(0)
        self.motor_3.speed(0)
        self.motors_speed = [0, 0, 0, 0]

    def stop(self):
        self.stop_motors()

    def forward(self, power):
        self.move(0, power, 0)

    def backward(self, power):
        self.move(180, power, 0)

    def turn_left(self, power):
        self.move(0, 0, power)

    def turn_right(self, power):
        self.move(0, 0, -power)


    def move_with_pid(self, angle, move_power, heading=0, drift=False, angle_flag=False):
        '''
        move with pid to keep the car heading

        :param angle, the direction you want the car to move 
        :param move_power, moving power
        :param heading, the car head pointing
        :param drift, Whether it is a drift mode, default flase 
                        true, drift mode, the car body will return to square
                        flase, drift mode, the car body will not return to square
        :param angle_flag, false, angle of view of field center
                            true, the angle of view of the car
        '''
        offset = 0
        rot = 0

        # get the current heading
        _roll, _picth, _yaw = self.read_euler()
        current_heading =  _yaw
        error = current_heading - heading - self.origin_heading
        # convert to -180~180
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        if error > self.MOVE_ERROR_IGNORE or error < -self.MOVE_ERROR_IGNORE:
            offset += self.move_pid.update(error)
            rot += max(-100, min(100, offset))

        # filed centric
        if not angle_flag and (angle != 0 or drift is True):
            angle = angle - current_heading + self.origin_heading

        self.move(angle, move_power, rot, drift)

    def set_move_pid(self, kp=None, ki=None, kd=None, out_max=None):
        if kp is not None:
            self.move_pid.kp = kp
        if ki is not None:
            self.move_pid.ki = ki
        if kd is not None:
            self.move_pid.kd = kd
        if out_max is not None:
            self.move_pid.out_max = out_max

    # servos
    # ===============================================================================
    def set_cam_pan(self, angle):
        angle = constrain(angle, self.CAM_PAN_MIN, self.CAM_PAN_MAX)
        angle = self.CAM_PAN_DIR * (angle + self.cam_pan_offset)
        self.cam_pan.angle(angle)

    def set_cam_tilt(self, angle):
        angle = constrain(angle, self.CAM_TILT_MIN, self.CAM_TILT_MAX)
        angle = self.CAM_TILT_DIR * (angle + self.cam_tilt_offset)
        self.cam_tilt.angle(angle)

    def set_cam_servos_offset(self, offset):
        self.cam_pan_offset = round(offset[0], 1)
        self.cam_tilt_offset = round(offset[1], 1)
        self.config['servos']['offset']  = [self.cam_pan_offset, self.cam_tilt_offset]

    # ----
    def reset(self):
        self.stop_motors()
        # self.set_cam_pan(0)
        # self.set_cam_tilt(0)

    # compass
    # ===============================================================================
    def read_compass_raw(self):
        return self.compass.read_raw()

    def read_compass(self):
        return self.compass.read()
    
    def read_compass_angle(self, filter=False):
        return self.compass.read_angle(filter)
    
    def set_compass_offset(self, x_min, x_max, y_min, y_max, z_min, z_max):
        self.compass_offset = [x_min, x_max, y_min, y_max, z_min, z_max]
        self.compass_offset = [round(x, 2) for x in self.compass_offset]
        self.config['compass']['offset'] = self.compass_offset
        self.compass.set_offset(self.compass_offset)

    def set_compass_magnetic_declination(self, declination):
        self.compass.set_magnetic_declination(declination)
        self.config['compass']['magnetic_declination'] = declination

    def reset_heading(self):
        for _ in range(10):
            self.origin_heading = self.read_compass_angle()


    # imu
    # ===============================================================================
    def start_imu_fusion(self, with_mag=False):
        try:
            info("9D IMU fusion process init ... ", end='\n', flush=True)
            self.imu_fusion_process = Process(target=self.imu_fusion_process_func, args=(with_mag,))
            self.imu_fusion_process.daemon = True
            self.imu_fusion_process.start()

            # TODO: wait for yaw data to stabilize
            _count = 0
            _last_yaw = 0
            info("wait for data to stabilize ... ", end='', flush=True)
            while True:
                _yaw = self.yaw.value
                _error = _yaw - _last_yaw
                if abs(_yaw) > 0.001 and abs(_error) < 0.05:
                    _count += 1
                else:
                    _count = 0
                _last_yaw = _yaw
                print(f'_count: {_count} _yaw:{_yaw:.6f} {_error:.6f}')
                
                if _count > 100:
                    self.origin_heading = _yaw
                    break
                time.sleep(0.01)
            self.imu_fusion_run = True
            info("ok")
        except Exception as e:
            error("fail")
            _track = traceback.format_exc()
            error(_track)

    def stop_imu_fusion(self):
        if self.imu_fusion_process != None:
            print(f'imu_fusion_process_before: {self.imu_fusion_process}')
            self.imu_fusion_process.terminate()
            print(f'imu_fusion_process_after: {self.imu_fusion_process}')
            self.imu_fusion_process = None
        self.imu_fusion_run = False

    def read_imu_raw(self):
        if self.imu_fusion_run:
            return self.acc_raw
        else:
            return self.imu.read_raw()
        return self.imu.read_raw()

    def read_imu(self):
        return self.imu.read()

    def read_euler(self):
        if self.imu_fusion_run:
            return self.roll.value, self.pitch.value, self.yaw.value
        else:
            raise(error("IMU fusion not running"))

    def imu_fusion_process_func(self, with_mag=False):
        # https://github.com/xioTechnologies/Fusion/blob/main/Python/advanced_example.py
        offset = imufusion.Offset(self.IMU_FUSION_SAMPLE_RATE)
        ahrs = imufusion.Ahrs()
        ahrs.settings = imufusion.Settings(*self.IMU_FUSION_SETTINGS)
        dt = 1.0 / self.IMU_FUSION_SAMPLE_RATE
        
        _st = time.time()
        while True:
            self.acc.value, self.gyro.value = self.imu.read()
            acc_x, acc_y, acc_z = self.acc.value
            gyro_x, gyro_y, gyro_z = self.gyro.value
            acc_arry = np.array([acc_x, acc_y, acc_z])
            gyro_arry = np.array([gyro_x, gyro_y, gyro_z])

            gyro_arry = offset.update(gyro_arry)
            if with_mag:
                mag_x, mag_y, mag_z, _angle = self.compass.read()
                mag_arry = np.array([mag_x, mag_y, mag_z])
                ahrs.update(gyro_arry, acc_arry, mag_arry, dt)
            else:
                ahrs.update_no_magnetometer(gyro_arry, acc_arry, dt)

            self.roll.value, self.pitch.value, self.yaw.value = ahrs.quaternion.to_euler()

            # --- delay --
            try:
                time.sleep(dt-(time.time() - _st))
            except:
                self._imufusion_timeout_cnt.value += 1
                error(f'imu_fusion_timeout')

            self._imufusion_take.value = time.time() - _st
            _st = time.time()

    # ir_obstacle
    # ===============================================================================
    def read_ir_obstacle(self):
        val_l = self.ir_obstacle_left.value()
        val_r = self.ir_obstacle_right.value()
        return (val_l, val_r)
    
    # grayscale module
    # ===============================================================================
    def read_grayscale(self):
        g_l, g_m, g_r = self.grayscale.read()
        status = ['space']*3
        # left
        if (g_l < self.cliff_reference[0]):
            status[0] = 'cliff'
        elif (g_l < self.line_reference[0]):
            status[0] = 'inside'
        else:
            status[0] = 'outside'
        # middle
        if (g_m < self.cliff_reference[1]):
            status[1] = 'cliff'
        elif (g_m < self.line_reference[1]):
            status[1] = 'inside'
        else:
            status[1] = 'outside'
        # right
        if (g_r < self.cliff_reference[2]):
            status[2] = 'cliff'
        elif (g_r < self.line_reference[2]):
            status[2] = 'inside'
        else:
            status[2] = 'outside'

        return ([g_l, g_m, g_r], status)
    
    def set_line_reference(self, reference):
        if isinstance(reference, list) and len(reference) == 3:
            self.line_reference = list.copy(reference)
        else:
            raise TypeError("reference parameter must be 1*3 list.")

    def set_cliff_reference(self, reference):
        if isinstance(reference, list) and len(reference) == 3:
            self.cliff_reference = list.copy(reference)
        else:
            raise TypeError("reference parameter must be 1*3 list.")

    # speaker
    # ===============================================================================
    def enable_speaker(self):
        utils.enable_speaker()

    def disable_speaker(self):
        utils.disable_speaker()

