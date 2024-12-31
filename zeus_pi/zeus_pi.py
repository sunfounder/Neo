
from robot_hat import Pin, ADC, PWM, Servo, Motor, Music
from robot_hat import Grayscale_Module, Ultrasonic, utils
from robot_hat import Config
from .rgb_strip import RGB_Strip
from .sh3001 import SH3001
from .compass import Compass
from .utils import *

from math import pi, sqrt, sin, cos
import time
import ast

class ZeusPi():

    CONFIG = f'/opt/zeus-pi/zeus-pi.config'

    CONFIG_DESCRIPTION = '''
  This the offset file is used to calibrate the ZeusPi robot.
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

    ROT_MAX_POWER_PERCENT = 0.5

    DEFAULT_RGB_CONFIG = {
        'rgb_led_count': 16,
        'rgb_color': '#0a000a',
        'rgb_brightness': 100,
        'rgb_style': 'flow',
        'rgb_speed':100,
    }


    DEFAULT_MOTORS_DIRECTION = [1, 1, 0, 0] # Note: in python, 0 and None is False, 1, -1 etc. is True

    DEFAULT_LINE_REFERENCE = [1000, 1000, 1000]
    DEFAULT_CLIFF_REFERENCE = [200, 200, 200]

    CAM_PAN_DIR= -1
    CAM_TILT_DIR = 1
    CAM_PAN_MIN = -90
    CAM_PAN_MAX = 90
    CAM_TILT_MIN = -55
    CAM_TILT_MAX = 90

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

        # --------- config_flie ---------
        self.config = Config(path=config,
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
            self.ir_obstacle_left = Pin(ir_obstacle_pins[0], mode=Pin.IN, pull=Pin.PULL_DOWN)
            self.ir_obstacle_right = Pin(ir_obstacle_pins[1], mode=Pin.IN, pull=Pin.PULL_DOWN)
            debug("ok")
        except Exception as e:
            error("fail")
            error(e)

        # --------- imu sh3001 init ---------
        try:
            debug("imu sh3001 init ... ", end='', flush=True)
            self.imu = SH3001(db=config)
            debug("ok")
        except Exception as e:
            error("fail")
            error(e)

        # --------- geomagnetism qmc6310 init ---------
        try:
            debug("compass qmc6310 init ... ", end='', flush=True)
            self.compass_placement = compass_placement
            self.compass = Compass(self.compass_placement)
            self.set_compass_offset(*self.compass_offset)
            self.set_compass_magnetic_declination(self.magnetic_declination)
            debug("ok")
        except Exception as e:
            error("fail")
            error(e)

        # --------- ws2812 rgb_LEDs init ---------
        try:
            debug("ws2812 rgb_LEDs init ... ", end='', flush=True)
            self.rgb_config = self.DEFAULT_RGB_CONFIG
            self.rgb_strip = RGB_Strip(self.rgb_config)
            debug("ok")
        except Exception as e:
            error("fail")
            error(e)

        # --------- speaker init ---------
        # try:
        #     debug("speaker init ... ", end='', flush=True)
        #     self.speaker = Music()
        #     debug("ok")
        # except Exception as e:
        #     error("fail")
        #     error(e)

        # --------- microphone check ---------


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

        rot_ratio = rotate_power * self.ROT_MAX_POWER_PERCENT
        max_move_ratio = (100 - rot_ratio) / (sqrt(2))

        move_ratio = (move_power / 100) * max_move_ratio
        if (move_ratio > max_move_ratio):
            move_ratio = max_move_ratio
        
        if drift:
            power_0 = (sin(rad) + cos(rad)) * move_ratio
            power_1 = (sin(rad) - cos(rad)) * move_ratio + rot_ratio * 2
            power_2 = (sin(rad) + cos(rad)) * move_ratio - rot_ratio * 2
            power_3 = (sin(rad) - cos(rad)) * move_ratio
        else:
            power_0 = (sin(rad) + cos(rad)) * move_ratio + rot_ratio
            power_1 = (sin(rad) - cos(rad)) * move_ratio + rot_ratio
            power_2 = (sin(rad) + cos(rad)) * move_ratio - rot_ratio
            power_3 = (sin(rad) - cos(rad)) * move_ratio - rot_ratio

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
        self.set_cam_pan(0)
        self.set_cam_tilt(0)

    # compass
    # ===============================================================================
    def read_compass(self):
        return self.compass.read()
    
    def set_compass_offset(self, x_min, x_max, y_min, y_max, z_min, z_max):
        self.compass_offset = [x_min, x_max, y_min, y_max, z_min, z_max]
        self.compass_offset = [round(x, 2) for x in self.compass_offset]
        self.config['compass']['offset'] = self.compass_offset
        self.compass.set_calibration(*self.compass_offset)

    def set_compass_magnetic_declination(self, declination):
        self.compass.set_magnetic_declination(declination)
        self.config['compass']['magnetic_declination'] = declination


    # ir_obstacle
    # ===============================================================================
    def read_ir_obstacle(self):
        val_l = self.ir_obstacle_left.value()
        val_r = self.ir_obstacle_right.value()
        return [val_l, val_r]
    
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
