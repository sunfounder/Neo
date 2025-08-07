import time
import threading

import numpy as np
from math import sin, cos, pi, sqrt

# https://github.com/adafruit/Adafruit_CircuitPython_NeoPixel_SPI
# https://github.com/adafruit/Adafruit_CircuitPython_Pixelbuf/blob/main/adafruit_pixelbuf.py
import board
import neopixel_spi as neopixel
from neopixel_spi import NeoPixel_SPI

# utils
# =================================================================
COLORS = {
    'white':   (255, 255, 255),
    'black':   (0,   0,   0),
    'red':     (255,   0,   0),
    'yellow':  (255, 225,   0),
    'green':   (0, 255,   0),
    'blue':    (0,   0, 255),
    'cyan':    (0, 255, 255),
    'magenta': (255,   0, 255),
    'pink':    (255, 100, 100)
}

def map_value(x, from_min, from_max, to_min, to_max):
    return (x - from_min) * (to_max - to_min) / (from_max - from_min) + to_min

# str or hex, eg: 'ffffff', '#ffffff', '#FFFFFF'
def hex_to_rgb(hex):
    try:
        hex = hex.strip().replace('#', '')
        r = int(hex[0:2], 16)
        g = int(hex[2:4], 16)
        b = int(hex[4:6], 16)
        return [r, g, b]
    except Exception as e:
        print('color parameter error: \n%s' % e)

def hsl_to_rgb( hue, saturation=1, brightness=1):
    hue = hue % 360
    _hi = int((hue/60)%6)
    _f = hue / 60.0 - _hi
    _p = brightness * (1 - saturation)
    _q = brightness * (1 - _f * saturation)
    _t = brightness * (1 - (1 - _f) * saturation)
    
    if _hi == 0:
        _R_val = brightness
        _G_val = _t
        _B_val = _p
    if _hi == 1:
        _R_val = _q
        _G_val = brightness
        _B_val = _p
    if _hi == 2:
        _R_val = _p
        _G_val = brightness
        _B_val = _t
    if _hi == 3:
        _R_val = _p
        _G_val = _q
        _B_val = brightness
    if _hi == 4:
        _R_val = _t
        _G_val = _p
        _B_val = brightness
    if _hi == 5:
        _R_val = brightness
        _G_val = _p
        _B_val = _q
    
    r = int(_R_val * 255)
    g = int(_G_val * 255)
    b = int(_B_val * 255)
    return (r, g, b)

def color_2_tuple(color):
    '''
    convert color to tuple like (255, 255, 255)

    :param color: str, hex, list, tuple, int, eg: 'ffffff', '#ffffff', '#FFFFFF', (255, 255, 255), 0xffffff
    :return: tuple
    '''
    try:
        if isinstance(color, str):
            if color.lower() in COLORS:
                return COLORS[color.lower()]
            elif color.startswith('#') and len(color) == 7:
                return (int(color[1:3], 16), int(color[3:5], 16), int(color[5:7], 16))
            else:
                raise # trigger exception
        elif isinstance(color, list) or isinstance(color, tuple):
            return (color[0], color[1], color[2])
        elif isinstance(color, int):
            return (color >> 16, color >> 8 & 0xff, color & 0xff)
    except:
        raise ValueError('\033[0;31m%s\033[0m'%("Invalid color value."))

# Variables define
# =================================================================
STYLES = [
    'solid', 'breathing', 'flow', 'flow_reverse', 'rainbow', 'rainbow_reverse', 'hue_cycle',

]
class WS2812_SPI(NeoPixel_SPI):

    def __init__(self, led_num):

        spi = board.SPI()
        PIXEL_ORDER = neopixel.GRB
        self.led_num = led_num

        super().__init__(spi,
                        led_num,
                        pixel_order=PIXEL_ORDER,
                        brightness=1.0,
                        auto_write=True
                        )
        time.sleep(0.01)
        self.clear()

    def show(self):
        super().show()

    def clear(self):
        super().fill(0)
        super().show()

    def fill(self, color:str='#000000'):
        super().fill(color)

    def fill_pattern(self, pattern):
        for i in range(self.led_num):
            self[i] = pattern[i]

    def set_brightness(self, value):
        self.brightness = value
    
    def get_brightness(self):
        return self.brightness

#TODO:
#  class RGBStrip_I2C()
#
#

class NeoRGBStrip(WS2812_SPI):

    MIN_DELAY = 0.05
    DEFAULT_BRIGHTNESS = 0.5

    def __init__(self):

        self.running = False
        self._is_ready = False

        self.headlights = {
            'leds': [0, 1, 2, 3, 4, 5, 6, 7],
            'number': 8,
            'style': None,
            'color': (0, 255, 255),
            'bps': 1,
            'brightness': 1,
            'frames': [[[0, 0, 0]] * 8],
            'max_frames': 1,
            'frame_index': 0
        }
        self.tail_lights = {
            'leds': [8, 9, 10, 11, 12, 13, 14, 15],
            'number': 8,
            'style': None,
            'color': (0, 255, 255),
            'bps': 1,
            'brightness': 1,
            'frames': [[[0, 0, 0]] * 8],
            'max_frames': 1,
            'frame_index': 0
        }

        try:
            super().__init__(led_num=self.headlights['number'] + self.tail_lights['number'])
            self.set_brightness(self.DEFAULT_BRIGHTNESS)
            self._is_ready = True
        except Exception as e:
            self._is_ready = False
            raise(e)

    def loop(self):
        self.running = True
        self.counter = 0
        self.counter_max = 100
        if not self._is_ready:
            print("rgb strip not ready")
            return
        while self.running:
            if self.headlights['frame_index'] >= self.headlights['max_frames']:
                self.headlights['frame_index'] = 0
            if self.tail_lights['frame_index'] >= self.tail_lights['max_frames']:
                self.tail_lights['frame_index'] = 0

            headlights_frame = self.headlights['frames'][self.headlights['frame_index']]
            tail_lights_frame = self.tail_lights['frames'][self.tail_lights['frame_index']]
            mix_frames = headlights_frame + tail_lights_frame

            self.headlights['frame_index'] += 1
            self.tail_lights['frame_index'] += 1

            self.fill_pattern(mix_frames)
            self.show()
            time.sleep(self.MIN_DELAY)

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self.loop)
        self.thread.start()

    def stop(self):
        if self.running:
            self.running = False
            self.thread.join()
        self.clear()
        self.show()
        print("WS2812 Stop")

    def close_headlights(self):
        self.headlights['style'] = None
        self.headlights['frames'] = [[[0, 0, 0]] * self.headlights['number']]
        self.headlights['max_frames'] = 1
        self.headlights['frame_index'] = 0
        self.headlights['max_frames'] = 1
        self.headlights['frame_index'] = 0

    def close_tail_lights(self):
        self.tail_lights['style'] = None
        self.tail_lights['frames'] = [[[0, 0, 0]] * self.tail_lights['number']]
        self.tail_lights['max_frames'] = 1
        self.tail_lights['frame_index'] = 0

    def set_headlights_style(self, style, color, bps=1, brightness=1):
        self.headlights['style'] = style
        self.headlights['color'] = color_2_tuple(color)
        self.headlights['bps'] = bps
        self.headlights['brightness'] = brightness
         
        fuc = getattr(self, style)
        self.headlights['frames'] = fuc(self.headlights['leds'], 
                                        self.headlights['color'], 
                                        self.headlights['bps'],
                                        self.headlights['brightness'])
        self.headlights['max_frames'] = len(self.headlights['frames'])
        self.headlights['frame_index'] = 0


    def set_tail_lights_style(self, style, color, bps=1, brightness=1):
        self.tail_lights['style'] = style
        self.tail_lights['color'] = color_2_tuple(color)
        self.tail_lights['bps'] = bps
        self.tail_lights['brightness'] = brightness

        fuc = getattr(self, style)
        self.tail_lights['frames'] = fuc(self.tail_lights['leds'],
                                        self.tail_lights['color'],
                                        self.tail_lights['bps'],
                                        self.tail_lights['brightness'])
        self.tail_lights['max_frames'] = len(self.tail_lights['frames'])
        self.tail_lights['frame_index'] = 0
        
    # styles frames calculations
    # =================================================================
    def cos_func(self, peak, a, x, offset=0, vm=0):
        """
        cos fuction

        :param peak:
        :param a: multiple
        :param x: xpos
        :return: result, float  or int
        """
        return (peak/2.0) * cos(a*x + offset) + peak/2 + vm
    
    def normal_distribution_calculate(self, u, sig, A, x, offset):
        """
        Normal distribution calculate

        :param u: mathematical expectation, average value, affects the x position of the highest point 
        :type u: float or int
        :param sig: standard deviation, affects the magnitude and range of the central area
        :type sig: float or int
        :param A: amplitude ratio
        :type  A: float or int
        :param x: x pos
        :type x: int
        :param offset: amplitude offset
        :type offset: float or int
        :return: Normal distribution y(x)
        :rtype: float or int
        """
        y = A*np.exp(-(x-u)**2/(2*sig**2))/(sqrt(2*pi)*sig) + offset
        return y
    
    def solid(self, leds, color, bps, brightness=1):
        color = [i*brightness for i in color]
        frames = []
        max_frames = 1
        for frame_index in range(max_frames):
            _frame = []
            for led_index in range(len(leds)):
                _frame.append(color)
            frames.append(_frame)

        return frames
    
    def breath(self, leds, color, bps, brightness=1):
        color = [i*brightness for i in color]

        frames = []
        max_frames = int(1/bps/self.MIN_DELAY)

        u = 4 # Control the brightest point position
        sig = 2 # Control the degree of dispersion
        A = 5 # Control the amplitude of the wave
        multiple = float(2*pi/max_frames) # multiple, period = max_frames

        for frame_index in range(max_frames):
            _frame = []
            offset = -self.cos_func(1, multiple, frame_index)
            for led_index in range(len(leds)):
                _brightness = self.normal_distribution_calculate(u, sig, A, led_index, offset)
                _frame.append(list([max(0, int(c * _brightness)) for c in color]))
            frames.append(_frame)

        return frames

    def speak(self, leds, color, bps, brightness=1):
        color = [i*brightness for i in color]
        frames = []
        max_frames = int(1/bps/self.MIN_DELAY)

        # sig = 1
        # A = 2.5
        sig = 0.8
        A = 2
        peak = (len(leds)-1)/2
        multiple = float(2*pi/(max_frames)) # multiple, period = max_frames

        for frame_index in range(max_frames):
            _frame = []
            u_offset = self.cos_func(peak, multiple, frame_index)
            for led_index in range(len(leds)):
                if led_index <= peak:
                    u = u_offset 
                else:
                    u = 2*peak - u_offset
                _brightness = self.normal_distribution_calculate(u, sig, A, led_index, 0)
                _frame.append(list([max(0, int(c * _brightness)) for c in color]))
            frames.append(_frame)

        return frames

    def listen(self, leds, color, bps, brightness=1):
        color = [i*brightness for i in color]
        frames = []
        max_frames = int(1/bps/self.MIN_DELAY)

        sig = 1
        A = 2.5   
        peak = len(leds)-1
        multiple = float(2*pi/(max_frames)) # T, multiple, period = max_frames
        offset = pi/2

        for frame_index in range(max_frames):
            _frame = []
            u = self.cos_func(peak, multiple, frame_index, offset)
            for led_index in range(len(leds)):
                _brightness = self.normal_distribution_calculate(u, sig, A, led_index, 0)
                _frame.append(list([max(0, int(c * _brightness)) for c in color]))
            frames.append(_frame)

        return frames
    
    def flow(self, leds, color, bps, brightness=1):
        color = [i*brightness for i in color]
        frames = []
        max_frames = int(1/bps/self.MIN_DELAY)

        sig = 1
        A = 2.5   
        peak = len(leds) + 6
        vm = -3
        multiple = float(pi/(max_frames)) # T/2, multiple, period = max_frames
        offset = -pi
        
        for frame_index in range(max_frames):
            _frame = []
            u = self.cos_func(peak, multiple, frame_index, offset, vm)
            for led_index in range(len(leds)):
                _brightness = self.normal_distribution_calculate(u, sig, A, led_index, 0)
                _frame.append(list([max(0, int(c * _brightness)) for c in color]))
            frames.append(_frame)

        return frames


    def flow_reverse(self, leds, color, bps, brightness=1):
        color = [i*brightness for i in color]
        frames = []
        max_frames = int(1/bps/self.MIN_DELAY)

        sig = 1
        A = 2.5   
        peak = len(leds) + 6
        vm = -3
        multiple = float(pi/(max_frames)) # T/2, multiple, period = max_frames
        offset = 0
        
        for frame_index in range(max_frames):
            _frame = []
            u = self.cos_func(peak, multiple, frame_index, offset, vm)
            for led_index in range(len(leds)):
                _brightness = self.normal_distribution_calculate(u, sig, A, led_index, 0)
                _frame.append(list([max(0, int(c * _brightness)) for c in color]))
            frames.append(_frame)

        return frames

if __name__ == '__main__':
    try:
        led_num = 16
        rgbs = NeoRGBStrip()

        # --- pre style ---
        rgbs.start()

        rgbs.set_tail_lights_style('solid', color='red', bps=1)
        # rgbs.set_headlights_style('breath', color='yellow', bps=.5)
        # rgbs.set_headlights_style('speak', color='pink', bps=1)
        # rgbs.set_headlights_style('listen', color='cyan', bps=1)
        # rgbs.set_headlights_style('flow', color='blue', bps=1)
        # rgbs.set_headlights_style('flow_reverse', color='red', bps=1)

        rgbs.set_headlights_style('breath', color='yellow', bps=1)
        time.sleep(3)

        rgbs.set_headlights_style('flow', color='blue', bps=1)
        time.sleep(3)

        rgbs.set_headlights_style('flow_reverse', color='red', bps=1)
        time.sleep(3)

        rgbs.stop()

        # --- set rgb items ---
        hue_delta = 360 * 1.0 / led_num 
        for i in range(led_num):
            hue = i * hue_delta
            color = hsl_to_rgb(hue, 1, 1)
            rgbs[i] = color
            rgbs.show()
            time.sleep(0.1)
       
       # --- change brightness ---
        while True:
            for i in range(21):
                brightness = i / 20
                print(f'changing brightness to {brightness}')
                rgbs.brightness = brightness
                rgbs.show()
                time.sleep(0.1)
            for i in range(20, -1, -1):
                brightness = i / 20
                print(f'changing brightness to {brightness}')
                rgbs.brightness = brightness
                rgbs.show()
                time.sleep(0.1)
    finally:
        rgbs.stop()


