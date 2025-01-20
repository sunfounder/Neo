import time
import threading

from math import cos, pi

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
RGB_STYLES = [
    'solid', 'breathing', 'flow', 'flow_reverse', 'rainbow', 'rainbow_reverse', 'hue_cycle'
]


class WS2812_SPI(NeoPixel_SPI):

    def __init__(self, led_num):

        spi = board.SPI()
        PIXEL_ORDER = neopixel.GRB

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

class RGB_Strip(WS2812_SPI):

    lights_order = [
        0, 1, 2, 3, 4, 5, 6, 7, 
        8, 9, 10, 11, 12, 13, 14, 15,
    ]

    def __init__(self, led_num):

        self.led_num = led_num
        self.running = False
        self._is_ready = False
        self.style = 'breath'
        self.color = (0, 255, 255)
        self.speed = 80

        try:
            super().__init__(self.led_num)
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
            try:
                style_func = getattr(self, self.style)
                style_func()
            except KeyError as e:
                print(f'Style error: {e}')
            except Exception as e:
                print(f'WS2812 error: {type(e)} {e}')
            self.counter += 1
            if self.counter >= self.counter_max:
                self.counter = 0

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

    def set_style(self, style, color, speed, log=False):
        # style
        if not isinstance(style, str) or style not in RGB_STYLES:
            if log:
                print(f"Invalid style: {style}")
            return
        self.style = style
        # color
        self.color = color_2_tuple(color)
        # speed
        if not isinstance(speed, int):
            if log:
                print(f"Invalid speed: {speed}")
            return
        self.speed = speed

    # styles
    # =================================================================
    def solid(self):
        self.fill(self.color)
        self.show()
        time.sleep(1)

    def breathing(self):
        self.counter_max = 200
        delay = map_value(self.speed, 0, 100, 0.1, 0.001)

        if self.counter < 100:
            i = self.counter
            r, g, b = [int(x * i * 0.01) for x in self.color]
        else:
            i = 200 - self.counter
            r, g, b = [int(x * i * 0.01) for x in self.color]
        self.fill((r, g, b))
        self.show()
        time.sleep(delay)

    def flow(self, order = None):
        self.counter_max = self.led_num
        delay = map_value(self.speed, 0, 100, 0.5, 0.1)
        
        if order is None:
            order = self.lights_order

        self.fill(0)
        index = self.lights_order[self.counter]
        self[index] = self.color
        self.show()
        time.sleep(delay)

    def flow_reverse(self):
        order = self.lights_order[::-1]
        self.flow(order)

if __name__ == '__main__':
    try:
        led_num = 16
        rgbs = RGB_Strip(led_num)

        # --- pre style ---
        rgbs.set_style('flow', (0, 0, 255), 80)
        rgbs.start()
        print(f'starting rgb strip, style: {rgbs.style}, brightness:{rgbs.brightness}')
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


