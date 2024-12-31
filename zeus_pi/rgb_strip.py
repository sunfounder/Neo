import time
import threading

from math import cos, pi

# utils
# =================================================================
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

# Variables define
# =================================================================
RGB_STYLES = [
    'solid', 'breathing', 'flow', 'flow_reverse', 'rainbow', 'rainbow_reverse', 'hue_cycle'
]

default_config = {
    'rgb_led_count': 16,
    'rgb_color': '#00ffff',
    'rgb_brightness': 100,  # 0-100
    'rgb_style': 'breath',
    'rgb_speed': 50,
}


class WS2812_SPI():

    def __init__(self, led_num):

        # https://github.com/adafruit/Adafruit_CircuitPython_NeoPixel_SPI
        import board
        import neopixel_spi as neopixel
        from neopixel_spi import NeoPixel_SPI

        spi = board.SPI()
        PIXEL_ORDER = neopixel.GRB

        self.strip = NeoPixel_SPI(
                spi, led_num, pixel_order=PIXEL_ORDER, auto_write=False
        )
        time.sleep(0.01)
        self.clear()

    def show(self):
        self.strip.show()

    def clear(self):
        self.strip.fill(0)
        self.strip.show()

    def fill(self, color:str='#000000'):
        self.strip.fill(color)

    def fill_pattern(self, pattern):
        for i in range(self.led_num):
            self.strip[i] = pattern[i]

#TODO:
#  class RGBStrip_I2C()
#
#

class RGB_Strip():

    default_config = {
        'rgb_led_count': 16,
        'rgb_color': '#00ffff',
        'rgb_brightness': 100,  # 0-100
        'rgb_style': 'breath',
        'rgb_speed': 100,
    }

    lights_order = [
        0, 1, 2, 3, 4, 5, 6, 7, 
        8, 9, 10, 11, 12, 13, 14, 15,
    ]

    def __init__(self, config=default_config, driver='ws2812_spi'):

        self.led_num = None
        self.driver = driver
        self.running = False
        self._is_ready = False
        self.color = None
        self.speed = None
        self.style = None
        self.brightness = None
        self.update_config(config, log=False)

        # try:
        if self.driver == 'ws2812_spi':
            self.rgbs = WS2812_SPI(self.led_num)
        # elif self.driver == 'rgbstrip_i2c':
        #     self.rgbs = RGBStrip_I2C()
        self._is_ready = True
        # except Exception as e:
        #     self._is_ready = False
        #     print(f"Error: {e}")

    def clear(self):
        self.rgbs.clear()

    def show(self):
        self.rgbs.show()

    def fill(self, color):
        self.rgbs.fill(color)

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

    def update_config(self, config, log=True):
        if 'rgb_led_count' in config:
            if not isinstance(config['rgb_led_count'], int):
                if log:
                    print("Invalid rgb_led_count")
                return
            self.led_num = config['rgb_led_count']
            if log:
                print(f"Update LED count: {self.led_num}")
        if 'rgb_enable' in config:
            if not isinstance(config['rgb_enable'], bool):
                if log:
                    print("Invalid rgb_enable")
                return
            self.enable = config['rgb_enable']
            if log:
                print(f"Update RGB enable: {self.enable}")
        if 'rgb_color' in config:
            if not isinstance(config['rgb_color'], str):
                if log:
                    print("Invalid rgb_color")
                return
            self.color = hex_to_rgb(config['rgb_color'])
            if log:
                print(f"Update RGB color: {self.color}")
        if 'rgb_brightness' in config:
            if not isinstance(config['rgb_brightness'], int):
                if log:
                    print("Invalid rgb_brightness")
                return
            self.brightness = config['rgb_brightness']
            if log:
                print(f"Update RGB brightness: {self.brightness}")
        if 'rgb_speed' in config:
            if not isinstance(config['rgb_speed'], int):
                if log:
                    print("Invalid rgb_speed")
                return
            self.speed = config['rgb_speed']
            if log:
                print(f"Update RGB speed: {self.speed}")
        if 'rgb_style' in config:
            if not isinstance(config['rgb_style'], str) or config['rgb_style'] not in RGB_STYLES:
                if log:
                    print("Invalid rgb_style")
                return
            self.style = config['rgb_style']
            if log:
                print(f"Update RGB style: {self.style}")
    # styles
    # =================================================================
    def solid(self):
        color = [int(x * self.brightness * 0.01) for x in self.color]
        self.strip.fill(color)
        self.show()
        time.sleep(1)

    def breathing(self):
        self.counter_max = 200
        delay = map_value(self.speed, 0, 100, 0.1, 0.001)
        color = [int(x * self.brightness * 0.01) for x in self.color]

        if self.counter < 100:
            i = self.counter
            r, g, b = [int(x * i * 0.01) for x in color]
        else:
            i = 200 - self.counter
            r, g, b = [int(x * i * 0.01) for x in color]
        self.fill((r, g, b))
        self.show()
        time.sleep(delay)

    def flow(self, order = None):
        self.counter_max = self.led_num
        delay = map_value(self.speed, 0, 100, 0.5, 0.1)
        color = [int(x * self.brightness * 0.01) for x in self.color]
        
        if order is None:
            order = self.lights_order

        self.fill(0)
        index = self.lights_order[self.counter]
        self.rgbs.strip[index] = color
        self.show()
        time.sleep(delay)

    def flow_reverse(self):
        order = self.lights_order[::-1]
        self.flow(order)

if __name__ == '__main__':

    config = {
        'rgb_led_count': 16,
        'rgb_color': '#0a000a',
        'rgb_brightness': 100,
        'rgb_style': 'flow',
        'rgb_speed':50,
    }

    rgb = RGB_Strip(config)
    rgb.start()
