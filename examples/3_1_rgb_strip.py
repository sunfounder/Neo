from zeus_pi import ZeusPi
from time import sleep

my_car = ZeusPi()

try:
    #
    my_car.rgb_strip.start()
    sleep(3)
    #
    config = {
        'rgb_led_count': 16,
        'rgb_color': '#001aaf',
        'rgb_brightness': 100,  # 0-100
        'rgb_style': 'flow',
        'rgb_speed': 100,
    }
    my_car.rgb_strip.update_config(config)

    while True:
        sleep(1)

finally:
    my_car.rgb_strip.stop()
