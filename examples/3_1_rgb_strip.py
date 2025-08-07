from neo import Neo
from neo.rgb_strip import hsl_to_rgb
from time import sleep

my_car = Neo()

try:
    # --- use preset style ---
    my_car.rgbs.start() # start preset style loop

    # set tail_lights
    my_car.rgbs.set_tail_lights_style('listen', color=(0, 0, 255), bps=1)
    sleep(3)
    my_car.rgbs.close_tail_lights() # close tail_lights

    # set head_lights
    my_car.rgbs.set_headlights_style('breath', color='yellow', bps=1)
    sleep(3)
    my_car.rgbs.set_headlights_style('flow', color='blue', bps=1)
    sleep(3)
    my_car.rgbs.set_headlights_style('flow_reverse', color='red', bps=1)
    sleep(3)
    
    my_car.rgbs.stop() # stop preset style loop

    # --- set rgb items ---
    hue_delta = 360 * 1.0 / my_car.rgbs.led_num 
    for i in range(my_car.rgbs.led_num):
        hue = i * hue_delta
        color = hsl_to_rgb(hue, 1, 1)
        my_car.rgbs[i] = color
        my_car.rgbs.show() # show rgbs
        sleep(0.1)
    
    # --- change brightness ---
    while True:
        for i in range(21):
            brightness = i / 20
            print(f'changing brightness to {brightness}')
            my_car.rgbs.brightness = brightness
            my_car.rgbs.show()
            sleep(0.1)
        for i in range(20, -1, -1):
            brightness = i / 20
            print(f'changing brightness to {brightness}')
            my_car.rgbs.brightness = brightness
            my_car.rgbs.show()
            sleep(0.1)
finally:
    my_car.rgbs.stop()
