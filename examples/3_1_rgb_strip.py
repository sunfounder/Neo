from zeus_pi import Neo
from zeus_pi.rgb_strip import hsl_to_rgb
from time import sleep

my_car = Neo()

try:
    # --- use preset style ---
    my_car.rgbs.set_style('flow', (0, 0, 255), 80)
    my_car.rgbs.start() # start preset style loop
    print(f'starting rgb strip, style: {my_car.rgbs.style}, brightness:{my_car.rgbs.brightness}')
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
