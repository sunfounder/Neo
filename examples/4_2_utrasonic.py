from zeus_pi import ZeusPi
from time import sleep

my_car = ZeusPi()

try:
    while True:
        distance = my_car.ultrasonic.read()
        print(f"distance: {distance} cm")
        sleep(0.5)
finally:
    my_car.stop()
