from zeus_pi import Neo
from time import sleep

my_car = Neo()

try:
    while True:
        distance = my_car.ultrasonic.read()
        print(f"distance: {distance} cm")
        sleep(0.1)
finally:
    my_car.stop()
