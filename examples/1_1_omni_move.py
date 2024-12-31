from zeus_pi import ZeusPi
from time import sleep

my_car = ZeusPi()
power = 30

try:
    while True:
        my_car.move(0, power)
        sleep(1)
        my_car.move(180, power)
        sleep(1)
        my_car.move(45, power)
        sleep(1)
        my_car.move(225, power)
        sleep(1)
        my_car.move(90, power)
        sleep(1)
        my_car.move(270, power)
        sleep(1)
        my_car.move(135, power)
        sleep(1)
        my_car.move(315, power)
        sleep(1)
        #
        my_car.move(180, power)
        sleep(1)
        my_car.move(0, power)
        sleep(1)
        my_car.move(225, power)
        sleep(1)
        my_car.move(45, power)
        sleep(1)
        my_car.move(270, power)
        sleep(1)
        my_car.move(90, power)
        sleep(1)
        my_car.move(315, power)
        sleep(1)
        my_car.move(135, power)
        sleep(1)
        #
        my_car.stop()
        sleep(2)
finally:
    print("Stop")
    my_car.stop()
    sleep(0.1)
