from zeus_pi import Neo
from time import sleep

my_car = Neo()
move_power = 60
roate_power = 70

try:
    while True:
        my_car.move(0, 0, roate_power) # rotate_power = 50, anticlockwise rotate
        sleep(1)
        my_car.move(0, move_power, roate_power) # rotate and move
        sleep(3)
        my_car.move(0, 0, -roate_power) # rotate_power = -50, clockwise rotate
        sleep(1)
        my_car.move(-180, move_power, roate_power) # rotate and move
        sleep(3)
finally:
    my_car.stop()
