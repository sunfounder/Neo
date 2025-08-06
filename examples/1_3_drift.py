from neo import Neo
from time import sleep

my_car = Neo()
move_power = 60
roat_power = 50

try:
    while True:
        my_car.move(0, move_power, drift=False) 
        sleep(0.2)
        my_car.move(0, move_power, roat_power, drift=True) 
        sleep(1)
finally:
    my_car.stop()
