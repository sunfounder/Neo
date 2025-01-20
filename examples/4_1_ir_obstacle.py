from zeus_pi import ZeusPi
from time import sleep

my_car = ZeusPi()

try:
    while True:
        ir_l, ir_r = my_car.read_ir_obstacle()
        print(f"ir_l: {ir_l},    ir_r: {ir_r}")
        sleep(0.1)
finally:
    my_car.stop()

