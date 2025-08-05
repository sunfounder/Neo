from zeus_pi import Neo
from time import sleep

my_car = Neo()

try:
    while True:
        ir_l, ir_r = my_car.read_ir_obstacle()
        print(f"ir_l: {ir_l},    ir_r: {ir_r}")
        sleep(0.1)
finally:
    my_car.stop()

