from zeus_pi import Neo
from time import sleep

my_car = Neo()

# please run examples/calibration.py to set line_reference and cliff_reference
# my_car.set_line_reference([1000, 1000, 1000])
# my_car.set_cliff_reference([200, 200, 200])

try:
    while True:
        value, status = my_car.read_grayscale()
        for i in range(3):
            if status[i] == 'cliff':
                status[i] = '!'
            elif status[i] == 'inside':
                status[i] = 'x'
            elif status[i] == 'outside':
                status[i] = '_'
        print(f"value: {value},    status: {status[0]} {status[1]} {status[2]}")
        sleep(0.5)
finally:
    my_car.stop()
