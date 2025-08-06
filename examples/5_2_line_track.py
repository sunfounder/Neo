from neo import Neo
from time import sleep

my_car = Neo()

FORWARD_POWER = 30
TURNING_POWER = 15

# please run examples/calibration.py to set line_reference and cliff_reference
# my_car.set_line_reference([1000, 1000, 1000])
# my_car.set_cliff_reference([200, 200, 200])

my_car.set_line_reference([657, 765, 658])
my_car.set_cliff_reference([200, 200, 200])

last_status = 'stop'

def line_track():
    global  last_status
 
    # read data
    value, _status = my_car.read_grayscale()

    # ananalyze status
    for i in range(3):
        if _status[i] == 'cliff':
            _status[i] = '!'
        elif _status[i] == 'inside':
            _status[i] = 'x'
        elif _status[i] == 'outside':
            _status[i] = '_'
    
    status = None
    if '!' in _status:
        status = 'cliff'
        last_status = 'cliff'
    elif _status == ['_', '_', '_'] :
        if last_status == 'stop' or last_status == 'cliff':
            status = 'stop'
            last_status = 'stop'
        else:
            status = 'outside'
            last_status = 'outside'
    elif _status[1] == 'x':
        status = 'forward'
        last_status = 'forward'
    elif _status[0] == 'x':
        status = 'left'
        last_status = 'left'
    elif _status[2] == 'x':
        status = 'right'
        last_status = 'right'

    print(f'value: {value},  status: {_status[0]} {_status[1]} {_status[2]}, {status}')

    # move
    if status == 'cliff':
        my_car.stop()
    elif status == 'forward':
        my_car.forward(FORWARD_POWER)
    elif status == 'left':
        my_car.turn_left(TURNING_POWER)
    elif status == 'right':
        my_car.turn_right(TURNING_POWER)
    elif status == 'outside':
        #TODO: outside handler
        my_car.stop()

try:
    while True:
        line_track()
        sleep(0.05)
finally:
    my_car.stop()
