'''
Use ultrasonic module and IR obstacle detection module to achieve mobile obstacle avoidance.
Adjust the triggering distance of the IR obstacle detection module to `15cm` is optimal.

Reference Tutorial: https://docs.sunfounder.com/projects/zeus-car/en/latest/get_started/ar_avoid.html

'''
from zeus_pi import ZeusPi
from time import sleep

my_car = ZeusPi()

SAFE_DISTANCE = 40 # cm
FORWARD_POWER = 65
TURNING_POWER = 45
last_status = "forward"

def obstacle_avoidance():
    global last_status

    ir_l, ir_r = my_car.read_ir_obstacle()
    distance = my_car.ultrasonic.read()

    is_left_clear = ir_l
    is_right_clear = ir_r
    is_middle_clear = 1 if (distance >= SAFE_DISTANCE) else 0

    print(f"{[is_left_clear, is_middle_clear, is_right_clear]},    distance: {distance}")

    # 111
    if (is_left_clear and is_right_clear and is_middle_clear):
        last_status = 'forward'
        my_car.forward(FORWARD_POWER)
    else:
        # 101, 000, 010
        if ((is_left_clear and is_right_clear) or (not is_left_clear and not is_right_clear)):
            if (last_status == 'left'):
                my_car.turn_left(TURNING_POWER)
            elif (last_status == 'right'):
                my_car.turn_right(TURNING_POWER)
            else:
                last_status = 'left'
                my_car.turn_left(TURNING_POWER)
        # 100, 110
        elif (is_left_clear):
            last_status = 'left'
            my_car.turn_left(TURNING_POWER)
        # 001, 011
        elif (is_right_clear):
            last_status = 'right'
            my_car.turn_right(TURNING_POWER)

try:
    while True:
        obstacle_avoidance()
        sleep(0.01)
finally:
    my_car.stop()

