from zeus_pi import Neo
from time import sleep

my_car = Neo()

FOLLOWING_MIN_DISTANCE = 5 # cm
FOLLOWING_SUITE_DISTANCE = 10 # cm
FOLLOWING_MAX_DISTANCE = 15 # cm
FORWARD_POWER = 65
FORWARD_SLOWLY_POWER = 30
TURNING_POWER = 50

def obstacle_following():
    ir_l, ir_r = my_car.read_ir_obstacle()
    distance = my_car.ultrasonic.read()

    is_left_clear = ir_l
    is_right_clear = ir_r
    is_middle_clear = 1 if (distance > FOLLOWING_MAX_DISTANCE ) else 0

    print(f"{[is_left_clear, is_middle_clear, is_right_clear]},    distance: {distance}")

    if (distance < FOLLOWING_MIN_DISTANCE):
        my_car.stop()
    elif (distance < FOLLOWING_SUITE_DISTANCE):
        my_car.forward(FORWARD_SLOWLY_POWER)
    elif (distance < FOLLOWING_MAX_DISTANCE):
        my_car.forward(FORWARD_POWER)
    else:
        if (not is_left_clear):
            my_car.turn_left(TURNING_POWER)
        elif (not is_right_clear):
            my_car.turn_right(TURNING_POWER)
        else:
            my_car.stop()

try:
    while True:
        obstacle_following()
        sleep(0.05)
finally:
    my_car.stop()

