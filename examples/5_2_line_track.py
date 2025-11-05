from neo import Neo
from fusion_hat.modules import Grayscale_Module,LineTracker
from fusion_hat.adc import ADC
import time
import threading

my_car = Neo()

line_tracker_slopes = my_car.config.get('line_tracker_slopes', [1, 1, 1])
line_tracker_offsets = my_car.config.get('line_tracker_offsets', [0, 0, 0])

gs = Grayscale_Module(ADC(0), ADC(1), ADC(2), reference=2000)
line_tracker = LineTracker(ADC(0), ADC(1), ADC(2),offsets=line_tracker_offsets,slopes=line_tracker_slopes)

line_position = 0.0  # mid
display_run = True  
thread_run = True  

FORWARD_POWER = 50
TURNING_POWER = 45

# DISPLAY_MODE: 'animation' - animation mode, 'simple' - simple print
DISPLAY_MODE = 'simple'

def read_reference():
    '''read line tracker reference from config file'''
    try:
        print("--------------------")
        print(f"line_tracker_slopes: {line_tracker_slopes}")
        print(f"line_tracker_offsets: {line_tracker_offsets}")
        print("--------------------")
    except Exception as e:
        raise e

def display_position_bar(position):
    position = max(-1.0, min(1.0, position))
    
    total_separators = 10
    
    left_count = int((position + 1.0) / 2.0 * total_separators)
    right_count = total_separators - left_count
    
    # creat position bar
    bar = f"|{'-' * left_count}□{'-' * right_count}|"
    
    return bar

def car_action():
    global line_position, display_run,thread_run

    while thread_run:  
        if display_run: 
            if DISPLAY_MODE == 'animation':
                print('\r\033[K', end='')  # clear current line
                bar = display_position_bar(line_position)
                print(f"位置: {line_position:.2f} | {bar}")
            elif DISPLAY_MODE == 'simple':
                print(f"位置: {line_position:.2f}")
        
        time.sleep(0.1)  


def main():
    # read from config file
    read_reference()
    global display_run,thread_run

    display_thread = threading.Thread(target=car_action)
    display_thread.daemon = True  
    display_thread.start()

    try:
        while True:
            # read
            gray_value_raw = line_tracker.read(raw=True)
            line_position = line_tracker.get_line_position(gray_value_raw)
            is_on_cliff = line_tracker.is_on_cliff()
            is_on_line = line_tracker.is_on_line()

            if is_on_line and not is_on_cliff:
                display_run = True
                if line_position < -0.5 and line_position != 0.0:
                    my_car.turn_left(TURNING_POWER)
                    print("turn left")
                elif line_position > 0.5 and line_position != 0.0:
                    my_car.turn_right(TURNING_POWER)
                    print("turn right")
                elif line_position == 0.0:
                    my_car.forward(FORWARD_POWER)
                    print("forward")
                time.sleep(0.05)
            elif not is_on_line:
                print("is not on line")
                my_car.stop()
                display_run = False
                # my_car.turn_left(TURNING_POWER)
                # if line_position:
                #     break
            elif is_on_cliff:
                print("is on cliff")
                my_car.stop()
                display_run = False
                break
          
    except KeyboardInterrupt:
        print("\nstop\n")
    finally:
        my_car.stop()  
        display_run = False
        thread_run = False 

if __name__ == '__main__':
    try:
        main()
    finally:
        thread_run = False
