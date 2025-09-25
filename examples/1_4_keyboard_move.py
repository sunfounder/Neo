from neo import Neo
from time import sleep
import readchar     

manual = '''
Press keys on keyboard to control Neo robot!
    w: Forward
    a: Turn left
    s: Backward
    d: Turn right
    i: Head up
    k: Head down
    j: Turn head left
    l: Turn head right
    space: Stop
    ctrl+c: Press twice to exit the program
'''

def show_info():
    print("\033[H\033[J", end='')  # Clear the terminal window
    print(manual)


if __name__ == "__main__":
    try:
        pan_angle = 0  # Pan angle
        tilt_angle = 0  # Tilt angle
        power = 50      # Motor power
        
        my_car = Neo()
        show_info()
        
        while True:
            key = readchar.readkey()
            key = key.lower()
            
            if key in('wsadikjl'):
                if 'w' == key:
                    my_car.forward(power)
                elif 's' == key:
                    my_car.backward(power)
                elif 'a' == key:
                    my_car.turn_left(power)
                elif 'd' == key:
                    my_car.turn_right(power)
                elif 'i' == key:
                    tilt_angle += 5
                    if tilt_angle > 30:
                        tilt_angle = 30
                elif 'k' == key:
                    tilt_angle -= 5
                    if tilt_angle < -30:
                        tilt_angle = -30
                elif 'l' == key:
                    pan_angle += 5
                    if pan_angle > 30:
                        pan_angle = 30
                elif 'j' == key:
                    pan_angle -= 5
                    if pan_angle < -30:
                        pan_angle = -30
                
                # Update camera angle
                my_car.set_cam_tilt(tilt_angle)
                my_car.set_cam_pan(pan_angle)
                show_info()
                
            elif key == ' ':
                my_car.stop()
                show_info()
                print("Robot stopped")
            elif key == readchar.key.CTRL_C:
                print("\n Quit")
                break
            
            sleep(0.1)  # delay to avoid high CPU usage
    
    finally:
        my_car.set_cam_tilt(0)
        my_car.set_cam_pan(0)
        my_car.stop()
        sleep(0.2)
        print("Robot returned to initial position")

