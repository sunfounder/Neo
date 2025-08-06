from neo import Neo
from vilib import Vilib
import readchar
import time

my_car = Neo()


manual = '''
Press keys on keyboard to control the camera Servos!
    pan: W/S
    tilt: A/D                   Ctrl+C: Exit

'''
pan_angle = 0
tilt_angle = 0
SERVO_MOVE_STEP = 3

def main():
    global pan_angle, tilt_angle

    Vilib.camera_start(vflip=True, hflip=False, size=(1280, 720))
    Vilib.show_fps()
    Vilib.display(local=True, web=True) 

    # wait for web display to be ready
    while True:
        if Vilib.flask_start:
            break
        time.sleep(0.01)

    time.sleep(.5)
    print(manual)

    while True:
        key = readchar.readkey().lower()
        if key in ('ad'):
            if key == 'a':
                pan_angle -= SERVO_MOVE_STEP
                if pan_angle < -90:
                    pan_angle = -90
            elif key == 'd':
                pan_angle += SERVO_MOVE_STEP
                if pan_angle > 90:
                    pan_angle = 90
            my_car.set_cam_pan(pan_angle)
        elif key in ('ws'):
            if key == 'w':
                tilt_angle += SERVO_MOVE_STEP
                if tilt_angle > 90:
                    tilt_angle = 90
            elif key == 's':
                tilt_angle -= SERVO_MOVE_STEP
                if tilt_angle < -55:
                    tilt_angle = -55
            my_car.set_cam_tilt(tilt_angle)
            
        time.sleep(0.1)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\033[31mERROR: {e}\033[m")
    finally:
        Vilib.camera_close()
