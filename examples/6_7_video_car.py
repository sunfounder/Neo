# #!/usr/bin/env python3

from neo import Neo
from vilib import Vilib
from time import sleep, time, strftime, localtime
import readchar
import os

user = os.getlogin()
user_home = os.path.expanduser(f'~{user}')

os.makedirs(f"{user_home}/picar-x/photo/", exist_ok=True)

manual = '''
Press key to call the function(non-case sensitive):

    O: speed up
    P: speed down
    W: forward  
    S: backward
    A: turn left
    D: turn right
    F: stop
    T: take photo

    Ctrl+C: quit
'''

# you can view the video stream at http://localhost:9000/mjpg
car = Neo()

def take_photo():
    _time = strftime('%Y-%m-%d-%H-%M-%S',localtime(time()))
    name = 'photo_%s'%_time
    
    path = f"{user_home}/picar-x/photo/"
    
    try:
        ret = Vilib.take_photo(name, path)
        if ret:
            print('\nphoto save as %s%s.jpg'%(path,name))
        else:
            print('\nphoto save failed')

    except Exception as e:
        print(f"\nError: {e}")
        print("\nAll possible paths failed. Cannot save photo.")


def move(operate:str, speed):

    if operate == 'stop':
        car.stop()  
    else:
        if operate == 'forward':
            car.forward(speed)
        elif operate == 'backward':
            car.backward(speed)
        elif operate == 'turn left':
            car.turn_left(speed)
        elif operate == 'turn right':
            car.turn_right(speed)

def main():
    speed = 0
    status = 'stop'

    Vilib.camera_start(vflip=True,hflip=True)
    Vilib.show_fps()  
    Vilib.display(local=True,web=True)
    
    while True:
        if Vilib.flask_start:
            break
        sleep(0.01)
    
    sleep(2)  # wait for startup
    print(manual)
    
    while True:
        print("\rstatus: %s , speed: %s    "%(status, speed), end='', flush=True)
        # readkey
        key = readchar.readkey().lower()
        # operation 
        if key in ('wsadfop'):
            # throttle
            if key == 'o':
                if speed <=90:
                    speed += 10           
            elif key == 'p':
                if speed >=10:
                    speed -= 10
                if speed == 0:
                    status = 'stop'
            # direction
            elif key in ('wsad'):
                if speed == 0:
                    speed = 10
                if key == 'w':
                    # Speed limit when reversing,avoid instantaneous current too large
                    if status != 'forward' and speed > 60:  
                        speed = 60
                    status = 'forward'
                elif key == 'a':
                    status = 'turn left'
                elif key == 's':
                    if status != 'backward' and speed > 60: # Speed limit when reversing
                        speed = 60
                    status = 'backward'
                elif key == 'd':
                    status = 'turn right' 
            # stop
            elif key == 'f':
                status = 'stop'
            # move 
            move(status, speed)  

        elif key == 't':
            take_photo()
        # quit
        elif key == readchar.key.CTRL_C:
            print('\nquit ...')
            car.stop()
            Vilib.camera_close()
            break 

        sleep(0.1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Keyboard interrupt")
    finally:
        car.reset()
        Vilib.camera_close()
        print("Stop and exit")
        sleep(0.1)
        