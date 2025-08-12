from vilib import Vilib
import time

# More To see: 
# https://github.com/sunfounder/vilib/tree/main/examples,
# https://github.com/sunfounder/vilib/tree/main/vilib

def main():
    # For the assembled Raspberry Pi camera, using the 4:3 native resolution will result in a faster frame rate.
    Vilib.camera_start(vflip=True, hflip=True, size=(1280, 960))
    Vilib.show_fps()
    Vilib.display(local=True, web=True) 
    '''
    local:local display, web:web display
    when local=True, the image window will be displayed on the system desktop
    when web=True, the image window will be displayed on the web browser at http://localhost:9000/mjpg
    '''
    # wait for web display to be ready
    while True:
        if Vilib.flask_start:
            break
        time.sleep(0.01)

    time.sleep(.5)
    print('\npress Ctrl+C to exit')

    while True:
        # do something
        time.sleep(1)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\033[31mERROR: {e}\033[m")
    finally:
        Vilib.camera_close()

