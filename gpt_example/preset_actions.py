
from time import sleep
import random
from math import sin, cos, pi

def shake_head(car):
    car.stop()
    car.set_cam_pan(0)
    car.set_cam_pan(60)
    sleep(.2)
    car.set_cam_pan(-50)
    sleep(.1)
    car.set_cam_pan(40)
    sleep(.1)
    car.set_cam_pan(-30)
    sleep(.1)
    car.set_cam_pan(20)
    sleep(.1)
    car.set_cam_pan(-10)
    sleep(.1)
    car.set_cam_pan(10)
    sleep(.1)
    car.set_cam_pan(-5)
    sleep(.1)
    car.set_cam_pan(0)

def nod(car):
    car.reset()
    car.set_cam_tilt(0)
    car.set_cam_tilt(5)
    sleep(.1)
    car.set_cam_tilt(-30)
    sleep(.1)
    car.set_cam_tilt(5)
    sleep(.1)
    car.set_cam_tilt(-30)
    sleep(.1)
    car.set_cam_tilt(0)

def depressed(car):
    car.reset()
    car.set_cam_tilt(0)
    car.set_cam_tilt(20)
    sleep(.22)
    car.set_cam_tilt(-22)
    sleep(.1)
    car.set_cam_tilt(10)
    sleep(.1)
    car.set_cam_tilt(-22)
    sleep(.1)
    car.set_cam_tilt(0)
    sleep(.1)
    car.set_cam_tilt(-22)
    sleep(.1)
    car.set_cam_tilt(-10)
    sleep(.1)
    car.set_cam_tilt(-22)
    sleep(.1)
    car.set_cam_tilt(-15)
    sleep(.1)
    car.set_cam_tilt(-22)
    sleep(.1)
    car.set_cam_tilt(-19)
    sleep(.1)
    car.set_cam_tilt(-22)
    sleep(.1)

    sleep(1.5)
    car.reset()

def keep_think(car):
    car.reset()
    for i in range(11):
        car.set_cam_pan(i*3)
        car.set_cam_tilt(-i*2)
        sleep(.05)

def honking(music):
    import utils
    # utils.speak_block(music, "../sounds/car-double-horn.wav", 100)
    music.sound_play_threading("../sounds/car-double-horn.wav", 100)

def start_engine(music):
    import utils
    # utils.speak_block(music, "../sounds/car-start-engine.wav", 100)
    music.sound_play_threading("../sounds/car-start-engine.wav", 50)


actions_dict = {
    "shake head":shake_head, 
    "nod": nod,
    "depressed": depressed,
    "keep_think": keep_think,
}

sounds_dict = {
    "honking": honking,
    "start engine": start_engine,
}


if __name__ == "__main__":
    from neo import Neo
    import os

    current_path = os.path.dirname(os.path.abspath(__file__))
    os.chdir(current_path) # change working directory

    my_car = Neo()
    my_car.reset()


    sleep(.5)

    _actions_num = len(actions_dict)
    actions = list(actions_dict.keys())
    for i, key in enumerate(actions_dict):
        print(f'{i} {key}')
    
    _sounds_num = len(sounds_dict)
    sounds = list(sounds_dict.keys())
    for i, key in enumerate(sounds_dict):
        print(f'{_actions_num+i} {key}')

    last_key = None

    try:
        while True:
            key = input()

            if key == '':
                if last_key > _actions_num - 1:
                    print(sounds[last_key-_actions_num])
                    sounds_dict[sounds[last_key-_actions_num]](my_car.speaker)
                else:
                    print(actions[last_key])
                    actions_dict[actions[last_key]](my_car)
            else:
                key = int(key)
                if key > (_actions_num + _sounds_num - 1):
                    print("Invalid key")
                elif key > (_actions_num - 1):
                    last_key = key
                    print(sounds[last_key-_actions_num])
                    sounds_dict[sounds[last_key-_actions_num]](my_car.speaker)
                else:
                    last_key = key
                    print(actions[key])
                    actions_dict[actions[key]](my_car)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error:\n {e}')
    finally:
        my_car.reset()
        sleep(.1)




