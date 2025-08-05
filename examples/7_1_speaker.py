from zeus_pi import Neo
from time import sleep

# More to see: robot_hat/music.py
#  https://github.com/sunfounder/robot-hat/blob/v2.0/robot_hat/music.py

import os
current_path = os.path.dirname(os.path.abspath(__file__))
os.chdir(current_path) # change working directory

# MUSIC_PATH = '../music/slow-trail-Ahjay_Stelino.mp3'
MUSIC_PATH = '../music/spry.mp3'
SOUND_PATH = '../sounds/car-double-horn.wav'

my_car = Neo()

try:
    my_car.speaker.music_play(MUSIC_PATH, volume=100) # background music

    for i in range(100, 20, -2):
        print(f'music volume: {i}')
        my_car.speaker.music_set_volume(i)
        sleep(0.1)
    for i in range(20, 72, 2):
        print(f'music volume: {i}')
        my_car.speaker.music_set_volume(i)
        sleep(0.1)

    sleep(1.5)

    my_car.speaker.music_pause()
    my_car.speaker.sound_play(SOUND_PATH, volume=100)
    sleep(1.5)
    my_car.speaker.music_resume()
    sleep(3)

    my_car.speaker.sound_play(SOUND_PATH, volume=100)
    sleep(6)

finally:
    my_car.stop()


