from neo import Neo
import time
import os
# ****************************************************************
# To run the script, you need to install the sox tool first:
#    sudo apt install sox
#
# ****************************************************************

REC_PATH = '/tmp/'

_time = 5 # seconds

print(f'Recording for {_time} seconds')
os.system(f'rec -c 1 -r 44100 {REC_PATH}rec_test..wav trim 0 {_time}')

print('Playing the recorded sound')
os.system(f'play {REC_PATH}rec_test..wav')

