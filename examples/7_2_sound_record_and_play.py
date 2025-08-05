from zeus_pi import Neo
import time
import os
# ****************************************************************
# To run the script, you need to install the sox tool first:
#    sudo apt install sox
#
# ****************************************************************

REC_PATH = '/tmp/'


os.system(f'rec -c 1 -r 44100 {REC_PATH}rec_test..wav'
        f" && play {REC_PATH}rec_test..wav"
          )

