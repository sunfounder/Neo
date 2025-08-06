from robot_hat import TTS
from time import sleep
import readline # optimize keyboard input, only need to import
import sys

tts = TTS()
tts.lang("en-US")

print('Please enter the sentence to TTS')

while True: 
    _result = input(f'\033[38;5;240m{"intput: "}\033[0m').encode(sys.stdin.encoding).decode('utf-8')

    if _result == False or _result == "":
        print() # new line
        continue
    else:
        tts.say(_result)

    sleep(.1)
