from fusion_hat.tts import Piper
from time import sleep
import readline # optimize keyboard input, only need to import
import sys

"""
    Checkout available countries and models for Piper TTS
    print("Available countries:", tts.available_countrys(),end='\n\n')
    print("Available models for en_US:", tts.available_models('en_US'),end='\n\n')
"""

tts = Piper()
tts.set_model('en_US-amy-medium')

print('Please enter the sentence to TTS')

while True: 
    try:
        _result = input(f'\033[38;5;240m{"input: "}\033[0m').encode(sys.stdin.encoding).decode('utf-8')

        if _result == "":
            print() # new line
            continue
        else:
            print(f'\033[38;5;46m{"Speaking: "}{_result}\033[0m')
            tts.say(_result)

        sleep(.1)
    except KeyboardInterrupt:
        print('\nExiting...')
        break
    except Exception as e:
        print(f'\033[38;5;196m{"Error: "}{str(e)}\033[0m')
