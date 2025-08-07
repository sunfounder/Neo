from openai_helper import OpenAiHelper
from keys import OPENAI_API_KEY, OPENAI_ASSISTANT_ID
from utils import *
import time

openai_helper = OpenAiHelper(OPENAI_API_KEY, OPENAI_ASSISTANT_ID, 'Neo')

VOLUME_DB = 3
TTS_VOICE = 'echo'

txt = "Oh, it says 'Hello!' Just like the sound of a friendly horn!"

st = time.time()

_time = time.strftime("%y-%m-%d_%H-%M-%S", time.localtime())
_tts_f = f"./tts/{_time}_raw.wav"
_tts_status = openai_helper.text_to_speech(txt, _tts_f, TTS_VOICE, response_format='wav') # alloy, echo, fable, onyx, nova, and shimmer
if _tts_status:
    tts_file = f"./tts/{_time}_{VOLUME_DB}dB.wav"
    _tts_status = sox_volume(_tts_f, tts_file, VOLUME_DB)

print(f'save as: {tts_file}')
gray_print(f'tts takes: {time.time() - st:.3f} s')

import os
while True:
    os.system(f'sudo play {tts_file}')
    time.sleep(2)
