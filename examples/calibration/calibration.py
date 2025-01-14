# https://github.com/jquast/blessed
# https://blessed.readthedocs.io/en/latest/
from blessed import Terminal

from zeus_pi import ZeusPi
import time
import threading
import random

# init ZeusPi
# ============================================================
my_car = ZeusPi()

# -----------------------------------------------------------------------------
mode = 0

motors_direction = list.copy(my_car.motors_direction)
cam_pan_offset = round(my_car.cam_pan_offset, 1)
cam_tilt_offset = round(my_car.cam_tilt_offset, 1)

MAX_SERVOS_OFFSET = 20.0
SERVOS_CALIBRATE_STEP = 0.2
MOTORS_TEST_POWER = 35

compass_offset = [0]*6

# Terminal
# ============================================================
term = Terminal()

CONTENT_WIDTH = 80
CONTENT_HEIGHT = 20

if (term.width < CONTENT_WIDTH) or (term.height < CONTENT_HEIGHT):
    print("Terminal size is too small")
    print(f"Terminal size: {term.width}x{term.height}")
    print(f"Required size: {CONTENT_WIDTH}x{CONTENT_HEIGHT}")
    exit()

time.sleep(.5)

# color 
THEME_COLOR = term.skyblue
THEME_BGROUND_COLOR = term.black
THEME_CHOSEN_COLOR = term.black_on_skyblue
THEME_UNCHOSEN_COLOR = term.white
'''
|\| ┌—-|-|—|-|--┐ |/|
|\|=| 3       1 |=|/|
|\| |           | |/| 
    |           |
|/| |           | |\|
|/|=| 4       2 |=|\|
|/| └———————————┘ |\|

'''
TITLE = "ZEUS PI CALIBRATION"
MODE_OPTIONS = {
    "location": (2, 2),
    "content": [
        "[1] Motors and Servos calibration ",
        "[2] Compass calibration ",
        "[3] Grayscale Module calibration ",
    ],
    'box_width': 35,
}
MODE_OPTIONS_TIPS = {
    "location": (CONTENT_WIDTH-22, 2),
    "content": [
        "[↑]      Select Up  ",
        "[↓]      Select Down",
        "[Enter]  OK",
        "[Esc]    Exit"
    ]
}

TITLE_MOTORS = "MOTORS AND SERVOS CALIBRATION"
MOTORS_OPTIONS = {
    "location": (2, 2),
    "content": [
        "[1] change Motor 0 (left-front) direction",
        "[2] change Motor 1 (right-front) direction",
        "[3] change Motor 2 (left-rear) direction",
        "[4] change Motor 3 (right-rear) direction",
        "[Q] motos run/stop",
        "",
        "[A/D] move camera pan servo",
        "[W/S] move camera tilt servo",
        "[R] servos test",
    ]
}

MOTORS_OPTIONS_TIPS = {
    "location": (CONTENT_WIDTH-18, 2),
    "content": [
        "[SPACE]   save",
        "[Esc]     Back",
        "[Ctrl+C]  Exit",
    ]
}

TITLE_COMPASS = "COMPASS CALIBRATION"
COMPASS_OPTIONS = {
    "location": (2, 2),
    "content": [
        "Please press the [Enter] key to start the calibration.",
        "The car will rotate and calibrate automatically.",
    ]
}
COMPASS_OPTIONS_TIPS = {
    "location": (CONTENT_WIDTH-18, 2),
    "content": [
        "[SPACE]   save",
        "[Esc]     Back",
        "[Ctrl+C]  Exit",
    ]
}

TITLE_GRAYSCALE = "GRAYSCALE MODULE CALIBRATION "
GRAYSCALE_OPTIONS = {
    "location": (2, 2),
    "content": [
        "[1] line reference calibration",
        "[2] cliff reference calibration",
    ]
}
GRAYSCALE_OPTIONS_TIPS = {
    "location": (CONTENT_WIDTH-22, 2),
    "content": [
        "[↑]      Select Up  ",
        "[↓]      Select Down",
        "[Enter]  OK",
        "[SPACE]  save",
        "[Esc]    Back",
        "[Ctrl+C] Exit",
    ]
}

ASK_SAVE = {
    "content": [
        '',
        'Confirm to save ? (y/n) ',
        ''
    ],
    "box_width": 50,
}

ASK_EXIT = {
    "content": [
        '',
        'Change not saved, confirm to exit? (y/n)',
        ''
    ],
    "box_width": 50,
}

# general functions
# ============================================================================
def draw_title(title):
    sapce = " "*int(CONTENT_WIDTH/2-len(title)/2)
    title = sapce + title + sapce
    print(term.home() + THEME_CHOSEN_COLOR(f'{title}'))

def draw(content, color, location=None, align='left', box_width=None):
    if location is None:
        _x, _y = term.get_location()
    else:
        _x, _y = location
    
    if not isinstance(content, list):
        content = [content]

    for i, line in enumerate(content):
        print(term.move_xy(_x, _y+i), end='')
        if box_width is None or len(line) >= box_width:
            print(color(f'{line}'), end='', flush=True)
        else:
            if align == 'left':
                space = " "*(box_width-len(line))
                print(color(f'{line}{space}'), end='', flush=True)
            elif align == 'right':
                space = " "*(box_width-len(line))
                print(color(f'{space}{line}'), end='', flush=True)
            elif align == 'center':
                space = " "*int((box_width-len(line))/2)
                print(color(f'{space}{line}{space}'), end='', flush=True)


def draw_options(content, 
                 selected_index,
                 selected_color,
                 unselected_color,
                 align='left',
                 box_width=None):

    _x, _y = content["location"]
    for i, line in enumerate(content["content"]):
        # location
        print(term.move_xy(_x, _y+i), end='')
        # color
        if i == selected_index:
            color = selected_color
        else:
            color = unselected_color

        # no fixed width
        if box_width is None or len(line) >= box_width:
            print(color(f'{line}'))
        # fixed width, align
        else:
            if align == 'left':
                space = " "*(box_width-len(line))
                print(color(f'{line}{space}'))
            elif align == 'right':
                space = " "*(box_width-len(line))
                print(color(f'{space}{line}'))
            elif align == 'center':
                space = " "*int((box_width-len(line))/2)
                print(color(f'{space}{line}{space}'))

def draw_ask(question,
             color=THEME_CHOSEN_COLOR,
             location=(0, term.height-1),
             align='left',
             box_width=None):

    with term.location():
        draw(question, color, location, align, box_width)

        while True:
            key = term.inkey()
            if key.lower() == 'y':
                return True
            elif key.lower() == 'n':
                return False
            elif key.name == 'KEY_ESCAPE':
                return False
            else:
                continue

def clear_line(location=(0, term.height-1)):
    with term.location():
        print(term.move_xy(*location), term.clear_eol, end='', flush=True)

def draw_bottom(content, color=THEME_UNCHOSEN_COLOR, align='left', box_width=None):
    if not isinstance(content, list):
        content = [content]
    with term.location():
        draw(content, color, (0, term.height-len(content)), align, box_width)

def clear_bottom(line=1):
    draw_bottom([' '*CONTENT_WIDTH]*line)

# mode select
# ============================================================================
def mode_select_handle():
    global mode

    # clear screen
    print(f"{term.home}{THEME_BGROUND_COLOR}{term.clear}")
    draw_title(TITLE)

    draw(MODE_OPTIONS_TIPS['content'],
        THEME_UNCHOSEN_COLOR, 
        MODE_OPTIONS_TIPS['location']
        )
    draw_options(MODE_OPTIONS,
                mode,
                THEME_CHOSEN_COLOR,
                THEME_UNCHOSEN_COLOR,
                align='left',
                box_width=MODE_OPTIONS['box_width']
                )

    while True:
        key = term.inkey()
        if key.name == 'KEY_UP':
            mode = mode - 1 if mode > 0 else len(MODE_OPTIONS['content']) - 1
        elif key.name == 'KEY_DOWN':
            mode = mode + 1 if mode < len(MODE_OPTIONS['content']) - 1 else 0
        elif key == '1':
            mode = 0
        elif key == '2':
            mode = 1
        elif key == '3':
            mode = 2
        elif key.name == 'KEY_ENTER':
            break
        elif key.name == 'KEY_ESCAPE':
            exit()
        else:
            continue

        draw_options(MODE_OPTIONS,
                        mode, 
                        THEME_CHOSEN_COLOR,
                        THEME_UNCHOSEN_COLOR,
                        align='left',
                        box_width=MODE_OPTIONS['box_width']
                        )

# motors_and_servos_calibration
# ============================================================================
def test_servos():
    my_car.set_cam_pan(-30)
    time.sleep(.2)
    my_car.set_cam_pan(30)
    time.sleep(.2)
    my_car.set_cam_pan(0)
    time.sleep(.2)
    my_car.set_cam_tilt(-30)
    time.sleep(.2)
    my_car.set_cam_tilt(30)
    time.sleep(.2)
    my_car.set_cam_tilt(0)

def motors_and_servos_calibration():
    global motors_direction, cam_pan_offset, cam_tilt_offset

    _has_saved = False
    _is_motors_run = False

    def show_static_content():
        # clear screen
        print(f"{term.home}{THEME_BGROUND_COLOR}{term.clear}")
        draw_title(TITLE_MOTORS)
        draw(MOTORS_OPTIONS_TIPS["content"], 
            THEME_UNCHOSEN_COLOR,
            MOTORS_OPTIONS_TIPS['location'])
        #
        draw(MOTORS_OPTIONS["content"], 
            THEME_UNCHOSEN_COLOR,
            MOTORS_OPTIONS['location'])
    
    show_static_content()
    # TODO: read from config file

    #
    cam_pan_offset = round(cam_pan_offset, 1)
    cam_tilt_offset = round(cam_tilt_offset, 1)
    _offset_obj = {
        "location": (2, 12),
        "content": [
            f"motors direction: {motors_direction} ",
            f"servos offset: {[cam_pan_offset, cam_tilt_offset]}"
        ],
        "box_width": 35,
    }

    def draw_offset():
        draw(_offset_obj["content"], 
            THEME_COLOR, 
            _offset_obj["location"],
            align='left',
            box_width=_offset_obj["box_width"])
    draw_offset()

    while True:
        key = term.inkey()
        if key.lower() in '1234adwsqr':
            clear_bottom()
            _has_saved = False
            _key = key.lower()
            if _key in '1234':
                if _key == '1':
                    motors_direction[0] = 0 if motors_direction[0] == 1 else 1
                elif _key == '2':
                    motors_direction[1] = 0 if motors_direction[1] == 1 else 1
                elif _key == '3':
                    motors_direction[2] = 0 if motors_direction[2] == 1 else 1
                elif _key == '4':
                    motors_direction[3] = 0 if motors_direction[3] == 1 else 1
                # change config
                my_car.set_motors_direction(motors_direction)
                if _is_motors_run:
                    my_car.move(0, MOTORS_TEST_POWER)
            elif _key == 'a':
                cam_pan_offset = cam_pan_offset - SERVOS_CALIBRATE_STEP
                if cam_pan_offset < -MAX_SERVOS_OFFSET:
                    cam_pan_offset = -MAX_SERVOS_OFFSET
                my_car.cam_pan.angle(cam_pan_offset*my_car.CAM_PAN_DIR)
            elif _key == 'd':
                cam_pan_offset = cam_pan_offset + SERVOS_CALIBRATE_STEP
                if cam_pan_offset > MAX_SERVOS_OFFSET:
                    cam_pan_offset = MAX_SERVOS_OFFSET
                my_car.cam_pan.angle(cam_pan_offset*my_car.CAM_PAN_DIR)
            elif _key == 'w':
                cam_tilt_offset = cam_tilt_offset - SERVOS_CALIBRATE_STEP
                if cam_tilt_offset < -MAX_SERVOS_OFFSET:
                    cam_tilt_offset = -MAX_SERVOS_OFFSET
                my_car.cam_tilt.angle(cam_tilt_offset*my_car.CAM_TILT_DIR)
            elif _key == 's':
                cam_tilt_offset = cam_tilt_offset + SERVOS_CALIBRATE_STEP
                if cam_tilt_offset > MAX_SERVOS_OFFSET:
                    cam_tilt_offset = MAX_SERVOS_OFFSET
                my_car.cam_tilt.angle(cam_tilt_offset*my_car.CAM_TILT_DIR)
            elif _key == 'r':
                my_car.set_cam_servos_offset([cam_pan_offset, cam_tilt_offset])
                test_servos()
            elif _key == 'q':
                if not _is_motors_run:
                    _is_motors_run = True
                    my_car.set_motors_direction(motors_direction)
                    my_car.move(0, MOTORS_TEST_POWER)
                else:
                    _is_motors_run = False
                    my_car.stop()
        elif key.name == 'KEY_ESCAPE':
            clear_bottom()
            if not _has_saved:
                _box_width = ASK_EXIT['box_width']
                if draw_ask(ASK_EXIT['content'], location=(int((CONTENT_WIDTH-_box_width)/2), 6), align='center', box_width=_box_width):
                    return
                else:
                    show_static_content()
                    draw_offset()
                    draw_bottom('Cancel.')
                    continue
            else:
                return
        elif key == ' ': # space
            clear_bottom()
            _box_width = ASK_SAVE['box_width']
            if draw_ask(ASK_SAVE['content'], location=(int((CONTENT_WIDTH-_box_width)/2), 6), align='center', box_width=_box_width):
                my_car.set_motors_direction(motors_direction)
                my_car.set_cam_servos_offset([cam_pan_offset, cam_tilt_offset])
                my_car.config.write() # write config to file
                #
                my_car.set_cam_pan(0)
                my_car.set_cam_tilt(0)
                #
                _has_saved = True
                show_static_content()
                draw_bottom('Saved.')
            else:
                _has_saved = False
                show_static_content()
                draw_bottom('Cancel.')
        else:
            continue
        #
        cam_pan_offset = round(cam_pan_offset, 1)
        cam_tilt_offset = round(cam_tilt_offset, 1)
        _offset_obj['content'] = [
            f"motors direction: {motors_direction} ",
            f"servos offset: {[cam_pan_offset, cam_tilt_offset]}"
        ]
        draw_offset()


# compass_calibration
# ============================================================================
compass_running = False
compass_data = [0, 0, 0, 0]
compass_offset_obj = {
    "location": (2, 6),
    "color": THEME_COLOR,
    "content": [
        f"compass offset: {compass_offset}",
    ],
    "box_width": 64,
}

compass_data_obj = {
    "location": (2, 7),
    "color": THEME_COLOR,
    "content": [
        f"compass data:",
        f"    x: {compass_data[0]:.2f} mGuass   y: {compass_data[1]:.2f} mGuass   z: {compass_data[2]:.2f} mGuass",
        f"    angle: {compass_data[3]}°"
    ],
    "box_width": 64,
}

def _draw_offset(obj):
    draw(obj["content"],
        obj['color'], 
        obj["location"],
        align='left',
        box_width=obj["box_width"])

def calibrate_compass_handler():
    global compass_offset, compass_offset_obj, compass_running
    
    _st = time.time()
    while compass_running:
        compass_offset = [random.randint(-32767, 32767) for _ in range(6)]
        compass_offset_obj['content'] = [
            f"compass offset: {compass_offset} "
        ]
        _draw_offset(compass_offset_obj)
        
        if (time.time() - _st) > 5:
            compass_running = False
            break

        time.sleep(0.2)

def compass_calibration():
    global compass_offset, compass_running
    _has_saved = False

    # TODO: read from config file

    def refresh_screen():
        # clear screen
        print(f"{term.home}{THEME_BGROUND_COLOR}{term.clear}")
        draw_title(TITLE_COMPASS)
        draw(COMPASS_OPTIONS_TIPS['content'],
            THEME_UNCHOSEN_COLOR, 
            COMPASS_OPTIONS_TIPS['location']
            )
        draw(COMPASS_OPTIONS['content'],
            THEME_UNCHOSEN_COLOR,
            COMPASS_OPTIONS['location']
            )
        _draw_offset(compass_offset_obj)
        _draw_offset(compass_data_obj)

    refresh_screen()
    while True:
        key = term.inkey(timeout=0.1)
        if key.name == 'KEY_ENTER':
            draw_bottom('Calibrating ... (press \'q\' to stop)',
                        THEME_CHOSEN_COLOR,
                        align='left',
                        box_width=CONTENT_WIDTH,
                        )
            compass_running = True
            t = threading.Thread(target=calibrate_compass_handler)
            t.daemon = True
            t.start()
            while True:
                # 
                if not compass_running:
                    clear_bottom()
                    draw_bottom('Calibration finished.')
                    break
                # time.sleep(1)
                key = term.inkey(timeout=0.1)
                if key.lower() == 'q':
                    compass_running = False
                    t.join()
                    clear_bottom()
                    draw_bottom('Cancel.')
                    break
        elif key.name == 'KEY_ESCAPE':
            clear_bottom()
            if not _has_saved:
                _box_width = ASK_EXIT['box_width']
                if draw_ask(ASK_EXIT['content'], location=(int((CONTENT_WIDTH-_box_width)/2), 6), align='center', box_width=_box_width):
                    return
                else:
                    refresh_screen()
                    draw_bottom('Cancel.')
                    continue
            else:
                return
        elif key == ' ': # space
            clear_bottom()
            _box_width = ASK_SAVE['box_width']
            if draw_ask(ASK_SAVE['content'], location=(int((CONTENT_WIDTH-_box_width)/2), 6), align='center', box_width=_box_width):
                # TODO: save to config file

                _has_saved = True
                refresh_screen()
                draw_bottom('Saved.')
            else:
                _has_saved = False
                refresh_screen()
                draw_bottom('Cancel.')

        # update data

        compass_data = [random.randint(-180, 180) for _ in range(4)]
        compass_data_obj['content'] = [
            f"compass data:",
            f"    x: {compass_data[0]:.2f} mGuass   y: {compass_data[1]:.2f} mGuass   z: {compass_data[2]:.2f} mGuass",
            f"    angle: {compass_data[3]}°"
        ]
        _draw_offset(compass_data_obj)


# grayscale_module_calibration
# ============================================================================
grayscale_date = [0, 0, 0]
line_reference = [0, 0, 0]
cliff_reference = [0, 0, 0]
grayscle_extremum = [
    [4095, 0],
    [4095, 0],
    [4095, 0],
    [4095, 0],
]
grayscale_running = False

grayscale_date_obj = {
    'location': (2, 7),
    'color': THEME_COLOR,
    'content': [
        f"grayscale data: {grayscale_date}",
    ],
    'box_width': 35
}
grayscale_extremum_obj = {
    'location': (2, 8),
    'color': THEME_COLOR,
    'content': [
        f"extremum: {grayscle_extremum}",
    ],
    'box_width': 70
}

grayscale_reference_obj = {
    'location': (2, 9),
    'color': THEME_COLOR,
    'content': [
        f"line reference: {line_reference}",
        f"cliff reference: {cliff_reference}",
    ],
    'box_width': 35
}

LINE_REF_CALI_TIPS = {
    'content': [
        "",
        "Please place the Zeus Pi car in the middle of the line, ",
        "and press [Enter] to start automatic calibration. ",
        "",
    ],
    'location': (int((CONTENT_WIDTH-60)/2), 5),
    'box_width': 60,
    'color': THEME_CHOSEN_COLOR,
}

def line_reference_calibrate_handler():
    global line_reference, grayscale_date, grayscale_extremum, grayscale_running

    # 0 ~ 3s, left
    # 3 ~ 6s, right
    # 7 ~ 10s, left
    # > 10s stop
    _st = time.time()
    while grayscale_running:
        if time.time() - _st > 1:
            _st = time.time()
            grayscale_date = [random.randint(0, 4095) for _ in range(3)]
        
        grayscale_date = [random.randint(0, 4095) for _ in range(3)]



def grayscale_module_calibration():
    global grayscale_date, line_reference, cliff_reference, grayscle_extremum

    _has_saved = False
    _mode = 0
    _last_mode = 0
    # TODO: read from config file

    def refresh_screen():
        # clear screen
        print(f"{term.home}{THEME_BGROUND_COLOR}{term.clear}")
        draw_title(TITLE_GRAYSCALE)
        draw(GRAYSCALE_OPTIONS_TIPS['content'],
            THEME_UNCHOSEN_COLOR, 
            GRAYSCALE_OPTIONS_TIPS['location']
            )
        draw_options(GRAYSCALE_OPTIONS,
                     _mode,
                    THEME_CHOSEN_COLOR, 
                    THEME_UNCHOSEN_COLOR,
                    align='left',
                    box_width=32
                    )
        _draw_offset(grayscale_date_obj)
        _draw_offset(grayscale_extremum_obj)
        _draw_offset(grayscale_reference_obj)

    refresh_screen()
    while True:
        key = term.inkey(timeout=0.1)
        print(key)
        if key.name == 'KEY_UP':
            _mode = _mode - 1 if _mode > 0 else len(GRAYSCALE_OPTIONS['content']) - 1
        elif key.name == 'KEY_DOWN':
            _mode = _mode + 1 if _mode < len(GRAYSCALE_OPTIONS['content']) - 1 else 0
        elif key == '1':
            _mode = 0
        elif key == '2':
            _mode = 1
        elif key.name == 'KEY_ENTER':
            if _mode == 0:
                draw(LINE_REF_CALI_TIPS['content'],
                     color=LINE_REF_CALI_TIPS['color'],
                     location=LINE_REF_CALI_TIPS['location'],
                     align='center',
                     box_width=LINE_REF_CALI_TIPS['box_width'],
                     )
                while True:
                    key = term.inkey(timeout=0.1)
                    if key.name == 'KEY_ENTER':
                        refresh_screen()
                        draw_bottom('line reference calibrating ... (press \'q\' to stop)',
                                    THEME_CHOSEN_COLOR,
                                    align='left',
                                    box_width=CONTENT_WIDTH,
                                    )
                        while True:
                            key = term.inkey(timeout=0.1)
                            if key.lower() == 'q':
                                # compass_running = False
                                # t.join()
                                clear_bottom()
                                draw_bottom('Cancel.')
                                break
                        break
            elif _mode == 1:
                # TODO:
                pass
        elif key.name == 'KEY_ESCAPE':
            clear_bottom()
            if not _has_saved:
                _box_width = ASK_EXIT['box_width']
                if draw_ask(ASK_EXIT['content'], location=(int((CONTENT_WIDTH-_box_width)/2), 6), align='center', box_width=_box_width):
                    return
                else:
                    refresh_screen()
                    draw_bottom('Cancel.')
                    continue
            else:
                return
        elif key == ' ': # space
            clear_bottom()
            _box_width = ASK_SAVE['box_width']
            if draw_ask(ASK_SAVE['content'], location=(int((CONTENT_WIDTH-_box_width)/2), 6), align='center', box_width=_box_width):
                # TODO: save to config file
                _has_saved = True
                refresh_screen()
                draw_bottom('Saved.')
            else:
                _has_saved = False
                refresh_screen()
                draw_bottom('Cancel.')
        # ------------------
        if _mode != _last_mode:
            _last_mode = _mode
            draw_options(GRAYSCALE_OPTIONS,
                        _mode,
                        THEME_CHOSEN_COLOR, 
                        THEME_UNCHOSEN_COLOR,
                        align='left',
                        box_width=32
                        )
        # ------------------
        grayscale_date = [random.randint(0, 4096) for _ in range(3)]
        grayscale_date_obj['content'] = [
            f"grayscale_date: {grayscale_date}",
        ]
  
        _draw_offset(grayscale_date_obj)


# main
# ============================================================================
def loop():
    #
    mode_select_handle()
    #
    if mode == 0:
        motors_and_servos_calibration()
    elif mode == 1:
        compass_calibration()
    elif mode == 2:
        grayscale_module_calibration()
    else:
        pass


def main():
    with term.fullscreen(), term.cbreak():
        while True:
            loop()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        my_car.reset()
