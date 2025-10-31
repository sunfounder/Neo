# https://github.com/jquast/blessed
# https://blessed.readthedocs.io/en/latest/
from blessed import Terminal

from neo import Neo
import time
import threading
import random

# init Neo
# ============================================================
# TODO:长时间不进行操作的时候，会出现音频ALSA lib错误。因为Neo初始化的音频功能会占用资源。
#      可以在初始化时禁用音频功能，或者在使用完后及时关闭音频，尚且不知道需不需要改。
my_car = Neo()

my_car.set_cam_pan(-10)
my_car.set_cam_tilt(-10)
time.sleep(.5)
my_car.set_cam_pan(0)
my_car.set_cam_tilt(0)
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
TITLE = "PI_NEOCAR CALIBRATION"
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
    # read from config file
    try:
        motors_direction = my_car.config.get('motors_direction', my_car.DEFAULT_MOTORS_DIRECTION).copy()
        cam_pan_offset = my_car.config.get('servos_offset', [0.0, 0.0])[0]
        cam_tilt_offset = my_car.config.get('servos_offset', [0.0, 0.0])[1]
 
        config_values = [ 
            f"motors direction: {motors_direction}",                              
            f"servos offset: {[cam_pan_offset, cam_tilt_offset]}"
        ]

        draw_bottom('Config loaded successfully.')
        time.sleep(.5) # Persistence of vision
        clear_bottom()
        draw_bottom(config_values)
        time.sleep(1)
        clear_bottom(line = len(config_values))
        
    except Exception as e:
        draw_bottom(f"Config read failed: {str(e)}. Using default values.")

    # get param from config/default and display
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
                try:
                    my_car.set_motors_direction(motors_direction)
                    my_car.set_cam_servos_offset([cam_pan_offset, cam_tilt_offset])
                    my_car.set_cam_pan(0)
                    my_car.set_cam_tilt(0)
                    _has_saved = True
                    show_static_content()
                    draw_bottom(f'saved: dir={motors_direction}, offset=[{cam_pan_offset},{cam_tilt_offset}]')
                    time.sleep(.5)
                    clear_bottom()
                except Exception as e:
                    _has_saved = False
                    draw_bottom(f"Save failed: {str(e)}.")
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
on_compass_calibrating = False
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
    global compass_offset, compass_offset_obj, on_compass_calibrating, x_min, x_max, y_min, y_max, z_min, z_max
    # value Init 
    x_min = 0
    x_max = 0
    y_min = 0
    y_max = 0
    z_min = 0
    z_max = 0

    _st = time.time()
    while on_compass_calibrating:
        x_raw, y_raw, z_raw = my_car.read_compass_raw()

        _changed = False
        if x_raw < x_min:
            x_min = x_raw
            _changed = True
        elif x_raw > x_max:
            x_max = x_raw
            _changed = True
        
        if y_raw < y_min:
            y_min = y_raw
            _changed = True
        elif y_raw > y_max:
            y_max = y_raw
            _changed = True

        if z_raw < z_min:
            z_min = z_raw
            _changed = True
        elif z_raw > z_max:
            z_max = z_raw
            _changed = True

        if _changed:
            st = time.time()
        elif time.time() - st > 3:
            my_car.set_compass_offset(x_min, x_max, y_min, y_max, z_min, z_max)
            on_compass_calibrating = False
            my_car.stop()

        compass_offset_obj['content'] = [
            f"compass offset: {x_min, x_max, y_min, y_max, z_min, z_max} "
        ]
        _draw_offset(compass_offset_obj)
        time.sleep(.01)

def compass_calibration():
    global compass_offset, on_compass_calibrating, x_min, x_max, y_min, y_max, z_min, z_max
    # Initialize all global variables to ensure they can be accessed even if the calibration process is not run
    x_min = 0
    x_max = 0
    y_min = 0
    y_max = 0
    z_min = 0
    z_max = 0
    _has_saved = False

    # read from config file
    try:
        # Get the compass offset configuration value directly from my_car.config
        compass_offset = my_car.config.get('compass_offset', [0]*6)
        config_values = [
            f"compass offset: {compass_offset}"
        ]
        draw_bottom('Compass config loaded successfully.')
        time.sleep(.5) # Persistence of vision
        
        draw_bottom(config_values)
        time.sleep(1)
        clear_bottom(line = len(config_values))
    except Exception as e:
        draw_bottom(f"Compass config read failed: {str(e)}. Using default values.")
        compass_offset = [0]*6  # default 
        
    # update
    compass_offset_obj['content'] = [
        f"compass offset: {compass_offset}",  
    ]

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
            on_compass_calibrating = True
            t = threading.Thread(target=calibrate_compass_handler)
            t.daemon = True
            t.start()
            while True:
                # 
                if not on_compass_calibrating:
                    clear_bottom()
                    draw_bottom('Calibration finished.')
                    break
                # time.sleep(1)
                key = term.inkey(timeout=0.1)
                if key.lower() == 'q':
                    on_compass_calibrating = False
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
            # Width of the calibration data save confirmation dialog
            _box_width = ASK_SAVE['box_width'] 
            calibration_values = [x_min, x_max, y_min, y_max, z_min, z_max]
            # if x_min == 0 and x_max == 0 and y_min == 0 and y_max == 0 and z_min == 0 and z_max == 0
            if all(value == 0 for value in calibration_values):
                clear_bottom()
                draw_bottom('No calibration data. Using default values.')
                time.sleep(.5)
                clear_bottom()
                continue
            elif draw_ask(ASK_SAVE['content'], location=(int((CONTENT_WIDTH-_box_width)/2), 6), align='center', box_width=_box_width):
                try:
                    my_car.set_compass_offset(x_min, x_max, y_min, y_max, z_min, z_max)
                    _has_saved = True
                    refresh_screen()
                    draw_bottom(f'save success! compass offset: [{x_min}, {x_max}, {y_min}, {y_max}, {z_min}, {z_max}]')
                    time.sleep(.5)
                    clear_bottom()
                except Exception as e:
                    _has_saved = False
                    refresh_screen()
                    draw_bottom(f'compass offset save failed: {str(e)}')
                    raise
            else:
                _has_saved = False
                refresh_screen()
                draw_bottom('Cancel.')

        # update data
        compass_data = my_car.read_compass()
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
grayscale_threshold = [
    [4095, 0],  # [min, max]
    [4095, 0],
    [4095, 0],
]
grayscale_running_flag = False

grayscale_date_obj = {
    'location': (2, 7),
    'color': THEME_COLOR,
    'content': [
        f"grayscale_data: {grayscale_date}",
    ],
    'box_width': 35
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
        "Please place the Neo Pi car in the middle of the line, ",
        "and press [Enter] to start automatic calibration. ",
        "",
    ],
    'location': (int((CONTENT_WIDTH-60)/2), 5),
    'box_width': 60,
    'color': THEME_CHOSEN_COLOR,
}

# Grayscale sensor data reading loop
# TODO:在line reference calibration执行结束之后，没有关闭这个线程，导致没办法继续save操作,但是可以继续回车，需要完善
def read_grayscale_data_loop():
    global grayscale_date, grayscale_threshold, grayscale_running_flag, line_reference
    
    while grayscale_running_flag:
        try:
            # Read the actual grayscale sensor data
            grayscale_reading, _ = my_car.read_grayscale()
            grayscale_date = grayscale_reading
            
            # Update the threshold record
            for i in range(3):
                # Update max and min values
                if grayscale_date[i] < grayscale_threshold[i][0]:
                    grayscale_threshold[i][0] = grayscale_date[i]
                if grayscale_date[i] > grayscale_threshold[i][1]:
                    grayscale_threshold[i][1] = grayscale_date[i]
                # Calculate line reference as average of threshold
                line_reference[i] = int((grayscale_threshold[i][0] + grayscale_threshold[i][1]) / 2)
        except Exception as e:
            draw_bottom(f"Error: {e}")
        
        time.sleep(.2)  # Read data every 0.2 seconds

""""-------------------line_reference_calibrate_handler-------------------"""
def line_reference_calibrate_handler():
    global line_reference, grayscale_date, grayscale_threshold, grayscale_running_flag
    global cliff_reference
    
    # Reset threshold
    grayscale_threshold = [
        [4095, 0], 
        [4095, 0],
        [4095, 0],
    ]
    
    
    grayscale_running_flag = True
    grayscale_data_thread = threading.Thread(target=read_grayscale_data_loop)  
    grayscale_data_thread.daemon = True
    grayscale_data_thread.start()
    
    try:
        _angle = 35
        _delay = 2
        _power = 30
        
        # turn left
        draw_bottom('Moving left forward...')
        my_car.set_cam_pan(-_angle)
        my_car.move(0, _power, _power)  
        time.sleep(_delay)
        
        # turn right
        draw_bottom('Moving left backward...')
        my_car.move(180, _power, _power)  
        time.sleep(_delay)
        
        # stop
        my_car.stop()
        my_car.set_cam_pan(0)
        time.sleep(.2)
        
        # move forward
        draw_bottom('Moving right forward...')
        my_car.set_cam_pan(_angle)
        my_car.move(0, _power, -_power)
        time.sleep(_delay)
        
        # move backward
        draw_bottom('Moving right backward...')
        my_car.move(180, _power, -_power)
        time.sleep(_delay)
        
        # stop
        my_car.set_cam_pan(0)
        my_car.stop()
        time.sleep(.2)
        
        line_reference = [
            int((grayscale_threshold[0][0] + grayscale_threshold[0][1]) / 2),
            int((grayscale_threshold[1][0] + grayscale_threshold[1][1]) / 2),
            int((grayscale_threshold[2][0] + grayscale_threshold[2][1]) / 2),
        ]
        
        if all(cliff_reference[i] < line_reference[i] for i in range(3)):
            cliff_reference = [
                int((cliff_reference[0] + line_reference[0]) / 2),
                int((cliff_reference[1] + line_reference[1]) / 2),
                int((cliff_reference[2] + line_reference[2]) / 2),
            ]
        
        draw_bottom('Line reference calibration completed!')
        time.sleep(1)
        
    except Exception as e:
        draw_bottom(f'Calibration error: {str(e)}')
        time.sleep(1)
    finally:
        my_car.stop()
        # wait for thread to finish
        grayscale_data_thread.join(timeout=1.0)

def cliff_reference_calibrate_handler():
    global cliff_reference, grayscale_date, grayscale_threshold, grayscale_running_flag
    
    try:
        draw_bottom('Cliff reference calibrating...')
        
        count = 0
        _left_val = 0
        _mid_val = 0
        _right_val = 0
        
        # collect 10 grayscale data
        while count < 10 and grayscale_running_flag:
            grayscale_reading, _ = my_car.read_grayscale()
            _left_val += grayscale_reading[0]
            _mid_val += grayscale_reading[1]
            _right_val += grayscale_reading[2]
            count += 1
            time.sleep(.2)
        
        # average
        _left_val /= 10
        _mid_val /= 10
        _right_val /= 10
        
        if _left_val < grayscale_threshold[0][1] and _mid_val < grayscale_threshold[1][1] and _right_val < grayscale_threshold[2][1]:
            _left_val = int((_left_val + grayscale_threshold[0][1]) / 2)
            _mid_val = int((_mid_val + grayscale_threshold[1][1]) / 2)
            _right_val = int((_right_val + grayscale_threshold[2][1]) / 2)
        
        cliff_reference = [int(_left_val), int(_mid_val), int(_right_val)]
        
        draw_bottom('Cliff reference calibration completed!')
        draw_bottom(f'Cliff reference: {cliff_reference}')
        time.sleep(2)
        
    except Exception as e:
        draw_bottom(f'Cliff calibration error: {str(e)}')
        time.sleep(1)

'''---------Grayscale Module Calibration---------'''
def grayscale_module_calibration():
    global grayscale_date, line_reference, cliff_reference, grayscale_running_flag

    _has_saved = False
    _mode = 0   # 0-line_cail, 1-cliff_cail
    _last_mode = 0


    # read from config file and show
    try:
        line_reference = my_car.config.get('line_reference', [0]*3)
        cliff_reference = my_car.config.get('cliff_reference', [0]*3)
        config_values = [
            f"Line reference: {line_reference}",
            f"Cliff reference: {cliff_reference}"
        ]
        
        draw_bottom('Config loaded successfully.')
        time.sleep(.5) # Persistence of vision
        
        # print config
        draw_bottom(config_values)
        time.sleep(1)
        clear_bottom(line = len(config_values))

    except Exception as e:
        draw_bottom(f"Config read failed: {str(e)}. Using default values.")
        time.sleep(2) 
        clear_bottom()



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
        _draw_offset(grayscale_reference_obj)

    refresh_screen()
    while True:
        key = term.inkey(timeout=0.1)
        # print(key) 
        if key.name == 'KEY_UP':
            _mode = _mode - 1 if _mode > 0 else len(GRAYSCALE_OPTIONS['content']) - 1
        elif key.name == 'KEY_DOWN':
            _mode = _mode + 1 if _mode < len(GRAYSCALE_OPTIONS['content']) - 1 else 0
        elif key == '1':
            _mode = 0
        elif key == '2':
            _mode = 1
        elif key.name == 'KEY_ENTER':
            # mode 0 : line reference calibration
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
                        draw_bottom('Line reference calibrating ... (press \'q\' to stop)',
                                    THEME_CHOSEN_COLOR,
                                    align='left',
                                    box_width=CONTENT_WIDTH,
                                    )
                        
                        #line_reference_calibrate
                        global grayscale_running_flag
                        grayscale_running_flag = True
                        calibration_thread = threading.Thread(target=line_reference_calibrate_handler)
                        calibration_thread.daemon = True    
                        calibration_thread.start()
                        
                        # wait for calibration thread to finish
                        while calibration_thread.is_alive() and grayscale_running_flag:
                            key = term.inkey(timeout=0.1)
                            if key.lower() == 'q':
                                grayscale_running_flag = False
                                break
                            # update screen
                            refresh_screen()
                            draw_bottom('Line reference calibrating ... (press \'q\' to stop)',
                                        THEME_CHOSEN_COLOR,
                                        align='left',
                                        box_width=CONTENT_WIDTH,
                                        )
                        
                        # wait for calibration thread to finish
                        calibration_thread.join(timeout=1.0)
                        
                        # reset
                        grayscale_running_flag = False
                        
                        # my_car.set_line_reference(line_reference)   # save
                        refresh_screen()
                        draw_bottom('Line reference: ' + str(line_reference))

                        time.sleep(1)
                        clear_bottom()

            # mode 1 : cliff reference calibration  
            elif _mode == 1:
                refresh_screen()
                draw_bottom('Cliff reference calibrating ... (press \'q\' to stop)',
                            THEME_CHOSEN_COLOR,
                            align='left',
                            box_width=CONTENT_WIDTH,
                            )
                
                # Start cliff reference calibration
                grayscale_running_flag = True
                calibration_thread = threading.Thread(target=cliff_reference_calibrate_handler)
                calibration_thread.daemon = True    
                calibration_thread.start()
                
                # wait for calibration thread to finish
                while calibration_thread.is_alive() and grayscale_running_flag:
                    key = term.inkey(timeout=0.1)
                    if key.lower() == 'q':
                        grayscale_running_flag = False
                        break
                    # update screen
                    refresh_screen()
                    draw_bottom('Cliff reference calibrating ... (press \'q\' to stop)',
                                THEME_CHOSEN_COLOR,
                                align='left',
                                box_width=CONTENT_WIDTH,
                                )
                
                # wait for calibration thread to finish
                calibration_thread.join(timeout=1.0)
                
                # reset
                grayscale_running_flag = False

                # check cliff_reference
                if not (isinstance(cliff_reference, list) and len(cliff_reference) == 3):
                    draw_bottom('Warning: Cliff reference must be a 1*3 list. Using default values.')
                    time.sleep(1)
                    cliff_reference = [0, 0, 0]  # make sure it's a valid list
                else:
                    # update screen and save config
                    refresh_screen()
                    draw_bottom('Cliff reference: ' + str(cliff_reference))
                
                time.sleep(1)
                clear_bottom(line = 1)
        
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
                try:
                    # add check data_validity
                    _is_valid = True
                    for i in range(3):
                        if line_reference[i] <= cliff_reference[i]:
                            _is_valid = False
                            break
                    if _is_valid:
                        my_car.set_line_reference(line_reference)
                        my_car.set_cliff_reference(cliff_reference)
                        _has_saved = True
                        refresh_screen()
                        draw_bottom(f'save! line_ref: {line_reference}, cliff_ref: {cliff_reference}')
                        time.sleep(.5)
                        clear_bottom()
                    else:
                        draw_bottom('Note that cliff reference values should be less than line reference values.')
                        time.sleep(1)
                        clear_bottom()
                except Exception as e:
                    _has_saved = False
                    refresh_screen()
                    draw_bottom(f'save failed: {str(e)}')
                    raise

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
        # show
        try:
            grayscale_reading, _ = my_car.read_grayscale()
            grayscale_date = grayscale_reading
            grayscale_date_obj['content'] = [f"grayscale_data: {grayscale_date}"]
            # update line_reference and cliff_reference
            grayscale_reference_obj['content'] = [
                f"line reference: {line_reference}",
                f"cliff reference: {cliff_reference}",
            ]
        except Exception as e:
            pass
  
        _draw_offset(grayscale_date_obj)
        _draw_offset(grayscale_reference_obj)


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
        # grayscale_module_calibration_under_construction()
    else:
        pass


def main():
    with term.fullscreen(), term.cbreak(), term.hidden_cursor():
        while True:
            loop()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        my_car.reset()
