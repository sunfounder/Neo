# https://github.com/jquast/blessed
# https://blessed.readthedocs.io/en/latest/
from blessed import Terminal

from neo import Neo
import time
import threading
import random

from fusion_hat.modules import Grayscale_Module,LineTracker
from fusion_hat.adc import ADC

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
TITLE = "NEOCAR CALIBRATION"
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
        "[1] Get White Line Threshold",
        "[2] Get Black Line Threshold",
    ]
}
GRAYSCALE_OPTIONS_TIPS = {
    "location": (CONTENT_WIDTH-18, 2),
    "content": [
        "[SPACE]   save",
        "[Esc]     Back",
        "[Ctrl+C]  Exit",
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
grayscale_running_flag = False

grayscale_date_obj = {
    'location': (2, 7),
    'color': THEME_COLOR,
    'content': [
        f"Raw grayscale data: {grayscale_date}",
    ],
    'box_width': 35
}

grayscale_reference_obj = {
    'location': (2, 9),
    'color': THEME_COLOR,
    'content': [
        "Calibration slopes: [1, 1, 1]",
        "Calibration offsets: [0, 0, 0]",
    ],
    'box_width': 35
}

# Grayscale sensor data reading loop
def read_grayscale_data_loop():
    global grayscale_date, grayscale_running_flag
    temp_line_tracker = LineTracker(ADC(0), ADC(1), ADC(2))

    try:     
        while grayscale_running_flag:
            raw_data = temp_line_tracker.read(raw=True)
            if raw_data and len(raw_data) >= 3:
                grayscale_date = raw_data
            time.sleep(0.05)
    except Exception as e:
        draw_bottom(f'grayscale sensor read failed: {str(e)}')
        grayscale_date = [0, 0, 0]

def set_line_tracker_calibration_data(line_tracker, slopes, offsets):
    if hasattr(line_tracker, '_slopes'):
        line_tracker._slopes = slopes
    if hasattr(line_tracker, 'slopes'):
        line_tracker.slopes = slopes
    if hasattr(line_tracker, '_offsets'):
        line_tracker._offsets = offsets
    if hasattr(line_tracker, 'offsets'):
        line_tracker.offsets = offsets
    return True

def get_line_tracker_calibration_data(line_tracker):
    slopes = None
    for attr_name in ['_slopes', 'slopes']:
        if hasattr(line_tracker, attr_name):
            slopes = getattr(line_tracker, attr_name)
            break
    
    offsets = None
    for attr_name in ['_offsets', 'offsets']:
        if hasattr(line_tracker, attr_name):
            offsets = getattr(line_tracker, attr_name)
            break
    
    return slopes, offsets

def line_tracker_calibrate_handler(line_tracker, option):
    
    slopes = None
    offsets = None
    calibration_data = {}
    
    if not hasattr(line_tracker_calibrate_handler, '_calibration_data'):
        line_tracker_calibrate_handler._calibration_data = {
            'white_data': None,
            'black_data': None
        }
    
    existing_data = line_tracker_calibrate_handler._calibration_data
    
    try:
        if option == 1:  # white surface
            clear_bottom()
            draw_bottom(['Please place the car on a WHITE surface.', 'Press Enter to continue...'], align='center')

            while True:
                key = term.inkey(timeout=0.1)
                if key.name == 'KEY_ENTER':
                    clear_bottom(line=2)
                    break

            white_data = [0, 0, 0]
            clear_bottom()
            draw_bottom(['Collecting white surface data...'], align='center')
            
            for _ in range(10):
                try:
                    raw_data = line_tracker.read(raw=True)
                    if raw_data and len(raw_data) >= 3:
                        for j in range(3):
                            white_data[j] += raw_data[j]
                except Exception as e:
                    raise Exception(f'Grayscale sensor read failed: {str(e)}')
                time.sleep(0.1)
            
            white_data = [int(val / 10) for val in white_data]
            calibration_data['white_data'] = white_data
            existing_data['white_data'] = white_data
            
            clear_bottom()
            draw_bottom([f'White surface data: {white_data}', 'Press Enter to continue...'], align='center')           
 
            while True:
                key = term.inkey(timeout=0.1)
                if key.name == 'KEY_ENTER':
                    clear_bottom(line=2)
                    break
            
            if existing_data['black_data'] is not None:
                clear_bottom()
                draw_bottom(['Both surfaces collected!', 'Performing calibration...'], align='center')
                time.sleep(.5)
                
                try:
                    slopes, offsets = line_tracker.calibrate(existing_data['white_data'], existing_data['black_data'])

                    set_line_tracker_calibration_data(line_tracker, slopes, offsets)
                    
                    clear_bottom()
                    draw_bottom(['Calibration completed!', f'Slopes: {slopes}', f'Offsets: {offsets}'], align='center')
                    time.sleep(1)  
                    
                except Exception as e:
                    clear_bottom()
                    draw_bottom([f'Calibration calculation failed: {str(e)}'], align='center')
                    time.sleep(1)
        
        elif option == 2:  # black surface
            clear_bottom()
            draw_bottom(['Please place the car on a BLACK surface.', 'Press Enter to continue...'], align='center')
            
            while True:
                key = term.inkey(timeout=0.1)
                if key.name == 'KEY_ENTER':
                    clear_bottom(line=2)
                    break
            
            black_data = [0, 0, 0]
            clear_bottom()
            draw_bottom(['Collecting black surface data...'], align='center')
            
            for _ in range(10):
                try:
                    raw_data = line_tracker.read(raw=True)
                    if raw_data and len(raw_data) >= 3:
                        for j in range(3):
                            black_data[j] += raw_data[j]
                except Exception:
                    pass
                time.sleep(0.1)
            
            black_data = [int(val / 10) for val in black_data]
            calibration_data['black_data'] = black_data
            existing_data['black_data'] = black_data
            
            clear_bottom()
            draw_bottom([f'Black surface data: {black_data}', 'Press Enter to continue...'], align='center')
            
            while True:
                key = term.inkey(timeout=0.1)
                if key.name == 'KEY_ENTER':
                    clear_bottom(line=2)
                    break
            
            if existing_data['white_data'] is not None:
                clear_bottom()
                draw_bottom(['Both surfaces collected!', 'Performing calibration...'], align='center')
                time.sleep(.5)
                
                try:
                    slopes, offsets = line_tracker.calibrate(existing_data['white_data'], existing_data['black_data'])
                    
                    set_line_tracker_calibration_data(line_tracker, slopes, offsets)
                    
                    clear_bottom()
                    draw_bottom(['Calibration completed!', f'Slopes: {slopes}', f'Offsets: {offsets}'], align='center')
                    time.sleep(1) 
                    
                except Exception as e:
                    clear_bottom()
                    draw_bottom([f'Calibration calculation failed: {str(e)}'], align='center')
                    time.sleep(1)
        
    except Exception as e:
        clear_bottom()
        draw_bottom([f'Error during calibration: {str(e)}', 'Press Enter to continue...'], align='center')
        while True:
            key = term.inkey(timeout=0.1)
            if key.name == 'KEY_ENTER':
                clear_bottom(line=2)
                break
    
    clear_bottom()

    return slopes, offsets, calibration_data

def grayscale_module_calibration():
    global line_tracker,grayscale_date, grayscale_running_flag

    # Initialize LineTracker with ADC channels
    line_tracker = LineTracker(ADC(0), ADC(1), ADC(2))
    
    # Calibration data
    slopes = [1, 1, 1]
    offsets = [0, 0, 0]
    
    # Two-step calibration support
    calibration_data = {
        'white_data': None,
        'black_data': None
    }
    
    _has_saved = False
    _mode = 0
    _last_mode = 0

    grayscale_running_flag = False

    # read from config file and show
    try:
        # Try to load existing calibration data if available
        slopes = my_car.config.get('line_tracker_slopes', [1, 1, 1])
        offsets = my_car.config.get('line_tracker_offsets', [0, 0, 0])
        
        set_line_tracker_calibration_data(line_tracker, slopes, offsets)
        
        # read from config file and show
        config_values = [
            f"Slopes: {slopes}",
            f"Offsets: {offsets}"
        ]
        
        draw_bottom('Config loaded successfully.')
        time.sleep(.5)  
        clear_bottom()
        
        draw_bottom(config_values)
        time.sleep(1)
        clear_bottom(line=len(config_values))

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
        
        # Display collected data status
        status_lines = []
        if calibration_data['white_data'] is not None:
            status_lines.append(f"White surface data: {calibration_data['white_data']}")
        if calibration_data['black_data'] is not None:
            status_lines.append(f"Black surface data: {calibration_data['black_data']}")
        
        if status_lines:
            clear_bottom(line=2)
            status_obj = {
                'location': (2, 12),
                'color': THEME_COLOR,
                'content': status_lines,
                'box_width': 35
            }
            _draw_offset(status_obj)

    # read grayscale data loop action
    grayscale_running_flag = True
    grayscale_data_thread = threading.Thread(target=read_grayscale_data_loop)
    grayscale_data_thread.daemon = True
    grayscale_data_thread.start()

    refresh_screen()
    while True:
        key = term.inkey(timeout=0.1)
        
        if key.name == 'KEY_UP':
            _mode = 0  # White thread
            refresh_screen()
        elif key.name == 'KEY_DOWN':
            _mode = 1  # Black thread
            refresh_screen()
        elif key.name == 'KEY_ENTER':        
            new_slopes, new_offsets, new_cal_data = line_tracker_calibrate_handler(line_tracker, _mode+1)
            
            # Update calibration data if returned
            if new_cal_data:
                if 'white_data' in new_cal_data:
                    calibration_data['white_data'] = new_cal_data['white_data']
                if 'black_data' in new_cal_data:
                    calibration_data['black_data'] = new_cal_data['black_data']
            
            if new_slopes is not None and new_offsets is not None:
                slopes, offsets = new_slopes, new_offsets
                _has_saved = False  
                refresh_screen()
                draw_bottom('Calibration successful! Press SPACE to save.')
                time.sleep(2)
                clear_bottom()
            else:
                refresh_screen()
                # Update grayscale_reference_obj with new calibration data
                if new_slopes is not None and new_offsets is not None:
                    grayscale_reference_obj['content'] = [
                        f"Calibration slopes: {new_slopes}",
                        f"Calibration offsets: {new_offsets}"
                    ]
                
                if calibration_data['white_data'] is not None and calibration_data['black_data'] is not None:
                    draw_bottom('Both surfaces collected! Ready for final calibration.')
                else:
                    draw_bottom('Data collection in progress. Collect both surfaces.')
                time.sleep(1)
                clear_bottom()
                
                # Force redraw of calibration data
                _draw_offset(grayscale_reference_obj)
        
        elif key.name == 'KEY_ESCAPE':
            clear_bottom()
            if not _has_saved:
                _box_width = ASK_EXIT['box_width']
                if draw_ask(ASK_EXIT['content'], location=(int((CONTENT_WIDTH-_box_width)/2), 6), align='center', box_width=_box_width):
                    # Clean up
                    grayscale_running_flag = False
                    if 'grayscale_data_thread' in locals() and grayscale_data_thread.is_alive():
                        grayscale_data_thread.join(timeout=0.5)
                    return
                else:
                    refresh_screen()
                    draw_bottom('Cancel.')
                    continue
            else:
                return
        
        elif key == ' ':  # space - save calibration
            clear_bottom()
            _box_width = ASK_SAVE['box_width']
            if draw_ask(ASK_SAVE['content'], location=(int((CONTENT_WIDTH-_box_width)/2), 6), align='center', box_width=_box_width):
                try:
                    # Save calibration data to config
                    my_car.config['line_tracker_slopes'] = slopes
                    my_car.config['line_tracker_offsets'] = offsets
  
                    _has_saved = True
                    refresh_screen()
                    draw_bottom(f'Saved! Slopes: {slopes}, Offsets: {offsets}')
                    time.sleep(0.5)
                    clear_bottom()

                except Exception as e:
                    _has_saved = False
                    refresh_screen()
                    draw_bottom(f'Save failed: {str(e)}')
            else:
                _has_saved = False
                refresh_screen()
                draw_bottom('Cancel.')
        
        # Update display with current grayscale data and calibration values
        try:
            raw_data = line_tracker.read(raw=True)
            if raw_data and len(raw_data) >= 3:
                grayscale_date = raw_data
        except Exception:
            grayscale_date = [0, 0, 0]
        
        grayscale_date_obj['content'] = [f"Raw grayscale data: {grayscale_date}"]
        
        current_slopes, current_offsets = get_line_tracker_calibration_data(line_tracker)
        if current_slopes is None:
            current_slopes = slopes
        if current_offsets is None:
            current_offsets = offsets
        
        grayscale_reference_obj['content'] = [
            f"Calibration slopes: {current_slopes}",
            f"Calibration offsets: {current_offsets}",
        ]
  
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
