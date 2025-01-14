from zeus_pi import ZeusPi
from blessed import Terminal
import time


my_car = ZeusPi()
rotate_speed = 80

magnetic_declination = "3°18'W" # (-3.3°) shenzhen 2024 magnetic_declination (https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml)
my_car.set_compass_magnetic_declination(magnetic_declination)
print(f"magnetic_declination: {my_car.compass.magnetic_declination_angle}")

_on_calibration = False
st = time.time()
print_st = time.time()
x_min = 0
x_max = 0
y_min = 0
y_max = 0
z_min = 0
z_max = 0

term = Terminal()

def main():
    with term.fullscreen(), term.cbreak():
        while True:
            loop()

def loop():
    global _on_calibration, x_min, x_max, y_min, y_max, z_min, z_max
    global print_st, st

    key = term.inkey(timeout=0.01)
    if key == 'q':
        if not _on_calibration:
            st = time.time()
            _changed = False
            my_car.move(0, 0, rotate_speed)
            _on_calibration = True
        else:
            _on_calibration = False
            my_car.stop()

    if _on_calibration:
        x_raw, y_raw, z_raw = my_car.read_compass_raw()
        if time.time() - print_st > 0.5:
            print(f"x_raw: {x_raw}   y_raw: {y_raw}   z_raw: {z_raw}               ")
            # draw tips on bottom
            with term.location():
                print(term.move_xy(0, term.height - 1), end='')
                print(term.black_on_skyblue(f'  Press "q" to start / stop calibration  '), end='', flush=True)
            print_st = time.time()

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
            my_car.config.write()
            _on_calibration = False
            my_car.stop()
    else:
        x, y, z, angle = my_car.read_compass()
        if time.time() - print_st > 0.5:
            print(f"x: {x:.2f} mGuass   y: {y:.2f} mGuass   z: {z:.2f} mGuass   angle: {angle:.2f}°")
            # draw tips on bottom
            with term.location():
                print(term.move_xy(0, term.height - 1), end='')
                print(term.black_on_skyblue(f'  Press "q" to start / stop calibration  '), end='', flush=True)
            print_st = time.time()



if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        my_car.stop()




