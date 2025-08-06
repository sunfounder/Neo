from neo import Neo
from time import sleep
from vilib import Vilib

my_car = Neo()

my_car.set_cam_pan(0)
my_car.set_cam_tilt(0)

def clamp_number(num, a, b):
  return max(min(num, max(a, b)), min(a, b))

def main():
    camera_width = 800
    camera_height = 600

    Vilib.camera_start(vflip=True, hflip=True, size=(camera_width, camera_height))
    Vilib.show_fps()
    Vilib.display(local=True, web=True) 
    Vilib.color_detect(color="red")  # red, green, blue, yellow , orange, purple
    sleep(1)

    x_angle =0
    y_angle =0
    while True:
        n = Vilib.color_obj_parameter['n']
        color = Vilib.color_obj_parameter['color']
        
        if n != 0:
            x = Vilib.color_obj_parameter['x']
            y = Vilib.color_obj_parameter['y']
            w = Vilib.color_obj_parameter['w']
            h = Vilib.color_obj_parameter['h']

            if w > 150 and h > 150:
                # change the pan-tilt angle for track the object
                x_angle +=(x*10/camera_width)-5
                x_angle = clamp_number(x_angle, -35, 35)
                my_car.set_cam_pan(x_angle)

                y_angle -=(y*10/camera_height)-5
                y_angle = clamp_number(y_angle,-35,35)
                my_car.set_cam_tilt(y_angle)

        sleep(0.01)



if __name__ == "__main__":
    try:
       main()
    except KeyboardInterrupt:
        pass  
    except Exception as e:
        print(f"\033[31mERROR: {e}\033[m")
    finally:
        print("Stop")
        my_car.stop()
        Vilib.camera_close()
        sleep(0.1)
