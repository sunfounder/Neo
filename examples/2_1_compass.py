from zeus_pi import ZeusPi
from time import sleep

# TODO: setting the placement, range of compass 
my_car = ZeusPi()

while True:
    x, y, z, angle = my_car.read_compass()
    angle = my_car.read_compass_angle(filter=True)
    print(f"x: {x:.2f} mGuass   y: {y:.2f} mGuass   z: {z:.2f} mGuass   angle: {angle}°")
    sleep(0.01)
