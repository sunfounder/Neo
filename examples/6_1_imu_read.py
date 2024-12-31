from zeus_pi import ZeusPi
from time import sleep

my_car = ZeusPi()

try:
    while True:
        acc_data, gyro_data = my_car.imu.read_all()
        print(
            f"\rACC: {acc_data[0]:^8} {acc_data[1]:^8} {acc_data[2]:^8}"
            f"  |  GYRO: {gyro_data[0]:^8} {gyro_data[1]:^8} {gyro_data[2]:^8}",
            end="",
            flush=True)
        sleep(0.5)
finally:
    my_car.stop()
