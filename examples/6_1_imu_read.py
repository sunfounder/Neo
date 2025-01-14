from zeus_pi import ZeusPi
from time import sleep

my_car = ZeusPi()

try:
    while True:
        acc_data, gyro_data = my_car.read_imu()
        print(
            f"\rACC: {acc_data[0]:8.5f}g {acc_data[1]:8.5f}g {acc_data[2]:8.5f}g"
            f"  |  GYRO: {gyro_data[0]:8.5f}deg/s {gyro_data[1]:8.5f}deg/s {gyro_data[2]:8.5f}deg/s",
            end="",
            flush=True)
        sleep(0.1)
finally:
    my_car.stop()
