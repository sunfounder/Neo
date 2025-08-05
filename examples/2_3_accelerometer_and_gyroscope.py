from zeus_pi import Neo
from time import sleep

# TODO: setting the placement, range of imu 
my_car = Neo()

def main():
    while True:
        acc, gyro = my_car.read_imu()
        acc_x, acc_y, acc_z = acc
        gyro_x, gyro_y, gyro_z = gyro
        print(
            f"\rACC: {acc_x:8.5f}g {acc_y:8.5f}g {acc_z:8.5f}g"
            f"  |  GYRO: {gyro_x:8.5f}deg/s {gyro_y:8.5f}deg/s {gyro_z:8.5f}deg/s"
            f"{' '*8}", # Avoid display errors when length changes
            end="",
            flush=True)
        sleep(0.1)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        my_car.stop()
        sleep(0.1)
