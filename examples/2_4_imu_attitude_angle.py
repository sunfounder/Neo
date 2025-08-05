from zeus_pi import Neo
from time import sleep

'''
# https://github.com/xioTechnologies/Fusion

Use the fusion algorithm to convert the imu data into attitude angle.
Without magnetometer fusion, the yaw angle will accumulate errors, about 0.01 deg/s,
Using magnetometer fusion will be disturbed by the magnetic field environment
'''

my_car = Neo()
my_car.start_imu_fusion(with_mag=True)

def main():
    while True:
        roll, pitch, yaw = my_car.read_euler()
        print(f'Roll: {roll:.3f}°, Pitch: {pitch:.3f}°, Yaw: {yaw:.3f}°')
        sleep(0.1)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        my_car.stop()
        sleep(0.1)
