from zeus_pi import Neo
from time import sleep

'''
Obtaining Attitude Data from a 9D IMU and Using PID Control Algorithm to 
keep the car heading
'''

my_car = Neo()
my_car.start_imu_fusion(with_mag=True)

def main():
    while True:
        roll, pitch, yaw = my_car.read_euler()

        # my_car.move_with_pid(angle=0, move_power=0, heading=0) # stop and keep heading

        my_car.move_with_pid(angle=0, move_power=50, heading=0) # go straight and keep heading

        # my_car.move_with_pid(angle=-45, move_power=50, heading=45) # move to angle -45 and keep heading facing 45

        print(f'Roll: {roll:.3f}°, Pitch: {pitch:.3f}°, Yaw: {yaw:.3f}° {my_car.origin_heading:.3f}')
        sleep(0.01)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        my_car.stop()
        sleep(0.1)
