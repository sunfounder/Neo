from neo import Neo
from time import sleep
from neo.utils import debug             # 终端调试使用

my_car = Neo()
power = 50
debug("neo init ok ", end='\n', flush=True)


try:
    while True:
        try:
            my_car.move(0, power)
            sleep(1)
            my_car.move(180, power)
            sleep(1)
            my_car.move(45, power)
            sleep(1)
            my_car.move(225, power)
            sleep(1)
            my_car.move(90, power)
            sleep(1)
            my_car.move(270, power)
            sleep(1)
            my_car.move(135, power)
            sleep(1)
            my_car.move(315, power)
            sleep(1)
            debug("Round 1 ", end='\n', flush=True)
            #
            my_car.move(180, power)
            sleep(1)
            my_car.move(0, power)
            sleep(1)
            my_car.move(225, power)
            sleep(1)
            my_car.move(45, power)
            sleep(1)
            my_car.move(270, power)
            sleep(1)
            my_car.move(90, power)
            sleep(1)
            my_car.move(315, power)
            sleep(1)
            my_car.move(135, power)
            sleep(1)
            debug("Round 2 ", end='\n', flush=True)
            #
            my_car.stop()
            debug("Stop ", end='\n', flush=True)
            sleep(2)
        except Exception as e:
            print(f"Error during movement: {e}")
            debug(f"Error during movement: {e}")
            my_car.stop()
            sleep(1)  
            break  

finally:
    print("Stop")
    my_car.stop()
    sleep(0.1)
