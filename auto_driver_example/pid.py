# python pid class

class PID():
    def __init__(self, kp, ki, kd, out_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error = 0
        self.integral = 0
        self.out_max = out_max

    def update(self, error):
        self.integral += error
        self.last_error = error
        out =  self.kp * error + self.ki * self.integral + self.kd * (error - self.last_error)
        out = max(min(out, self.out_max), -self.out_max)
        return out

    def set_pid(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def set_out_max(self, out_max):
        self.out_max = out_max

    def reset(self):
        self.last_error = 0
        self.integral = 0
