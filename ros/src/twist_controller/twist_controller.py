from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.yawC = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.steerPID = PID(.5, .01, .1, mn=-.7, mx=.7)
        self.lowpass = LowPassFilter(0., 1.)

    def control(self, target_lin_vel, target_ang_vel, current_lin_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        steer = self.yawC.get_steering(target_lin_vel, target_ang_vel, current_lin_vel)        
        steer_pid = self.steerPID.step(steer, .05)
        steer_pid_lp = self.lowpass.filt(steer_pid)
        #throttle = 1 - (steer_pid*steer_pid)
        throttle = 1 - abs(steer_pid)
        # throttle = 1 - abs(steer_pid_lp)
        throttle = max(.5, throttle)
        return throttle, 0., steer_pid #target_ang_vel
        # return throttle, 0., steer_pid_lp #target_ang_vel
