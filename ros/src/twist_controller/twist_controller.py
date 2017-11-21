from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        # Implement
        self.yawC = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.steerPID = PID(.5,.01,.1,mn=-.7, mx=.7)
        self.targetAngLPF = LowPassFilter(3.,1.)
        self.throttlePID = PID(.5, .01, .1, mn=0, mx=1.)
        self.breakPID = PID(.5, .01, .1, mn=0, mx=1000.)

    def control(self, target_lin_vel, target_ang_vel, current_lin_vel):
        # Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer        
        steer = self.yawC.get_steering(target_lin_vel, self.targetAngLPF.filt(target_ang_vel), current_lin_vel)        
        steer_pid = self.steerPID.step(steer,.05)
        # throttle = 1 - (steer_pid*steer_pid)
        # TBD throttle/brake make sure that we dont go above target_lin_vel
        #throttle = 1 - abs(steer_pid)    
        #throttle = max(.5, throttle)
        # When close to target velocity, apply breaks and throttle accordingly
        if target_lin_vel == 0:
            break_pid = 1000.
            throttle_pid = 0
        elif (current_lin_vel-target_lin_vel) > -1:
            break_pid = self.breakPID.step( 75*(current_lin_vel-target_lin_vel+1), .05)
            throttle_pid = 0
            # throttle_pid = max(0.1,1 - abs(steer_pid) - self.throttlePID.step( (current_lin_vel-target_lin_vel+1), .05))
        else:
            break_pid = 0
            # When starting from rest, have a lower throttle speed
            if current_lin_vel < 2.:
                throttle_pid = .5
            else:
                throttle_pid = 1 - abs(steer_pid)
        return throttle_pid, break_pid, steer_pid #target_ang_vel
