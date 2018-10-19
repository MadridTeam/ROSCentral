import rospy

from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

# General ideals for improvement:
# 1. Make full use of the variables, e.g. use mass ,accel_limit, decel_limit etc.
# 2. Use PID at the very beginning. But can we also use MPC? Are we allowed to subscribe /final_waypoints? Can try it if have time.
# 3. Gain scheduling.

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
                 accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle, update_rate):
        min_speed = 0.1
        self.yaw_controller = YawController(
            wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        self.THROTTLE_DECEL = 0.5/update_rate

        kp = 2 * 1./update_rate # Should be very small, otherwise the accel limit will be exceeded suddenly
        ki = 0.2 * 1./update_rate
        kd = 0.0 * 1./update_rate
        mn = 0.
        mx = 1.
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        tau1 = 0.5
        tau2 = 0.5
        ts = 1./update_rate
        self.vel_lpf = LowPassFilter(tau1, ts)
        self.steer_lpf = LowPassFilter(tau2, ts)

        self.vehicle_mass = vehicle_mass+fuel_capacity*GAS_DENSITY
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.update_rate = update_rate

        self.last_time = rospy.get_time()
        self.last_vel = 0.
        self.last_throttle = 0.

    def control(self, current_vel, linear_vel, angular_vel, dbw_enabled):
        # Return throttle, brake, steer
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.

        # Time
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        # Velocity
        current_vel = self.vel_lpf.filt(current_vel)
        vel_error = linear_vel - current_vel

        # throttle and brake should not be positive at the same time
        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0.
        current_accel = self.update_rate*(current_vel -  self.last_vel)

        if linear_vel == 0 and vel_error < 0.1:  # Stop
            throttle = 0.
            brake = 700.  # 700 on Carla  # The torque to chill Carla
        elif throttle < 0.1 and vel_error < 0:  # Brake
            throttle = 0.
            decel = max(vel_error, self.decel_limit) # Use the minimal possible deceleration
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius
            brake = max(brake, self.brake_deadband)
        elif current_accel>self.accel_limit: # Accel exceeds the maximum accel
            throttle = self.last_throttle - self.THROTTLE_DECEL
            throttle = max(throttle, 0) # Prevent the throttle from being negative

        self.last_vel = current_vel
        self.last_throttle = throttle

        # Steer
        steering = self.yaw_controller.get_steering(
            linear_vel, angular_vel, current_vel)
        steering = self.steer_lpf.filt(steering)

        #rospy.logwarn('linear: %s, current: %s, error: %s',
        #              linear_vel, current_vel, vel_error)
        #rospy.logwarn('angular: %s', angular_vel)
        #rospy.logwarn('throttle: %s, brake: %s, steer: %s',
        #              throttle, brake, steering)
                      
        return throttle, brake, steering
