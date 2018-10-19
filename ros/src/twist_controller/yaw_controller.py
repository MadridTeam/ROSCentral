from math import atan

class YawController(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.min_speed = min_speed
        self.max_lat_accel = max_lat_accel

        self.min_angle = -max_steer_angle
        self.max_angle = max_steer_angle

        # This is for church lot test
        # If the velocity is smaller than this value, the rule of steering does not apply
        # self.min_steer_rule_vel = 5 
        # self.ratio_currentvel_desiredvel = 0.8

    def get_angle(self, radius):
        angle = atan(self.wheel_base / radius) * self.steer_ratio
        return max(self.min_angle, min(self.max_angle, angle))

    def get_steering(self, linear_velocity, angular_velocity, current_velocity):
        # When the real velocity is low and the desired linear velocity is large,
        # we should ensure that a large steering value can be reached.
        # For example in the church lot test
        # if current_velocity < self.min_steer_rule_vel:
        #     nominator = self.min_steer_rule_vel
        # else:
        #     nominator = current_velocity 
        nominator = current_velocity    
        angular_velocity = nominator * angular_velocity / linear_velocity if abs(linear_velocity) > 0. else 0.

        if abs(current_velocity) > 0.1:
            max_yaw_rate = abs(self.max_lat_accel / current_velocity)
            angular_velocity = max(-max_yaw_rate, min(max_yaw_rate, angular_velocity))

        return self.get_angle(max(current_velocity, self.min_speed) / angular_velocity) if abs(angular_velocity) > 0. else 0.0
