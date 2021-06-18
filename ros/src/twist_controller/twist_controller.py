from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass,fuel_capacity,brake_deadband,decel_limit,
		accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
		
		# Copy the values to class variables
		self.vehicle_mass = vehicle_mass
		self.fuel_capacity = fuel_capacity
		self.brake_deadband = brake_deadband
		self.decel_limit = decel_limit
		self.accel_limit = accel_limit
		self.wheel_radius = wheel_radius
		self.wheel_base = wheel_base
		self.steer_ratio = steer_ratio
		self.max_lat_accel = max_lat_accel
		self.max_steer_angle = max_steer_angle
		
		# initialise Yaw and throttle controller
		self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
		
		kp = 0.3
		ki = 0.1
		kd = 0.
		mn_throttle = 0. # Minimum Throttle value
		mx_throttle = 0.2 # Maximum Throttle value
		
		self.throttle_controller = PID(kp, ki, kd, mn_throttle, max_throttle)
		
		# set low pass filter for velocity signal
		tau = 0.5 # 1/(2 * pi * tau) : Cut off Frequency
		ts = 0.02 # sample time
		self.vel_lpf = LowPassFilter()
		
		self.last_time = rospy.get_time()
		

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
		if not dbw_enabled:
			self.throttle_controller.reset()
			return 0., 0., 0.
		
		# get Steering value using yaw controller
		vel_curr = self.vel_lpf.filt(current_vel)
		
		steering = self.yaw_controller.get_steering(linear_vel, angular_vel, vel_curr)
		
		# get throttle using PID controller
		vel_error = linear_vel - vel_curr # Velocity error is difference between actual velocity and required velocity
		self.last_vel = vel_curr
		
		time_current = rospy.get_time()
		time_sample = time_current - self.last_time
		self.last_time = time_current
		
		throttle = self.throttle_controller.step(vel_error, time_sample)
		
		# Setting Torque for brakes
		brake = 0
		
		if linear_vel == 0. and vel_curr < 0.1 : # Brining car to a stop , possibly due a traffic signal
			throttle = 0
			brake = 400 # Nm , to hold the car in place
		elif throttle < 0.1 and vel_error < 0 : # Car is going faster than required
			throttle = 0
			decel = max(vel_error, self.decel_limit) # dont decelerate more than minimum limit
			brake = abs(decel) * ( self.vehicle_mass * wheel_radius ) # Braking Torque = Mass * radius * accl 
			
		return throttle, brake, steering
