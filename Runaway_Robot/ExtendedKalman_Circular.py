# Extended Kalman Filter for 
# a Circular motion

from matrix import *
import random

# Extended Kalman Filter with
# 5 states variables [x, y, heading, turning, distance]
# Previously without noise, heading, turning and distance was calculated on average
# With noise, this solution mostly failes
# Extended Kalman filter will infer these three values in additio to x and y with measurements

# (x		) = ( x + distance * cos(theta + phi))
# (y		)	( y + distance * sin(theta + phi))
# (theta	)	( theta + phi 					 )
# (phi    )	    ( phi 							 ) # I assume constant angular_velocity	
# (distance)	( distance						 ) # constant translational_velocity


class ExtendedKalmanFilter:
	
	def __init__(self, covariance=1000., measurementNoise=0.075):
		
		# location x, y, heading, turning, distance
		self.x = matrix([[0.0], [0.0], [0.0], [0.], [0.]]) 
		
		# Covariance
		self.P = matrix([[1000.0, 0.0, 0.0, 0.0, 0.],
						[0.0, 1000.0, 0.0, 0., 0.],
						[0., 0., 1000., 0., 0.],
						[0., 0., 0., 1000., 0.],
						[0., 0., 0., 0., 1000.]])
		# external motion 
		self.u = matrix([[0.], [0.], [0.], [0.], [0.]])
		
		# measurement matrix
		self.H = matrix([[1.0, 0., 0., 0., 0.],
						[0.0, 1.0, 0.0, 0., 0.]])
		
		self.Q = matrix([[measurementNoise, 0.],
				[0., measurementNoise]])
				
		self.I = matrix([[]])
		self.I.identity(5)
		
	def update_step(self, measurement):
		Z = matrix([[measurement[0]], [measurement[1]]])
		# error 
		y = Z - (self.H * self.x)
		S = self.H * self.P * self.H.transpose() + self.Q 
		# Kalman gain 
		K = self.P * self.H.transpose() * S.inverse()	
		self.x = self.x + K * y 
		self.P = (self.I - (K * self.H)) * self.P
	
	
	def transition_function_and_jacobian(self):
		
		x = self.x.value[0][0]
		y = self.x.value[1][0]
		heading = self.x.value[2][0]
		turning = self.x.value[3][0]
		distance = self.x.value[4][0]
		next_x = x + distance * cos(heading + turning)
		next_y = y + distance * sin(heading + turning)
		next_heading = heading + turning
		next_turning = turning
		next_distance = distance
		
		J = matrix([[1., 0., -distance*sin(heading+turning), -distance*sin(heading+turning), cos(heading+turning)],
					[0., 1., distance*cos(heading+turning), distance*cos(heading+turning), sin(heading+turning)],
					[0., 0., 1., 1., 0.],
					[0., 0., 0., 1., 0.],
					[0., 0., 0., 0., 1]])
		
		X = matrix([[next_x], [next_y], [next_heading], [next_turning], [next_distance]])
		
		return X, J
		
		
	def predict(self):
		
		self.x, J = self.transition_function_and_jacobian()
		self.P = J * self.P * J.transpose() # no motion noise
		
		estimated_position = (self.x.value[0][0], self.x.value[1][0])
		
		return estimated_position
		
	def predict_motion_ahead(self, n):
		x, _ = self.transition_function_and_jacobian()
		for i in range(n - 1):
			x, _ = self.transition_function_and_jacobian()
		
		return (x.value[0][0], x.value[1][0])
	