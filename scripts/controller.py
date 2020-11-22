#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
class sahayak:
	def __init__(self):
		rospy.init_node('ebot_controller', anonymous = True)
    		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    		rospy.Subscriber('/ebot/laser/scan', LaserScan, self.laser_callback)
    		rospy.Subscriber('/odom', Odometry, self.odom_callback)
		self.rate = rospy.Rate(70)
		self.pose = [0.0, 0.0, 0.0]
		self.regions = {'bright': 0.0, 'fright': 0.0, 'front': 0.0, 'fleft': 0.0, 'bleft': 0.0}
		self.ebot_theta = 0.0
		self.euc_d = 0.0
		self.ang_v = 0.0
		self.i = 0
		self.range_max = 10.0

	def odom_callback(self, data):
    		x = data.pose.pose.orientation.x
    		y = data.pose.pose.orientation.y
    		z = data.pose.pose.orientation.z
    		w = data.pose.pose.orientation.w
    		self.pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]
    		self.ebot_theta = euler_from_quaternion([x,y,z,w])[2]

	def Waypoints(self,t):
    		if t == 0:
        		h = 1.7
        		k = 1.45
    		elif t == 1:
        		h = 4.0
        		k = -1.45   
    		elif t == 2:
        		h = 6.18
        		k = 0.0
    		return [h,k]  

	def laser_callback(self, msg):	
		self.regions = {'bright': min(min(msg.ranges[0:143]), self.range_max), 'fright': min(min(msg.ranges[144:297]), self.range_max), 'front': min(min(msg.ranges[298:431]), self.range_max), 'fleft': min(min(msg.ranges[432:575]), self.range_max), 'bleft': min(min(msg.ranges[576:713]), self.range_max)}

	def euc_dis(self, u, v):
		self.euc_d = math.sqrt(math.pow((u - self.pose[0]), 2) + math.pow((v - self.pose[1]), 2))
		return self.euc_d

	def ang_vel(self, w, z):
		self.ang_v = ((math.atan2((z - self.pose[1]), (w - self.pose[0]))) - (self.pose[2]))
		return self.ang_v	

	def control_loop(self):
        	velocity_msg = Twist()
		while not rospy.is_shutdown() and self.i<3:
			[x2,y2] = self.Waypoints(self.i)
			self.euc_dis(x2,y2)
			rospy.loginfo("x and y = %f     %f      %f", self.pose[2], self.euc_d, self.ang_v)
			while self.euc_dis(x2,y2) > 0.50:

				rospy.loginfo("x and y = %f     %f      %f", self.pose[2], self.euc_d, self.ang_v)
    				velocity_msg.linear.x = 0.2*self.euc_dis(x2,y2)
    				velocity_msg.angular.z = 2.3*self.ang_vel(x2,y2)
    				self.pub.publish(velocity_msg)
				self.rate.sleep()

			velocity_msg.linear.x = 0.0
    			velocity_msg.angular.z = 0.0
    			self.pub.publish(velocity_msg) 
			self.rate.sleep()
			self.i = self.i + 1
			m = self.pose[0]
    		if self.i > 2:
			d = 1.5
			x3 = 12.5
			y3 = 0.0

			while self.euc_dis(x3,y3) > 0.50 and self.pose[2] > 0.15:
				rospy.loginfo("%f  %f  %f  %f", self.regions['bright'], self.regions['fright'], self.regions['front'], self.pose[2])
				if self.regions['front'] > d and self.regions['fright'] > d and self.regions['bright'] > d:
					velocity_msg.linear.x = 0.1
					velocity_msg.angular.z = 0.0				
				elif self.regions['front'] < d and self.regions['fright'] > d and self.regions['bright'] > d:
					velocity_msg.angular.z = 0.3
				elif self.regions['front'] > d and self.regions['fright'] < d and self.regions['bright'] > d:
					velocity_msg.linear.x = 0.1
					velocity_msg.angular.z = 0.0
				elif self.regions['front'] < d and self.regions['fright'] < d and self.regions['bright'] > d:	
					velocity_msg.linear.x = 0.0
					velocity_msg.angular.z = 0.2
				elif self.regions['front'] > d and self.regions['fright'] < d and self.regions['bright'] < d:	
					velocity_msg.linear.x = 0.1
					velocity_msg.angular.z = 0.0
				elif self.regions['front'] > d and self.regions['fright'] > d and self.regions['bright'] < d:	
					velocity_msg.linear.x = 0.1
					velocity_msg.angular.z = -0.2
    				self.pub.publish(velocity_msg)
				self.rate.sleep()
				rospy.loginfo("euc dist %f   %f ", self.euc_dis(x3,y3), self.ang_vel(x3,y3))
			while self.euc_dis(x3,y3) > 0.50:
				rospy.loginfo("euc dist %f   %f ", self.euc_dis(x3,y3), self.ang_vel(x3,y3))
				velocity_msg.linear.x = 0.2*self.euc_dis(x3,y3)
    				velocity_msg.angular.z = 1.5*self.ang_vel(x3,y3)
    				self.pub.publish(velocity_msg)
				self.rate.sleep()						
			velocity_msg.linear.x = 0.0
    			velocity_msg.angular.z = 0.0
    			self.pub.publish(velocity_msg) 
			self.rate.sleep()

if __name__ == '__main__':
    	try:
		sh = sahayak()        		
		sh.control_loop()
    	except rospy.ROSInterruptException:
        	pass

