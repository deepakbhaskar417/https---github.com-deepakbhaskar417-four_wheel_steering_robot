#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
import math
from math import sin, cos, pi
from time import sleep
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf

class FourWheelSteeringController:
    def __init__(self):
        self.wheel_separation = 1.410
        self.track = 1.20
        self.wheel_base = 1.59
        self.wheel_radius = 0.330
        self.wheel_steering_y_offset = 0.103
        self.lf_steering_angle = rospy.Publisher('/front_left_steering_joint_position_controller/command', Float64, queue_size=1)
        self.rf_steering_angle = rospy.Publisher('/front_right_steering_joint_position_controller/command', Float64, queue_size=1)
        self.lb_steering_angle = rospy.Publisher('/rear_left_steering_joint_position_controller/command', Float64, queue_size=1)
        self.rb_steering_angle = rospy.Publisher('/rear_right_steering_joint_position_controller/command', Float64, queue_size=1)
        self.lf_wheel_speed = rospy.Publisher('/front_left_wheel_joint_velocity_controller/command', Float64, queue_size=1)
        self.rf_wheel_speed = rospy.Publisher('/front_right_wheel_joint_velocity_controller/command', Float64, queue_size=1)
        self.lb_wheel_speed = rospy.Publisher('/rear_left_wheel_joint_velocity_controller/command', Float64, queue_size=1)
        self.rb_wheel_speed = rospy.Publisher('/rear_right_wheel_joint_velocity_controller/command', Float64, queue_size=1)
        self.vel_subscriber = rospy.Subscriber('/cmd_vel',Twist,self.find_steering_angle)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.cmd_vel_timeout = 1
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        

    def find_steering_angle(self,velocity):
        self.start_time = rospy.get_time()
        self.vel_x= velocity.linear.x
        self.vel_y= velocity.linear.y
        self.omega = velocity.angular.z
       
        print(rospy.get_time(),self.start_time,self.cmd_vel_timeout,rospy.get_time() - self.start_time)
        if rospy.get_time() - self.start_time > self.cmd_vel_timeout:
            #publish wheel velocities
            self.lf_wheel_speed.publish(0.0)
            self.rf_wheel_speed.publish(0.0)
            self.lb_wheel_speed.publish(0.0)
            self.rb_wheel_speed.publish(0.0)


        print(self.vel_x)
        if  self.vel_x !=0 and self.vel_y ==0 and self.omega != 0:
            self.opp_phase_steering()
        if  self.vel_x ==0 and self.vel_y ==0 and self.omega != 0:
            self.pivot_mode()
        if  self.vel_x !=0 and self.vel_y !=0 and self.omega == 0:
            self.inphase_mode()
        if  self.vel_x !=0 and self.vel_y ==0 and self.omega == 0:
            self.straight_mode()
           
    def opp_phase_steering(self):
        self.current_time = rospy.Time.now()
        
        self.left_wheel_speed = math.copysign(1,self.vel_x)*(1/(self.wheel_radius))*(math.sqrt((self.vel_x-0.5*self.omega*self.track)**2+(0.5*self.omega*self.wheel_base)**2)-(self.omega*self.wheel_steering_y_offset))
        self.right_wheel_speed = math.copysign(1,self.vel_x)*(1/(self.wheel_radius))*(math.sqrt((self.vel_x+0.5*self.omega*self.track)**2+(0.5*self.omega*self.wheel_base)**2)+(self.omega*self.wheel_steering_y_offset))


        
        print('omega : %f vel:%f wb:%f track:%f',self.omega,self.vel_x,self.wheel_base,self.track) # for debugging
        # self.steering_angle = math.atan(((self.omega)*self.wheel_base)/((2*abs(self.vel_x))+(abs(self.omega)*self.track)))
        self.fl_steering_angle = math.atan((self.omega*self.wheel_base)/((2*self.vel_x)-(self.omega*self.track)))
        self.fr_steering_angle = math.atan((self.omega*self.wheel_base)/((2*self.vel_x)+(self.omega*self.track)))
        #publish steering angle
        self.rf_steering_angle.publish(1*self.fr_steering_angle)
        self.lf_steering_angle.publish(1*self.fl_steering_angle)
        self.rb_steering_angle.publish(-1*self.fr_steering_angle)
        self.lb_steering_angle.publish(-1*self.fl_steering_angle)

        #publish wheel velocities
        self.lf_wheel_speed.publish(self.left_wheel_speed)
        self.rf_wheel_speed.publish(self.right_wheel_speed)
        self.lb_wheel_speed.publish(self.left_wheel_speed)
        self.rb_wheel_speed.publish(self.right_wheel_speed)

        #compute wheel velocity from wheel speed
        self.left_turn_radius = (0.5*self.wheel_base-self.wheel_steering_y_offset*sin(self.fl_steering_angle))*math.tan(self.fl_steering_angle)
        self.tur_radius = cos(self.fl_steering_angle)*(self.left_turn_radius+self.wheel_steering_y_offset)+0.5*self.track
        self.vel_lf_wheel = self.wheel_radius*self.left_wheel_speed
        self.vel_lb_wheel = self.wheel_radius*self.left_wheel_speed # when using encoders it may change
        self.vx = self.tur_radius*(self.vel_lf_wheel/self.left_turn_radius)
        self.vy = 0
        self.vth = self.vx/self.tur_radius
        # compute odometry in a typical way given the velocities of the robot
        self.dt = (self.current_time - self.last_time).to_sec()
        self.delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * self.dt
        self.delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * self.dt
        self.delta_th = self.vth * self.dt

        self.x += self.delta_x
        self.y += self.delta_y
        self.th += self.delta_th
        # since all odometry is 6DOF we'll need a quaternion created from yaw
        self.odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

        # first, we'll publish the transform over tf
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.),
            self.odom_quat,
            self.current_time,
            "base_link",
            "odom"
        )
        # next, we'll publish the odometry message over ROS
        self.odom = Odometry()
        self.odom.header.stamp = self.current_time
        self.odom.header.frame_id = "odom"

        # set the position
        self.odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*self.odom_quat))

        # set the velocity
        self.odom.child_frame_id = "base_link"
        self.odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))

        # publish the message
        self.odom_pub.publish(self.odom)

        self.last_time = self.current_time

       
        #print(self.left_wheel_speed,self.right_wheel_speed,self.fl_steering_angle)

    def pivot_mode(self):
        self.wheel_speed = 1*pi*((0.5*math.sqrt(self.wheel_base**2+self.track**2)+self.wheel_steering_y_offset))*self.omega*2*pi*self.wheel_radius
        self.steering_angle = math.atan(self.wheel_base/self.track)
        #publish steering angle
        self.rf_steering_angle.publish(1*self.steering_angle)
        self.lf_steering_angle.publish(-1*self.steering_angle)
        self.rb_steering_angle.publish(-1*self.steering_angle)
        self.lb_steering_angle.publish(1*self.steering_angle)

         #publish wheel velocities
        self.lf_wheel_speed.publish(-self.wheel_speed)
        self.rf_wheel_speed.publish(self.wheel_speed)
        self.lb_wheel_speed.publish(-self.wheel_speed)
        self.rb_wheel_speed.publish(self.wheel_speed)
        print(self.wheel_speed,self.steering_angle)
       
    def inphase_mode(self):
        self.wheel_speed =  2*pi*math.copysign(1,self.omega)*2*pi*self.wheel_radius*math.sqrt(self.vel_x**2+self.vel_y**2)
        self.steering_angle = math.atan(self.vel_y/self.vel_x)
        print(self.wheel_speed,self.steering_angle)

        #publish steering angle
        self.rf_steering_angle.publish(1*self.steering_angle)
        self.lf_steering_angle.publish(1*self.steering_angle)
        self.rb_steering_angle.publish(1*self.steering_angle)
        self.lb_steering_angle.publish(1*self.steering_angle)

         #publish wheel velocities
        self.lf_wheel_speed.publish(self.wheel_speed)
        self.rf_wheel_speed.publish(self.wheel_speed)
        self.lb_wheel_speed.publish(self.wheel_speed)
        self.rb_wheel_speed.publish(self.wheel_speed)
        print(self.wheel_speed,self.steering_angle)
    
    def straight_mode(self):
        self.wheel_speed =  2*pi*math.copysign(1,self.vel_x)*2*pi*self.wheel_radius*math.sqrt(self.vel_x**2)
        self.steering_angle = 0
         #publish steering angle
        self.rf_steering_angle.publish(1*self.steering_angle)
        self.lf_steering_angle.publish(1*self.steering_angle)
        self.rb_steering_angle.publish(1*self.steering_angle)
        self.lb_steering_angle.publish(1*self.steering_angle)

        #publish wheel velocities
        self.lf_wheel_speed.publish(self.wheel_speed)
        self.rf_wheel_speed.publish(self.wheel_speed)
        self.lb_wheel_speed.publish(self.wheel_speed)
        self.rb_wheel_speed.publish(self.wheel_speed)


if __name__ == '__main__':
    rospy.init_node('fwd_controller',anonymous=True)
    FourWheelSteeringController()
    rospy.spin()
