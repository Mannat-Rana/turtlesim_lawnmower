#!/usr/bin/env python
#----------------------------------------------------------------------------
# Created By  : Mannat Rana (mrana8@asu.edu)
# Created Date: 01/20/22
# version ='2.0'
# ---------------------------------------------------------------------------
""" Assignment 1 for ASU Course SES 598: Autonomous Exploration Systems
 *        Taught by Professor Jnaneshwar Das during Spring 2022"""
# ---------------------------------------------------------------------------
# Imports
# ---------------------------------------------------------------------------
import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Kill, Spawn

# Create constant default node name
node_name           = "Lawnmower_Node"

# Get global parameters from parameter server 
turtle_name         = rospy.get_param('/turtle_name')
vel_topic           = rospy.get_param('/vel_topic')
pos_topic           = rospy.get_param('/pos_topic')
linear_deadband     = rospy.get_param('/linear_deadband')
angular_deadband    = rospy.get_param('/angular_deadband')
start_point         = rospy.get_param('/start_point')
height              = rospy.get_param('/height')
width               = rospy.get_param('/width')
linear_kp           = rospy.get_param('/linear_kp')
angular_kp          = rospy.get_param('/angular_kp')
window_max          = rospy.get_param('/window_max')
window_min          = rospy.get_param('/window_min')
max_lin_vel         = rospy.get_param('/max_lin_vel')
pose_hz             = rospy.get_param('/pose_hz')

# Create a class for the turtle to contain all of its motion
class Turtle:
    
    def __init__(self):
        # Initialize node
        rospy.init_node(node_name, anonymous=True)
        
        # Kill the original spawned turtle
        kill_client = rospy.ServiceProxy('kill', Kill)
        rospy.wait_for_service('kill')
        kill_success = kill_client.call('turtle1')
        if kill_success:
            rospy.loginfo('Killed original turtle.')
        else:
            rospy.logfatal('Could not kill original turtle.')
        
        # Spawn a turtle at our desired start pose
        spawn_client = rospy.ServiceProxy('spawn', Spawn)
        rospy.wait_for_service('spawn')
        spawn_success = spawn_client.call(start_point[0],
                                          start_point[1],
                                          start_point[2],
                                          turtle_name)
        if spawn_success:
            rospy.loginfo(f'Spawned {turtle_name} at location {start_point[:2]} with angle of {start_point[2]} radians.')
        else:
            rospy.logfatal(f'Could not spawn {turtle_name}.')
            
        # Initialize the velocity publisher and pose subscriber
        self.vel_pub = rospy.Publisher(turtle_name + vel_topic, Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber(turtle_name + pos_topic, Pose, self.pose_msg_callback)
        
        # Initialize member variables to hold turtle pose
        self.m_x = 0
        self.m_y = 0
        self.m_theta = 0
        
        # Initialize empty Twist message for velocity commands
        self.vel_msg = Twist()
        
        # Initialize boolean to track if Turtle pose is known
        self.ready = False
        
        # Establish message rate
        self.msg_rate = rospy.Rate(pose_hz)
        
        # Create setpoint counter
        self.point_counter = 1
        
    # Callback to update turtle pose
    def pose_msg_callback(self, turtle_pose):
        # Update turtle pose
        self.m_x = turtle_pose.x
        self.m_y = turtle_pose.y
        self.m_theta = turtle_pose.theta
        
        # Set pose flag to true
        self.ready = True
    
    # Function to determine if Turtle pose is known
    def is_ready(self):
        return self.ready    
    
    # Function to move turtle straight
    def move_straight(self, lin_vel):
        # Cap velocity command if needed
        if lin_vel > max_lin_vel:
            lin_vel = max_lin_vel
        self.vel_msg.linear.x = lin_vel
        self.vel_msg.angular.z = 0
        
        # Publish velocity command
        self.vel_pub.publish(self.vel_msg)
    
    # Function to rotate turtle in place    
    def rotate(self, ang_vel):
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = ang_vel
        
        # Publish velocity command
        self.vel_pub.publish(self.vel_msg)
    
    # Function to get turtle to specific setpoint
    def go_to_setpoint(self, setpoint):
        # Cap setpoint to be contained within window
        if setpoint[0] > window_max:
            rospy.logerr(f'X Coordinate for point {self.point_counter} is too high, reducing to {window_max}.')
            setpoint[0] = window_max
        elif setpoint[0] < window_min:
            rospy.logerr(f'X Coordinate for point {self.point_counter} is too low, raising to {window_min}.')
            setpoint[0] = window_min
        if setpoint[1] > window_max:
            rospy.logerr(f'Y Coordinate for point {self.point_counter} is too high, reducing to {window_max}.')
            setpoint[1] = window_max
        elif setpoint[1] < window_min:
            rospy.logerr(f'Y Coordinate for point {self.point_counter} is too low, raising to {window_min}.')
            setpoint[1] = window_min
        rospy.loginfo(f'Going to point {self.point_counter}: [{setpoint[0]:.1f}, {setpoint[1]:.1f}]...')

        # Calculate x, y, and theta error
        d_x = setpoint[0] - self.m_x
        d_y = setpoint[1] - self.m_y
        theta_error = math.atan2(d_y, d_x) - self.m_theta

        # Rotate turtle until angular error is within deadband
        while (abs(theta_error) > angular_deadband):
            # Simple P Controller for Angular Velocity
            self.rotate(angular_kp * theta_error)
            theta_error = math.atan2(d_y, d_x) - self.m_theta
            self.msg_rate.sleep()
        self.rotate(0)
        
        # Calculate raw distance to setpoint via Pythagorean Theorem
        distance_to_target = math.sqrt((d_x ** 2) + (d_y ** 2))
        
        # Drive turtle forward until raw distance is within deadband
        while(abs(distance_to_target) > linear_deadband):
            # Simple P Controller for Linear Velocity
            self.move_straight(linear_kp * distance_to_target)
            d_x = setpoint[0] - self.m_x
            d_y = setpoint[1] - self.m_y
            distance_to_target = math.sqrt((d_x ** 2) + (d_y ** 2))
            self.msg_rate.sleep()
        self.move_straight(0)
        rospy.loginfo(f'Reached point {self.point_counter}: [{setpoint[0]:.1f}, {setpoint[1]:.1f}].')
        self.point_counter += 1
        
    def mow_lawn(self):
        # Check to make sure turtle pose is known
        rospy.loginfo('Checking for position information.')
        while not self.is_ready():
            rospy.logwarn_once('Waiting for position information...')
        rospy.loginfo("Recieved position information, starting lawn mowing.")    

        # Create setpoints for the zigzag lawn mowing motion
        self.go_to_setpoint([self.m_x + width, self.m_y]);
        self.go_to_setpoint([self.m_x, self.m_y + height]);
        self.go_to_setpoint([self.m_x - width, self.m_y]);
        self.go_to_setpoint([self.m_x, self.m_y + height]);
        self.go_to_setpoint([self.m_x + width, self.m_y]);
        self.go_to_setpoint(start_point[:2])
        rospy.loginfo("Finished Lawn Mowing Path.")      
        
def main():
    while not rospy.is_shutdown():
        # Initialize turtle and begin lawn mowing    
        turtle = Turtle()
        turtle.mow_lawn()
        rospy.loginfo('Terminating Node')
        break
    
if __name__ == '__main__':
    main()