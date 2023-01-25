"""
.. module:: teleop_keyboard
   :platform: Linux
   :synopsis: Python code for teleop keyboard & assistance to avoid collisions
   
.. moduleauthor:: Aayush Vats<vatsaayush11@gmail.com>

Subscribes to: 
  /scan
  /odom
  
The node provides user with the options to control the robot using teleop keyboard as well as assist user of a collision free teleop keyboard control in the Gazebo simulation environment.

"""

#!/usr/bin/env python3

from __future__ import print_function
import threading
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
from sensor_msgs.msg import LaserScan
import os
import time

# it implements two behaviors:
# 1. use keyboard to navigate.
# 2. use keyboard with collision free assistance. 


front_obs = False
right_obs = False
left_obs = False
mesg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >
t : up (+z)
b : down (-z)
anything else : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to change robot behaviour
"""

# user will be able to use the keyboard to make the robot move
moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,1),
        'j':(0,0,0,-1),
        'l':(0,0,0,1),
        'u':(1,0,0,-1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,-1),
        'm':(-1,0,0,1),
       
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

def clbk_laser(msg):
    """
    
    Callback function that provides the laser readings of the robot which are helpful in deciding regions which further assists robot with collision avoidance assistance.
    Args: 
        msg(LaserScan): provides the laserscan values
        
    """
    global front_obs 
    global right_obs 
    global left_obs
    regions = {
        'right':  min(min(msg.ranges[0:240]), 10),
        'front':  min(min(msg.ranges[241:480]), 10),
        'left':  min(min(msg.ranges[481:720]), 10),
    }
    if regions['front'] < 1:
        front_obs = True
    else:
        front_obs = False
    if regions['right'] < 1:
        right_obs = True
        # print("right obs detected")
    else:
        right_obs = False
    if regions['left'] < 1:
        left_obs = True
        # print("left obs detected")
    else:
        left_obs = False

class PublishThread(threading.Thread):

    """
    Publishes thread to update the pose of robot and allows user to have possibility to control the robot by keyboard keys and also can assist them to avoid obstacles.
    Once the task finishes the thread resets everything and stops the twist messages. 
    """
    def __init__(self, rate):
    
        """
        constructor method
        """
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):

        global front_obs
        global right_obs
        global left_obs 
        twist = Twist()
        while not self.done:
            self.condition.acquire()

            if rospy.get_param('robot_state')=='w':
            
                """ 
                Copy state into twist message.
                """
                twist.linear.x = self.x * self.speed
                twist.linear.y = self.y * self.speed
                twist.linear.z = self.z * self.speed
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = self.th * self.turn

            elif rospy.get_param('robot_state')=='d':

                """ 
                avoids the robot from moving forward if there is an obstacle
                """
                if front_obs == True and self.x == 1:
                    twist.linear.x = 0
                """
                otherwise it can move forward.
                """
            else:

                twist.linear.x = self.x * self.speed 
                twist.linear.y = self.y * self.speed
                twist.linear.z = self.z * self.speed
                twist.angular.x = 0
                twist.angular.y = 0
                """
                avoids the robot from turning to right if there is an obstacle
                """
                if right_obs == True and self.th == -1:
                    twist.angular.z = 0
                    """
                    avoids the robot from turning to left if there is an obstacle
                    """
                elif left_obs == True and self.th == 1:
                    twist.angular.z = 0
                    """
                    otherwise robot can turn in any direction
                    """

                else:
                    twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist)


def getKey(key_timeout):

    """
    returns:
        key
    """
    settings = termios.tcgetattr(sys.stdin) 

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):

    """
    args: 
        speed: robot speed
        turn: angular speed 
    returns:
        speed(string): the speed of robot
        turn(string): the turning speed of robot
    """
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

def teleop():
    """
    teleop runs the thread messages it gets the parameters from the user and accordingly executes the messages sent to it.
    """
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

def teleop():

    settings = termios.tcgetattr(sys.stdin)
    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn)

        os.system('cls||clear')
        print("** TELEOP TWIST KEYBOARD NODE **\n")
        print(mesg)
        print(vels(speed,turn))
        while(1):
            key = getKey(key_timeout)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed,turn))
                if (status == 14):
                    print(mesg)
                status = (status + 1) % 15
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    rospy.set_param('robot_state', '0')
                    os.system('cls||clear')
                    print("** TELEOP TWIST KEYBOARD NODE **\n")
                    print("choose robot behaviour in master node")
                    time.sleep(5)
                    os.system('cls||clear')
                    print("** TELEOP TWIST KEYBOARD NODE **\n")
                    print("waiting for master node response...\n")
                    break
 
            pub_thread.update(x, y, z, th, speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        
def main():

    """
    The state **teleop_keyboard** or **teleop_keyboard with obstacle avoidance assistance** is initialized using the parameter robot_state.
    """

    rospy.init_node('teleop_keyboard')
    rospy.set_param('robot_state', '0')
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)      
    rate = rospy.Rate(20)
   
    while not rospy.is_shutdown():   
          if rospy.get_param('robot_state')=='w' or rospy.get_param('robot_state')=='d':
              teleop()    
          else:
              rate.sleep()
              continue
          rate.sleep()
          
if __name__ == '__main__':
    main()       
        
        
        
        
        
        
        
        
        
