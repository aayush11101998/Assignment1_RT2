"""
.. module:: assignment3
   :platform: Linux
   :synopsis: Python code for selecting robot behaviour   
   
.. moduleauthor:: Aayush Vats<vatsaayush11@gmail.com>

The node provides user with the possibility to select the robot's behaviour in the Gazebo simulation environment

Parameters:
  /robot_state
  used in the state() function
"""

#!/usr/bin/env python3

import rospy
from std_srvs.srv import *
import os


""" this program makes the user control robot's behaviour """

# 1- teleop keyboard
# 2- movebase client
# 3- assistance to avoid collisions

def state():

  """
  Function allows user to give a *char* input to robot so that it choses a mode to run in the simulation.  
    
  There are three behaviours from which a user can select:
    
  1 - ``Teleop keyboard``
  
  2 - ``assistance to avoid collisions``,
  advertised by :mod:`teleop_keyboard`
  
  3 - ``movebase client``,
  advertised by :mod:`move_base`
  """   
  x = input ('''choose the robot behaviour:

  w. use keyboard.
  s. autonomously reach your destination.
  d. robot using collision avoidance.
  input: ''')
   
  if x == 'w':
   
      rospy.set_param('robot_state', 'w')
      print("teleop keyboard")
       
  elif x == 's':
      
      rospy.set_param('robot_state', 's')
      print("movebase client")
   
  elif x == 'd':
      
      rospy.set_param('robot_state', 'd')
      print("assistance to avoid collisions")

def main():

   """
   The main allows to set the parameter robot_state according to the users choice.
   """

   rospy.init_node('assignment3')
   rospy.set_param('robot_state', '0')
   rate = rospy.Rate(20)
   
   while not rospy.is_shutdown():
       if (rospy.get_param('robot_state')) == '0':
           state()
       else:
           rate.sleep()
           continue 
       rate.sleep()
           
if __name__ == '__main__':
    main()
   
   
   
   
   
   
   
