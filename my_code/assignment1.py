from __future__ import print_function
import time
from sr.robot import *
"""assignment 1"""
"""run using: python run.py Research_Track_Assignment_1.py"""
"""a_th is threshold for orientation checking variable"""
a_th = 2.0 
"""d_th is threshold for linear distance variable"""
d_th = 0.4
"""d_thg is threshold linear distance value to avoid collision with gold"""
d_thg = 0.8

R = Robot()

"""making class for robot to drive forward and backward"""

def drive(speed, seconds):
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0

"""making class for robot to turn"""

def turn(speed, seconds):
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = -speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0 
    R.motors[0].m1.power = 0
    
"""making classes for silver token and gold token detection"""

def find_silver_token(direction, width):
#direction is basically at what anlge with respect to the token from robot face direction token can only be specified as (0, 90, -90) 
#width is the width of the field.   
    dist = 300
    for token in R.see():
        if token.dist < dist and token.info.marker_type == MARKER_TOKEN_SILVER and (direction-width) <= token.rot_y <= (direction+width):
           dist = token.dist
           rot_y = token.rot_y
# -1 is returned when nothing is closer to robot otherwise distance from token and the angle made by token with respect to the token is returned.   
    if dist == 300:
        return -1, -1   
    else: 
        return dist, rot_y
     
def find_gold_token(direction, width):
     
    dist = 300
    for token in R.see():
        if token.dist < dist and token.info.marker_type == MARKER_TOKEN_GOLD and (direction-width) <= token.rot_y <= (direction+width):
#direction is basically at what anlge with respect to the token from robot face direction token can only be specified as (0, 90, -90) 
#width is the width of the field.
           dist = token.dist
           rot_y = token.rot_y  
# -1 is returned when nothing is closer to robot otherwise distance from token and the angle made by token with respect to the token is returned.    
    if dist == 300:
    	return -1, -1       
    else:
    	return dist, rot_y
           
def go_away_from_gold():
# this function search a gold wall that is in front of robot with a total angle made of (-60 to 60 degrees).
# this function helps the robot in decision making with which side it will be best to turn in order to avoid collision.   
    dist, rot_y = find_gold_token(0, 60)
    
    if dist == -1:
           return
    
    if dist < d_thg:
        print("gold tokens!!")
        left = left_priority()
# when dist_l is more than dist_r robot should turn left in order to avoid collision and viceversa.
        if left:
           
              turn(-20,0.5)
              print("turning left because left is better")
              dist, rot_y = find_gold_token(0,10)
        else: 
           
              turn(20,0.5)
              print("turning right because right is better")
              dist, rot_y = find_gold_token(0,10)
          
    
              
def left_priority():    
# this function makes a robot check distance from walls on right and left and compare them.      
    dist_l,rot_yl = find_gold_token(-90,10)
    dist_r, rot_yr= find_gold_token(90,10)
    
    print ("distance left =", str(dist_l))
    print ("distance right =", str(dist_r))
    if dist_r < dist_l:
       return True
    
    else:
       return False  
       
def lift_silver_token():
# function helps robot detect silver token and then makes the robot to grab the token and release it such that silver token does'nt come in it's way.
    dist, rot_y = find_silver_token(0,80)
    if dist == -1:
           print(":=(")
           return
    dist_g, rot_yg = find_gold_token(rot_y, 40)
    if dist_g < dist:
              return         
    while True:
        dist, rot_y = find_silver_token(0,90)
        if dist < d_th:
            print("grab silver token")
            R.grab()
            print("turning right")
            turn(29.5,2)
            R.release()
            turn(-29.5,2)
            return	            
# when robot is rightly inclined with silver token it moves straight and grab the token, else it first reallign itself. 	              
        elif -a_th <= rot_y <= a_th:
               print("moving towards silver token")
               drive(30,1)
        elif rot_y > a_th:
               print("alligning left")
               turn(2,1)       
        elif rot_y < -a_th:
               print("alligning right")
               turn(-2,1) 
        return     
  
while 1:
    drive(30,0.1)
    lift_silver_token()
    go_away_from_gold()

