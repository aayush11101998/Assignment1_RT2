# Assignment_1
### Abstract
Assignment 1 is a python based code that simulates an autonomous moving robot on a given path. The tasks given for the robot are to pickup and move aside the silver tokens and to stay away from gold tokens while moving in the environment.
### Introduction
The Assignment 1 is modified by me which was initially provided by Prof. Carmine Recchiuto. Functions such as grab(), release(), see() are used in the assignment. Methodology will show what logic is exactly used in the assignment and what each function does.
### Methodology
The robot has to do certain tasks while it covers the whole path specified in the environment.
###### 1. Robot moves
The first task is to move in the path. Define a function ***DRIVE*** the robot has two motors to activate m0 and m1. When same amount of speed is given to both motors the the robot moves forward or backward (+,-) defines whether the robot move forward or backward.  
###### 2. Robot turns
The second task of the robot is to turn. Define a function ***Turn*** the robot has two motors m0 and m1. When same amount of speed is given to the motors but each has direction different to the other then the robot turns.
###### 3. Robot see, grab and release
There is no need to make a function for this, the commands that are used:  R.see(), R.grab(), R.release() for the above tasks to happen. The R is a robot() which defines these functions in the folder Robot().
###### 4. Recognise silver token
Create variables "dist" and "rot_y" to know the position of robot from silver token. dist tells the distance of robot from silver token whereas rot_y tells the position. If the token.dist returns dist and token.rot_y returns rot_y then the token is a silver token.
The robot moves in the direction of token if it is alligned with the token i.e the robot and token make an angle less then the variable we define to check allignment of robot with token. If the angle is greater then the threshold value defined to check the allignment it has to first turn itself and make itself alligned with token. Once the robot checks its allignment and drives itself close to the token which is actually defined by another threshold value defined to check the distance of robot from the silver token, once the distance of robot is less than the threshold value it grabs the token turns back and then release it.
###### 5. Recognise gold token
Just like silver token we create variables "dist_g" and "rot_yg" to know the position of robot from gold token. dist_g tells the distance of robot from gold token whereas rot_yg tells the position. If the token.dist returns dist_g and token.rot_y returns rot_yg then the token is a gold token. 
The gold token need to be avoided. The robot must first make a side its priority be it left or right, i.e once the robot comes to a corner it should turn in the opposite direction of the gold token walls towards the path it has not covered. Threshold values for rot_yg are created in order to avoid collision and to choose direction. If the distance of robot and gold token is less then the threshold value the robot turn to the direction where the valid path exists. To decide whether robot turns right or left compare the distance between robot and gold token in both direction and the direction that is greater robot turns in that direction.
### Result
Code is in assignment 1 and can be run using ***python run.py assignment 1.py.*** Robot follows the path specified in the environment

