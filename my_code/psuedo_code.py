"""this is a pseudo-code. this is just a rough idea on how the robot code will be made according to the requirement"""

initialize R
define variables for detecting silver and gold tokens.
def drive(speed,seconds):
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0
def turn(speed, seconds):
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = -speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0
    
Make robot recognize silver token.
def find_silver_token(direction, width):
    initialize dist and rot_y # rot_y must be between values direction-width to direction+width
    dist = token.dist
    rot_y = token.rot_y
    if dist > 300:
       return -1,-1
    
    else:
      return dist, rot_y

make robot recognize gold token.
def find_gold_token(direction, width): 
    initialize dist and rot_y # rot_y must be between values direction-width to direction+width
    dist = token.dist
    rot_y = token.rot_y   
    if dist > 300:
       return -1,-1
    
    else:
      return dist, rot_y

make robot avoid gold wall      
def go_away_from_gold():
    
    if dist = -1:
         return
    if dist < d_thg:# d_thg is threshold linear distance value to avoid collision with gold
        robot looks for best direction to turn using left_priority:
             if left is best:
               turn left
             else:
               turn right

def left_priority:
    
    robot compares dist_left and dist_right
    if dist_l > dist_r:
       left is better side to turn
    else:
       right is better side to turn
       
make robot grab the silver token whenever near to robot
def lift_silver_token():
initialize functions find_gold_token(direction, width) and find silver toekn(direction, width)
  if dist = -1: 
      return
  elif dist_g < dist:
       return
       
  elif dist < d_th:
     while True:
        initialize find_silver_token(direction, width)
        robot grabs:
        turns releases
        turn back and move forward
        
        if robot alligned with silver token:
           it moves towards token and grabs
        else:
           it realigns itself with silver token.

while 1:
     drive(speed, seconds)
     call go_away_from_gold():
     call lift_silver_token():        
        
          
    
