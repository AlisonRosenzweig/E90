import myro
import svg.path as svg  
import import numpy as np
# parse_path will be useful 

DIST_PER_SEC = 1 # TODO: fill this in with an actual value

class Canvas:
  def __init__(self, height, svg):
    # assume robot is initially in the bottom left corner of the canvas (0, 0)
    # and is facing up (in the positive y direction)
    self.svg = svg
    self.height = height
    # TODO: calculate the width based on the height and the ratio in the svg
    self.width = height 


  def robot_in_bounds(self, robot):
    if robot.pos_x > self.width or robot.pos_x < 0:
      return False
    if robot.pos_y > self.height or robot.pos_y < 0:
      return False
    return True


class Robot:
  def __init__(self, myro_obj):
    # Assume the robot is initially in the bottom left corner of its canvas
    # and facing in the positive y direction (90 degrees)
    self.pos_x = 0.0
    self.pos_y = 0.0
    self.angle = 90.0
    # The myro object that controls the actual robot! 
    self.robot = myro_obj

  def angle_to_point(self, next_x, next_y):
    vec = (next_x - self.pos_x, next_y - self.pos_y)
    angle_in_rads = np.tan((next_y - self.pos_y), (next_x - self.pos_x))
    return angle_in_rads * 180 / np.pi

  def distance_to_point(self, next_x, next_y):
    vec = (next_x - self.pos_x, next_y - self.pos_y)
    return np.sqrt(vec[0]**2 + vec[1]**2)

  def turn_to_angle(self, desired_angle):
    amt_to_turn = (desired_angle - self.angle)%360
    self.robot.move(0, .5)
    i = 0
    while (i < 10): # TODO: make the condition a real thing with sensors
      i += 1
      pass
    self.robot.stop()
    # TODO: update self.angle (either here or inside the loop)
    # TODO: should look back at E28 code to see how we did the closed loop 

  def move_forward_distance(self, distance):
    secs = distance / DIST_PER_SEC
    self.robot.move(.5, 0)
    myro.wait(secs) 
    self.robot.stop() 

  def draw_line_seg(self, start_x, start_y, end_x, end_y): 
    # Get to the starting point without drawing
    self.pen_up() # TODO: this will likely be redundant so maybe take it out
    self.go_straight_to_point(start_x, start_y)
    
    # Put the pen down and draw the line 
    self.pen_down()
    self.go_straight_to_point(end_x, end_y)
    self.pen_up()

  """
  go_straight_to_point: turns and moves from current position to given position.
      combines calculating angle, turning to that angle, calculating
      distance, and moving by that distance. 
  """
  def go_straight_to_point(self, next_x, next_y):    
    angle = self.angle_to_point(next_x, next_y)
    self.turn_to_angle(angle)
    dist = self.distance_to_point(next_x, next_y)
    self.move_forward_distance(dist)

  def pen_up(self):
    # TODO: make this actually lift the pen once the servo works
    print("Lift pen")

  def pen_down(self):
    # TODO: make this actually lower the pen once the servo works
    print("Lower pen")
