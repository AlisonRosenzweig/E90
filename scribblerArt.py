import myro
import svg
import numpy as np
# parse_path will be useful 
import svgpathtools

DIST_PER_SEC = 2.95 # inches when moving at half speed
# TODO: further refine this value

ANGLE_PER_SEC = 23 # in degrees, assumes turning at 1/4 max
# TODO: this isn't very consistent, so should replace with sensor input

class Canvas:
  def __init__(self, svg_file, height=None):
    # assume robot is initially in the bottom left corner of the canvas (0, 0)
    # and is facing up (in the positive y direction)
    if height:
      self.height = float(height)
    else:
      self.height = None # in this case, will be set by load_svg
    self.width = None # will be set by load_svg
    self.width = height 
    self.segs = None # will be set by load_svg
    
    self.load_svg(svg_file)

  def robot_in_bounds(self, robot):
    if robot.pos_x > self.width or robot.pos_x < 0:
      return False
    if robot.pos_y > self.height or robot.pos_y < 0:
      return False
    return True
  
  def load_svg(self, svg_file):
    # Read in the svg file 
    paths, _, svg_attributes = svgpathtools.svg2paths2(svg_file)

    # Get the height and width of the SVG
    height = float(svg_attributes["height"])
    width = float(svg_attributes["width"])

    if self.height:
      scaling_factor = self.height/height
    else:
      self.height = height
      scaling_factor = 1.0
    
    self.width = scaling_factor*width

    line_segs = self.translate_svg_paths(paths, scaling_factor)
    self.segs = line_segs

  def translate_svg_paths(self, paths, scale):
    line_segs = []
    for path in paths: 
      for seg in path:
        if isinstance(seg, svgpathtools.Line):
          start = (scale*np.real(seg.start), scale*np.imag(seg.start))
          end = (scale*np.real(seg.end), scale*np.imag(seg.end))
          line_segs.append([start, end])
        else:
          # for now do the same thing as above, but will want to handle
          # each case differently in the future by checking for these classes: 
          # svgpathtools.QuadraticBezier, svgpathtools.CubicBezier, 
          # svgpathtools.Arc
          print("Got a more complex segment! Skipping it for now", seg)
    return line_segs
    
    
    

class Robot:
  def __init__(self, myro_obj=None):
    # Assume the robot is initially in the bottom left corner of its canvas
    # and facing in the positive y direction (90 degrees)
    self.pos_x = 0.0
    self.pos_y = 0.0
    self.angle = 90.0
    # The myro object that controls the actual robot! f
    if myro_obj is None:
      print("TEST MODE: NO ROBOT GIVEN")
    self.robot = myro_obj

  """
  angle_to_point - calculates and returns the angle between the robot's current
      posiiton and the given point (next_x, next_y)
  parameters: 
      next_x: X-coordinate of point to get angle pointing towards.
      next_y: Y-coordinate of point to get angle pointing towards.
  returns:
      Float angle in degrees of the vector between the robot's current position
      and the given point.
  """
  def angle_to_point(self, next_x, next_y):
    vec = (next_x - self.pos_x, next_y - self.pos_y)
    if vec == (0, 0):
      return self.angle
    elif vec[0] == 0:
      if vec[1] > 0:
        return 90.
      elif vec[1] < 0:
        return -90.
      else: # current position is equal to next_x, next_y
        return self.angle  # if the points are the same, return current angle
    angle_in_rads = np.arctan2(float(next_y - self.pos_y), (next_x - self.pos_x))
    degs = angle_in_rads * 180 / np.pi
    if degs > 180:
      degs = degs - 360
    return degs

  """
  distance_to_point - calculates and returns the distance between the robot's
      current position and the given point (next_x, next_y).
  parameters:
      next_x: X-coordinate of point to get distance to. 
      next_y: Y-coordinate of point to get distance from. 
  returns: 
      Float distance in inches between the robot's current position and the 
      given point (next_x, next_y).
  """
  def distance_to_point(self, next_x, next_y):
    vec = (next_x - self.pos_x, next_y - self.pos_y)
    return np.sqrt(vec[0]**2 + vec[1]**2)

  """
  turn_to_angle - Instructs the robot to turn to the given desired_angle. 
  parameters:
      desired_angle: Float angle in degrees that robot should be facing after
          function is executed.
  """
  def turn_to_angle(self, desired_angle):
    amt_to_turn = (desired_angle - self.angle)%360
    if amt_to_turn > 180:
        amt_to_turn -= 360
    print("amount to turn: " + str(amt_to_turn))
    print("time to turn: " + str(amt_to_turn/ANGLE_PER_SEC))
    turn_vel = .25
    if amt_to_turn < 0:
      amt_to_turn *= (-1)
      turn_vel *= (-1)
    self.robot.move(0, .25)

    myro.wait(amt_to_turn/ANGLE_PER_SEC)
    """
    i = 0
    while (i < amt_to_turn/ANGLE_PER_SEC): 
      # TODO: make the condition a real thing with sensors
      i += 1
      myro.wait(1)
      pass
    """
    self.robot.stop()

    # NOTE: don't actually want to assume the angle is correct
    # should use sensor data instead
    self.angle = desired_angle

  """
  move_forward_distance: Instructs the robot to move forward by the given 
      distance.
  parameters:
      distance: Float distance in inches that robot should be forward. 
  """
  def move_forward_distance(self, distance):
    secs = distance / DIST_PER_SEC
    print("Time to move forward: " + str(secs))
    speed = .5
    if secs < 0:
      secs *= -1
      speed *= -1
    self.robot.move(speed, 0)
    myro.wait(secs) 
    self.robot.stop() 

  """
  draw_line_seg - instructs the robot to draw a line from (start_x, start_y) to
      (end_x, end_y). 
  parameters: 
      start_x: X-coordinate of first endpoint in line segment to be drawn. 
      start_y: Y-coordinate of first endpoint in line segment to be drawn.
      end_x: X-coordinate of second endpoint in the line segment to be drawn.
      end_y: Y-coordinate of second endpoint in the line segment to be drawn.
  """
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
    print("Next angle: " + str(angle))
    self.turn_to_angle(angle)
    dist = self.distance_to_point(next_x, next_y)
    print("Distance: " + str(dist))
    self.move_forward_distance(dist)
    # TODO: replace the position update with sensor stuff
    self.x = next_x
    self.y = next_y

  """
  pen_up: Instructs robot to lift the pen if pen is not already in the raised
      position. For now, prints statement as a standin for the desired 
      behavior.
  """
  def pen_up(self):
    # TODO: make this actually lift the pen once the servo works
    print("Lift pen")
  
  """
  pen_down: Instructs robot to lower the pen if pen is not already in the 
      lowered position. For now, prints statement as a standin for the desired 
      behavior.
  """
  def pen_down(self):
    # TODO: make this actually lower the pen once the servo works
    print("Lower pen")


