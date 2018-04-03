import myro
import svg
import numpy as np
import svgpathtools
import requests
import json


SENSOR_URL = "http://192.168.1.115:5000/bno"

DIST_PER_SEC = 2.95 # inches when moving at half speed
# TODO: further refine this value

ANGLE_PER_SEC = 28 # 23 # in degrees, assumes turning at 1/4 max
# TODO: this isn't very consistent, so should replace with sensor input

ANGLE_TOLERANCE = 10.0 # Two degrees

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
    try:
      height = float(svg_attributes["height"])
      width = float(svg_attributes["width"])
    except KeyError:
      pass # TODO: should use bounding boxes and such

    if self.height:
      scaling_factor = self.height/height
    else:
      self.height = height
      scaling_factor = 1.0
    
    self.width = scaling_factor*width

    robot_paths = translate_svg_paths(paths, scaling_factor)
    self.paths = robot_paths

# TODO: update the default values, take into account units
def translate_svg_paths(paths, scale, tol=5, min_length=.5):
  robot_paths = []
  
  # Separate each path into the continuous paths.
  contiguous_paths = []
  for path in paths:
    contiguous_paths += path.continuous_subpaths()

  for path in contiguous_paths: 
    # List of Line objects in the current path
    current_path = []
    for seg in path:
      if isinstance(seg, svgpathtools.Line):
        current_path.append(seg)
      else: # Bezier curves and arcs.
        current_path += approximate_with_line_segs(seg, tol, min_length)
    # Convert points to tuples and get rid of redundancies
    if current_path:
      current_path_converted = [complex_to_tuple(line.start, scale) for line in current_path]
      current_path_converted.append(complex_to_tuple(current_path[-1].end, scale))
      robot_paths.append(current_path_converted) 
        
  return robot_paths


def complex_to_tuple(complex_num, scale):
  return (np.real(complex_num)*scale, np.imag(complex_num)*scale)

def approximate_with_line_segs(seg, tol, min_length):
  # Potential line approximation.
  line = svgpathtools.Line(seg.start, seg.end)

  # Check if the line segment is already super short
  if len(line) > min_length:
    # Check if the line segment is within the tolerance at the quartiles
    for p in [.25, .5, .75]:
      if np.abs(seg.point(p) - line.point(p)) > tol:
        left_seg, right_seg = seg.split(.5)
        return (approximate_with_line_segs(left_seg, tol, min_length) + 
                approximate_with_line_segs(right_seg, tol, min_length))
  # Return the line if it's already short or sufficiently close to the curve
  return [line]


class Robot:
  def __init__(self, myro_obj=None, sensor_url=SENSOR_URL):
    # Assume the robot is initially in the bottom left corner of its canvas
    # and facing in the positive y direction (90 degrees)
    print("creating the robot??")
    self.pos_x = 0.0
    self.pos_y = 0.0
    self.angle = 90.0
    self.sensor_url = sensor_url
    calibrated = self.is_sensor_calibrated()
    while not calibrated: 
      print("Waiting on sensor calibration...")
      myro.wait(5) # Wait 5 seconds before checking again.
      calibrated = self.is_sensor_calibrated()
      
    raw_input("Ready to go? When robot is in place hit [Enter] to continue.")
    # Once the robot is in position, record offset between absolute and relative headings. 
    abs_heading = json.loads(requests.get(self.sensor_url).content)["heading"]
    self.angle_offset = abs_heading - 90

    # The myro object that controls the actual robot!
    if myro_obj is None:
      print("TEST MODE: NO ROBOT GIVEN")
    self.robot = myro_obj

  def get_relative_heading(self):
    r = requests.get(self.sensor_url)
    if r.status_code == requests.codes.OK:
      data = json.loads(r.content)
      relative_heading = data["heading"] - self.angle_offset
      self.angle = relative_heading
      return relative_heading
    # Right eturns none if can't get the sensor data. May want to modify this behavior. 

  def is_sensor_calibrated(self):
    r = requests.get(self.sensor_url)
    if r.status_code != requests.codes.OK:
      print("Can't get sensor data - check sensor server.")
      return False
    data = json.loads(r.content)

    # Check the gyroscope is calibrated and the "system" is calibrated.
    # If system isn't calibrated - heading values will jump from relative to
    # absolute as soon as the system calibrates.
    return (data["calSys"] > 0) and (data["calGyro"] > 0)

  """
  angle_to_point - calculates and returns the angle between the robot's current
      position and the given point (next_x, next_y)
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
    degs = np.rad2deg(
        np.arctan2(float(next_y - self.pos_y), (next_x - self.pos_x)))
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
    # Don't really care about amount to turn, just about which direction to turn (CW/CCW).
    amt_to_turn = (desired_angle - self.angle)%360
    if amt_to_turn > 180:
        amt_to_turn -= 360
    print("amount to turn: " + str(amt_to_turn))
    # print("time to turn: " + str(amt_to_turn/ANGLE_PER_SEC))
    max_turn_vel = .25
    # TODO: maybe get rid of that? (and make above a magnitude)
    if amt_to_turn < 0:
      max_turn_vel *= (-1)

    cur_heading = self.get_relative_heading()
    angle_delta = desired_angle - cur_heading
    while not np.abs(angle_delta) < ANGLE_TOLERANCE: 
      # TODO: add a condition to change turn_vel in cases of overshoot
      self.robot.move(0, max_turn_vel*float(angle_delta)/180)
      # get the current heading
      angle_delta = desired_angle - self.get_relative_heading()
      
    self.robot.stop()
    self.get_relative_heading() # Get heading one more time in case stopping moved a little"


  def turn_to_angle_old(self, desired_angle):
    amt_to_turn = (desired_angle - self.angle)%360
    if amt_to_turn > 180:
        amt_to_turn -= 360
    print("amount to turn: " + str(amt_to_turn))
    # print("time to turn: " + str(amt_to_turn/ANGLE_PER_SEC))
    turn_vel = -.25
    if amt_to_turn < 0:
      amt_to_turn *= (-1) # still necessary for old version of code
      turn_vel *= (-1)

    # Old version - use for comparison
    self.robot.move(0, turn_vel)
    myro.wait(float(amt_to_turn)/ANGLE_PER_SEC)
    self.robot.stop()
    self.angle = desired_angle # Assume robot made it to the correct angle

  """
  move_forward_distance: Instructs the robot to move forward by the given 
      distance.
  parameters:
      distance: Float distance in inches that robot should be forward. 
  """
  def move_forward_distance(self, distance):
    secs = distance / DIST_PER_SEC
    print("Heading: " + str(self.angle))
    print("Time to move forward: " + str(secs))
    speed = .5
    if secs < 0:
      secs *= -1
      speed *= -1
    self.robot.move(speed, 0)
    myro.wait(secs) 
    self.robot.stop()
    self.pos_x = self.pos_x + distance*np.cos(np.deg2rad(self.angle))
    self.pos_y = self.pos_y + distance*np.sin(np.deg2rad(self.angle))

  """
  draw_continuous_path - instructs the robot to draw a path connecting all of
      the given points.
  parameters: 
      points: list of points
  """
  def draw_continuous_path(self, points): 
    if len(points) < 2: 
      raise ValueError("Can't have a path with fewer than 2 points")
      # TODO: handle case of drawing points? 

    # Get to the starting point without drawing
    self.pen_up() # NOTE: this will likely be redundant so maybe take it out 
    start_x, start_y = points.pop(0)
    self.go_straight_to_point(start_x, start_y) 

    # Draw all the subsequent points 
    self.pen_down()
    for x, y in points:
      self.go_straight_to_point(x, y)
    
    # Pick pen up when done
    self.pen_up()

  """
  go_straight_to_point: turns and moves from current position to given position.
      combines calculating angle, turning to that angle, calculating
      distance, and moving by that distance. 
  """
  def go_straight_to_point(self, next_x, next_y):    
    angle = self.angle_to_point(next_x, next_y)
    print("Current position:" + str(self.pos_x) + ", " + str(self.pos_y))
    print("Current heading: " + str(self.angle))
    print("Next angle: " + str(angle))
    self.turn_to_angle(angle) # TODO: make sure this is correct version!!!
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


