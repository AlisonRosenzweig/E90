import myro
import numpy as np
import svgpathtools
import requests
import json
import logging

import svg_processing as svg


PI_URL = "http://192.168.1.115:5000"

DIST_PER_SEC = 2.95 # inches when moving at half speed
# TODO: further refine this value

ANGLE_PER_SEC = 28 # In degrees, assumes turning at 1/4 max
# TODO: this isn't very consistent, so should replace with sensor input

ANGLE_TOLERANCE = .5 # In degrees.

MAX_TURN_VEL = .2

FORWARD_SPEED = .5

# Servo angles corresponding to the up and down positions of the pen.
UP_ANGLE = 135
DOWN_ANGLE = 90

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
    xmin = float("inf")
    xmax = -1
    ymin = float("inf")
    ymax = -1
    for path in paths:
      (x1, x2, y1, y2) = path.bbox()
      if x1 < xmin:
        xmin = x1
      if x2 > xmax:
        xmax = x2
      if y1 < ymin:
        ymin = y1
      if y2 > ymax:
        ymax = y2  

    xmax -= xmin
    ymax -= ymin

    if self.height:
      scaling_factor = self.height/ymax
    else:
      self.height = ymax
      scaling_factor = 1.0

    self.width = scaling_factor*xmax

    # translate the SVG paths into robot paths, xmin and ymin as offsets
    robot_paths = svg.translate_svg_paths(paths, xmin, ymin, scaling_factor)
    self.paths = robot_paths


class Robot:
  def __init__(self, myro_obj=None, robot_url=PI_URL):
    # Assume the robot is initially in the bottom left corner of its canvas
    # and facing in the positive y direction (90 degrees)
    self.pos_x = 0.0
    self.pos_y = 0.0
    self.angle = 90.0
    self.sensor_url = robot_url + "/bno"
    self.robot_url = robot_url
    calibrated = self.is_sensor_calibrated()
    if not calibrated:
      # Just print once...
      print("Waiting on sensor calibration...")
    while not calibrated: 
      calibrated = self.is_sensor_calibrated()
      
    raw_input("Ready to go? When robot is in place hit [Enter] to continue.")
    # Once the robot is in position, record offset between absolute and relative headings. 
    # Heading value is backwards so subtract from 360 to get true CCW+ orientation. 
    abs_heading = self.get_absolute_heading()
    self.angle_offset = abs_heading - 90

    # The myro object that controls the actual robot!
    if myro_obj is None:
      print("TEST MODE: NO ROBOT GIVEN")
    self.robot = myro_obj

  def get_relative_heading(self):
    relative_heading = self.get_absolute_heading() - self.angle_offset
    self.angle = relative_heading
    return relative_heading
  
  def get_absolute_heading(self):
    r = requests.get(self.sensor_url)
    if r.status_code == requests.codes.OK:
      data = json.loads(r.content)
      quat_x, quat_y, quat_z, quat_w = data["quatX"], data["quatY"], data["quatZ"], data["quatW"]
      x, y, z = quat_to_euler(quat_x, quat_y, quat_z, quat_w)
      return x 
    

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
      turn_vel_fn: Function that calculates the turning veocity as a function of
          angle left to turn.
  """
  def turn_to_angle(self, desired_angle, turn_vel_fn, max_vel):
    angle_delta = self.calculate_angle_delta(desired_angle)
    logging.info("amount to turn: " + str(angle_delta))

    while not np.abs(angle_delta) < ANGLE_TOLERANCE: 
      turn_vel = turn_vel_fn(angle_delta, max_vel)
      self.robot.move(0, turn_vel)
      logging.debug("Heading: " + str(self.angle))
      logging.debug("Desired: " + str(desired_angle))
      logging.debug("Angle delta: " + str(angle_delta))
      logging.debug("Turn Direction: " + str(np.sign(turn_vel)))
      angle_delta = self.calculate_angle_delta(desired_angle) 

    self.robot.stop()

    # Update heading one more time in case it changed while stopping.
    self.get_relative_heading()

  """
  calculate_angle_delta - Returns the angle to turn given the desired angle.
  """
  def calculate_angle_delta(self, desired_angle):
    cur_angle = self.get_relative_heading()
    # Get positive value of amount to turn.
    # Adds 360 before % to make negative angles easier to deal with.
    angle_delta = (desired_angle - cur_angle + 360)%360
    if angle_delta > 180: 
      angle_delta -= 360
    return angle_delta

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
    speed = FORWARD_SPEED
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
    self.turn_to_angle(angle, turn_vel_binary, MAX_TURN_VEL) # TODO: make sure this is correct version!!!
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
    self.move_pen(UP_ANGLE)
    print("Lift pen")
  
  """
  pen_down: Instructs robot to lower the pen if pen is not already in the 
      lowered position. For now, prints statement as a standin for the desired 
      behavior.
  """
  def pen_down(self):
    self.move_pen(DOWN_ANGLE)
    print("Lower pen")
  
  def move_pen(self, angle):
    r = requests.get(self.robot_url + "/move_to_" + str(angle)) 
    if r.status != requests.OK:
      print("Couldn't connect to server to move pen... :( ")
      exit()
    # Sleep for a second to give the servo time to move.
    time.sleep(1)


def turn_vel_linear(angle_delta, max_vel):
  return max_vel*float(angle_delta)/180

def turn_vel_binary(angle_delta, max_vel):
  return max_vel*np.sign(angle_delta)

def turn_vel_sinusoidal(angle_delta, max_vel):
  vel = max_vel*(1 - np.cos(np.deg2rad(angle_delta)))
  if angle_delta >= 0:
    return vel
  else:
    return -vel


"""
quat_to_euler: Helper function to convert quaternions to euler angles. 
  Math for this is taken from the Adafruit_BNO055 C library method toEuler()
"""
def quat_to_euler(x, y, z, w):
  sqw = w*w
  sqx = x*x
  sqy = y*y
  sqz = z*z

  euler_x = np.arctan2((2.0*(x*y + z*w)), (sqx-sqy-sqz+sqw));
  euler_y = np.arcsin(-2.0*(x*z - y*w)/(sqx+sqy+sqz+sqw));
  euler_z = np.arctan2(2.0*(y*z + x*w), (-sqx - sqy + sqz + sqw));
  
  return (np.rad2deg(euler_x), np.rad2deg(euler_y), np.rad2deg(euler_z))
