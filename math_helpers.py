import numpy as np


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


"""
Interpolation functions to compute velocity based on angle (left) to turn.
"""

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
