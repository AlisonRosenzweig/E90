import sys
import myro
import logging
import timeout_decorator

import drawings
import robots
import svg_processing as svg

logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)

def main():
  num_robots = int(sys.argv[1])
  if num_robots > 1: 
    print "Sorry, this only works with 1 robot right now :("
    exit()
  height = float(sys.argv[2])
  if height <= 0:
    print "Height must be a positive number."
    exit()
  
  svg_file = sys.argv[3]
  try:
    drawing = drawings.Drawing(svg.load_svg(svg_file=svg_file, height=height))
  except(IOError):
    print "Could not open the SVG file, please check filename and try again."
    exit()
  
  robots = load_robots(num_robots)
  
  # NOTE: this will not work with more than one robot - will need to introduce
  # threading and allow whichever robot finishes first to grab a new task.
  while not drawing.is_done:
    next_path = drawing.assign_path(robots[0])
    robots[0].draw_continuous_path(next_path)


def load_robots(num_robots):
  connected_robots = []
  for i in range(num_robots):
    port = raw_input("what port is robot %s connected to? " % i)
    connected = False
    while not connected:
      try:
        myro_obj = connect_to_robot(port)
        connected = True
      except timeout_decorator.TimeoutError:
        print "Robot took too long to connect, will try again."
    connected_robots.append(robots.Robot(myro_obj))
  return connected_robots

@timeout_decorator.timeout(10)
def connect_to_robot(port):
  return myro.Scribbler("/dev/rfcomm" + port)

def draw_square(robot):
  #draw two 4x4 squares, 2 inches apart
  square_1_segs = [(0, 0), (6, 0), (6, 6), (0, 6), (0, 0)]
  square_2_segs = [(2, 6), (2, 10), (6, 10), (6, 6), (2, 6)]
  robot.draw_continuous_path(square_1_segs) 

if __name__ == "__main__":
  main()
