import sys
import myro
import scribblerArt


def main():
  # TODO: add validation that it's a number and in a reasonable range
  num_robots = int(sys.argv[1])
  height = float(sys.argv[2])
  svg_file = sys.argv[3]
  robots = load_robots(num_robots)
  canvas = scribblerArt.Canvas(svg_file=svg_file, height=height)
  
  # have one robot draw the SVG
  for path in canvas.paths:
    robots[0].draw_continuous_path(path)
    
  
def load_robots(num_robots):
  robots = []
  for i in range(num_robots):
    port = raw_input("what port is robot %s connected to? " % i)
    robots.append(scribblerArt.Robot(myro.Scribbler("/dev/rfcomm"+port)))
  return robots

def draw_square(robot):
  #draw two 4x4 squares, 2 inches apart
  square_1_segs = [(0, 0), (0, 4), (4, 4), (4, 0), (0, 0)]
  square_2_segs = [(2, 6), (2, 10), (6, 10), (6, 6), (2, 6)]
  robot.draw_continuous_path(square_1_segs) 

# TODO: would it be worth it to make a line class segment with a custom sort
# so that objects are sorted by closeness of either end point to a given point?


if __name__ == "__main__":
  main()
