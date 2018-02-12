import sys
import myro
import scribblerArt


def main():
  # TODO: add validation that it's a number and in a reasonable range
  num_robots = int(sys.argv[1])
  height = float(sys.argv[2])
  # TODO: extrapolate width from height input SVG is a thing
  # NOTE: for now, assuming square canvases. 
  width = height
  robots = load_robots(num_robots)
  canvas = scribblerArt.Canvas(height, width)
 

def load_robots(num_robots):
  robots = []
  for i in range(num_robots):
    port = raw_input("what port is robot %s connected to? " % i)
    robots.append(scribblerArt.Robot(myro.Scribbler(port)))
  return robots

# TODO: would it be worth it to make a line class segment with a custom sort
# so that objects are sorted by closeness of either end point to a given point?



if __name__ == "__main__":
  main()
