import sys
import myro
import scribblerArt

try:
  robo = scribblerArt.Robot(myro.Scribbler("/dev/rfcomm" + sys.argv[1]))
except IndexError:
  print("gimme a port number pls")
  exit()

print("waiting 5 seconds to make sure robot is awake")
for i in range(5):
  myro.wait(1)
  print(5 - i)

robo.move_forward_distance(10)
