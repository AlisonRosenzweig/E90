import sys
import myro

try:
  myro.init("/dev/rfcomm" + sys.argv[1])
except IndexError:
  print("gimme a port number pls")
  exit()

print("waiting 5 seconds to make sure robot is awake")
for i in range(5):
  myro.wait(1)
  print(5 - i)


wait_time = 1
myro.move(.5, 0)
myro.wait(wait_time)
myro.move(0, 0)
