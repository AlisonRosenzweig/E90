import sys
import myro
import atexit

try:
  myro.init("/dev/rfcomm" + sys.argv[1])
except IndexError:
  print("gimme a port number pls")
  exit()

print("waiting 5 seconds to make sure robot is awake")
for i in range(5):
  myro.wait(1)
  print(5 - i)

num_steps = 0
for i in range(194):
  myro.move(0, .25)
  myro.wait(.01)
  #myro.move(0, 0)
  num_steps += 1
  #myro.wait(1)
myro.move(0, 0)


def exit_handler():
  print(num_steps)


atexit.register(exit_handler)

