import sys
import myro

try:
  myro.init("/dev/rfcomm" + sys.argv[1])
except IndexError:
  print("gimme a port number pls")
  exit()

print(myro.getBattery())
