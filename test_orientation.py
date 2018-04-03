import sys
import scribblerArt
import myro
import csv
import json
import requests

try:
  myrobo = myro.Scribbler("/dev/rfcomm" + sys.argv[1])
except IndexError:
  print("gimme a port number pls")
  exit()

print(myro.getBattery())
robot = scribblerArt.Robot(myrobo)
relative_headings = []
absolute_headings = []

myrobo.move(0, .1)
for i in range(100):
  print(str(i))
  myro.wait(.1)
  relative_headings.append(robot.get_relative_heading())
  abs_heading = json.loads(requests.get(robot.sensor_url).content)["heading"]
  absolute_headings.append(abs_heading)
myrobo.stop()

with open('orientation_' + str(robot.angle_offset) + ".csv", 'w') as csvfile:
  writer = csv.writer(csvfile)
  for i in range(100):
    writer.writerow([absolute_headings[i], relative_headings[i]])
    
