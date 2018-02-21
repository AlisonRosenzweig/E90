import unittest
import myro
import scribblerArt
# from jyro import simulator

class TestRobot(unittest.TestCase):
  def setUp(self):
    self.robot = scribblerArt.Robot()

  def test_angle_to_point_from_origin(self):
    # Test cases with robot at (0, 0)
    inputs = [(1, 1), (1, 0), (0, 1)]
    answers = [45, 0, 90]
    for i, answer in enumerate(answers): 
      next_x, next_y = inputs[i]
      angle = self.robot.angle_to_point(next_x, next_y)
      self.assertAlmostEqual(angle, answer)
  
  def test_angle_to_point_from_1_1(self):
    self.robot.pos_x = 1
    self.robot.pos_y = 1
    inputs = [(0, 0), (2, 0), (0, 2), (2, 2), (2, 1), (1, 2)]
    answers = [-135, -45, 135, 45, 0, 90]
    for i, answer in enumerate(answers):
      next_x, next_y = inputs[i]
      angle = self.robot.angle_to_point(next_x, next_y)
      self.assertAlmostEqual(angle, answer)
    
    

def simple_world(sim):
  sim.addBox(0, 0, 5, 5, fill="backgroundgreen", wallcolor="lightgrey")
  sim.addBox(1, 1, 2, 2, "purple")

if __name__ == '__main__':
  unittest.main()
