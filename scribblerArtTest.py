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
    

class TestCanvas(unittest.TestCase):
  def setUp(self): 
    self.square_svg_file = "square.svg"
    
  def test_load_svg_simple(self):
    self.canvas = scribblerArt.Canvas(self.square_svg_file, 500.00)
    self.assertAlmostEqual(self.canvas.height, 500.)
    self.assertAlmostEqual(self.canvas.width, 500.)
    expected_segs = [[(100, 100), (100, 400)], 
                     [(100, 400), (400, 400)], 
                     [(400, 400), (400, 100)],
                     [(400, 100), (100, 100)]]
    # sort the points inside the lists
    expected_segs = [point.sort() for point in expected_segs]
    result_segs = [point.sort() for point in self.canvas.segs]
    self.assertItemsEqual(expected_segs, result_segs)

  def test_load_svg_with_scale(self):
    self.canvas = scribblerArt.Canvas(self.square_svg_file, 250)
    self.assertAlmostEqual(self.canvas.height, 250)
    self.assertAlmostEqual(self.canvas.width, 250)
    expected_segs = [[(50, 50),   (50, 200)], 
                     [(50, 200),  (200, 200)], 
                     [(200, 200), (200, 50)],
                     [(200, 50),  (50, 50)]]
    # sort the points inside the lists
    expected_segs = [point.sort() for point in expected_segs]
    result_segs = [point.sort() for point in self.canvas.segs]
    self.assertItemsEqual(expected_segs, result_segs) 

if __name__ == '__main__':
  unittest.main()
