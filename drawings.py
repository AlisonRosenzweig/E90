import svg_processing as svg

class Drawing:
  def __init__(self, paths):
    self.paths = paths

  def assign_path(self, robot):
    self.paths = sorted(
      self.paths, key = lambda p : p.distance_to_endpoint((robot.pos_x, robot.pos_y)))
    return self.paths.pop(0)

  @property
  def is_done(self):
    return (len(self.paths) == 0)
