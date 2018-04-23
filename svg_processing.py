import svgpathtools
import numpy as np

def load_svg(svg_file, height=None):
  # Read in the svg file. 
  paths, _, svg_attributes = svgpathtools.svg2paths2(svg_file)

  # Find the smallest bounding box that encapsulates all the SVG elements.
  xmin = float("inf")
  xmax = -1
  ymin = float("inf")
  ymax = -1
  
  for path in paths:
    (x1, x2, y1, y2) = path.bbox()
    if x1 < xmin:
      xmin = x1
    if x2 > xmax:
      xmax = x2
    if y1 < ymin:
      ymin = y1
    if y2 > ymax:
      ymax = y2  
 
  # Calc xmax and ymax of the bbox shifted to box with bottom left corner at 
  # (0, 0) and top right corner at (xmax, ymax).
  xmax -= xmin
  ymax -= ymin

  # If a height is given, calculate the scale factor from bbox dimensions.
  if height:
    scaling_factor = height/ymax
  else:
    scaling_factor = 1.0

  # Translate the SVG paths into robot paths, xmin and ymin as offsets.
  robot_paths = translate_svg_paths(paths, xmin, ymin, scaling_factor)
  return robot_paths


"""
translate_svg_paths - converts SVG paths to paths in form interpretable 
    by the Robot class in robots.py
parameters:
    paths: List of SVG path objects (class defined in svgpathtools library)
    x_off: Offset to be subtracted from x coordinate before scaling.
    y_off: Offset to be subtracted from y coordinate before scaling.
    scale: Scaling factor to scale drawing to available canvas size.
    tol: Maximum allowed distance between a bezier curve and the straight line
        approximation. 
    min_length: Minimum allowed length of a line segment in inches (when 
        approximating bezier curves)
"""
def translate_svg_paths(paths, x_off, y_off, scale, tol=.5, min_length=2):
  robot_paths = []
  
  # Separate each path into the continuous paths.
  contiguous_paths = []
  for path in paths:
    contiguous_paths += path.continuous_subpaths()

  for path in contiguous_paths: 
    # List of Line objects in the current path.
    current_path = []
    for seg in path:
      if isinstance(seg, svgpathtools.Line):
        current_path.append(seg)
      else: # Bezier curves and arcs.
        current_path += approximate_with_line_segs(seg, tol, min_length)
    # Convert points to tuples and get rid of redundancies.
    if current_path:
      current_path_converted = [
        complex_to_tuple(line.start, x_off, y_off, scale) for line in
        current_path]
      current_path_converted.append(
        complex_to_tuple(current_path[-1].end, x_off, y_off, scale))
      robot_paths.append(current_path_converted) 
        
  return Polyline(robot_paths)


"""
complex_to_tuple - converts a complex number to an (x, y) tuple, incorporating
    offsets (x_off, y_off) and scaling factor.
paramters:
    complex_num: complex number to be converted in the form x + y*i.
    x_off: Offset in x coordinate to be subtracted from the converted tuple. 
    y_off: Offset in y coordinate to be subtracted from the converted tuple.
    scale: Factor the converted tuple will be scaled by after subtracting
        offset values.
returns:
    Tuple of floats (x, y).
"""
def complex_to_tuple(complex_num, x_off, y_off, scale):
  return (np.real(complex_num) - x_off)*scale, (np.imag(complex_num) - y_off)*scale


# TODO: min length is checked before scaling- should scale while still bezier.
"""
approximate_with_line_segs - Approximates a bezier curve with striaght line
    segments.
parameters: 
    seg: The quadratic or cubic bezier curve segment to be approximated.
    tol: Maximum allowed distance between the curve and the striaght line
        approximation in inches. 
    min_length: Minimum allowed length of a line segment in the approximation.
        Note that this constraint gets priority over the tolerance. 
"""
def approximate_with_line_segs(seg, tol, min_length):
  # Potential line approximation.
  line = svgpathtools.Line(seg.start, seg.end)

  # Check if the line segment is already super short.
  if len(line) > min_length:
    # Check if the line segment is within the tolerance at the quartiles.
    for p in [.25, .5, .75]:
      if np.abs(seg.point(p) - line.point(p)) > tol:
        left_seg, right_seg = seg.split(.5)
        return (approximate_with_line_segs(left_seg, tol, min_length) + 
                approximate_with_line_segs(right_seg, tol, min_length))
  # Return the line if it's already short or sufficiently close to the curve.
  return [line]


class Polyline:
  def __init__(self, points):
    self.points = points
    if points[0]==points[-1]:
      self.is_closed = True
    else:
      self.is_closed = False

  """
  distance_to_endpoint - gives the distance from the given point to the closer
    of the two endpoints, or to the closest of all of the endpoints if the
    polyline is a closed shape.
  """
  def distance_to_endpoint(self, p):
    if not self.is_closed:
      # Just return the distance to the closer endpoint
      dist_to_start = self.distance_to_point(p, 0)
      dist_to_end = self.distance_to_point(p, -1)
      if dist_to_start < dist_to_end:
        return dist_to_start, 0
      else:
         return dist_to_end, -1
    else:
      dists = [self.distance_to_point(p, i) for i in range(len(self.points))]
      return min(dists), np.argmin(dists)

  """
  distance_to_point - gives the distance between given point p and point at
    index i in the polyline.
  """
  def distance_to_point(self, p, i):
    return np.sqrt((p[0] - self.points[i][0])**2 + (p[1] - self.points[i][1])**2)

