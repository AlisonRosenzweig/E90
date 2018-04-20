import svgpathtools
import numpy as np

# TODO: update the default values, take into account units
"""
translate_svg_paths - converts SVG paths to paths in form interpretable 
    by the Robot class in robots.py
parameters:
    paths: list of SVG path objects (class defined in svgpathtools library)
    x_off: 
    y_off:
    scale:
    tol: Maximum allowed distance between a bezier curve and the straight line
        approximation. 
    min_length: Minimum allowed length of a line segment in inches (when 
        approximating bezier curves)
"""
def translate_svg_paths(paths, x_off, y_off, scale, tol=5, min_length=.5):
  robot_paths = []
  
  # Separate each path into the continuous paths.
  contiguous_paths = []
  for path in paths:
    contiguous_paths += path.continuous_subpaths()

  for path in contiguous_paths: 
    # List of Line objects in the current path
    current_path = []
    for seg in path:
      if isinstance(seg, svgpathtools.Line):
        current_path.append(seg)
      else: # Bezier curves and arcs.
        current_path += approximate_with_line_segs(seg, tol, min_length)
    # Convert points to tuples and get rid of redundancies
    if current_path:
      current_path_converted = [complex_to_tuple(line.start, x_off, y_off, scale) for line in current_path]
      current_path_converted.append(complex_to_tuple(current_path[-1].end, x_off, y_off, scale))
      robot_paths.append(current_path_converted) 
        
  return robot_paths

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

  # Check if the line segment is already super short
  if len(line) > min_length:
    # Check if the line segment is within the tolerance at the quartiles
    for p in [.25, .5, .75]:
      if np.abs(seg.point(p) - line.point(p)) > tol:
        left_seg, right_seg = seg.split(.5)
        return (approximate_with_line_segs(left_seg, tol, min_length) + 
                approximate_with_line_segs(right_seg, tol, min_length))
  # Return the line if it's already short or sufficiently close to the curve
  return [line]
