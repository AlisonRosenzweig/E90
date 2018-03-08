import numpy as np
import svgpathtools


def translate_path_demo(path, tol=5, min_length=5):
  max_dept = 0
  done = False
  while not done:
    # List of Line objects in the current path
    current_path = []
    for seg in path:
      if isinstance(seg, svgpathtools.Line):
        current_path.append(seg)
      else: # Bezier curves and arcs.
        approx_path, done = approximate_with_line_segs(seg, tol, min_length)
        current_path += approx_path
    # save an svg
    svgpathtools.disvg(current_path, openinbrowser=False, svgattributes={"background": "#ffffff"})
    # TODO: add some nodes 
    max_depth += 1
  return robot_paths

def approximate_with_line_segs(seg, tol, min_length, depth, max_depth):
  # Potential line approximation.
  line = svgpathtools.Line(seg.start, seg.end)

  # Check if the line segment is already super short
  if len(line) > min_length and depth < max_depth:
    # Check if the line segment is within the tolerance at the quartiles
    for p in [.25, .5, .75]:
      if np.abs(seg.point(p) - line.point(p)) > tol:
        left_seg, right_seg = seg.split(.5)
        return (approximate_with_line_segs(left_seg, tol, depth + 1, max_depth) + 
                approximate_with_line_segs(right_seg, tol, depth + 1, max_depth))
  
  # Return the line if it's already short or sufficiently close to the curve
  return [line]

