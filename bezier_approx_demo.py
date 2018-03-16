import numpy as np
import svgpathtools


def translate_path_demo(path, tol=5, min_length=5):
  max_depth = 0
  done = False
  while not done:
    print max_depth
    # List of Line objects in the current path
    current_path = []
    for seg in path:
      if isinstance(seg, svgpathtools.Line):
        current_path.append(seg)
      else: # Bezier curves and arcs.
        approx_path, done = approximate_with_line_segs(seg, tol, min_length, 0, max_depth)
        current_path += approx_path
    # save an svg
    svgpathtools.disvg([path] + current_path, filename=("bez_demo_" + str(max_depth) + ".svg"))
    # TODO: add some nodes - different color for the last ones generated? 
    max_depth += 1

def approximate_with_line_segs(seg, tol, min_length, depth, max_depth):
  # Potential line approximation.
  line = svgpathtools.Line(seg.start, seg.end)
  done = True
  # Check if the line segment is already super short
  if line.length() > min_length:
    # Check if the line segment is within the tolerance at the quartiles
    for p in [.25, .5, .75]:
      if np.abs(seg.point(p) - line.point(p)) > tol:
        done = False
        break
  if depth < max_depth and not done:
    left_seg, right_seg = seg.split(.5)
    path_left, done_left = approximate_with_line_segs(left_seg, tol, min_length, depth + 1, max_depth)
    path_right, done_right = approximate_with_line_segs(right_seg, tol, min_length, depth + 1, max_depth)
  
    return (path_left + path_right), (done_left and done_right)
  
  # Return the line if it's already short or sufficiently close to the curve
  return [line], done

paths, _, _ = svgpathtools.svg2paths2("curve.svg")
translate_path_demo(paths[0])
