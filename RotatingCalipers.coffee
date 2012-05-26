class window.RotatingCalipers
  ### PRIVATE ###
  _inputVertices = null

  ###
  #._distance
  @sign Number _distance(Array start, Array end, Array point)
  @param start - the start point forming the dividing line.
  @param end - the end point forming the dividing line.
  @param point - the point from which the distance to the line is calculated.
  Find the distance between a point and a line formed by a start and end points.
  All params have first index as the x and second index as y coordinate.

  The real distance value could be calculated as follows:

  Calculate the 2D Pseudo crossproduct of the line vector (start 
  to end) and the start to point vector. 
  ((y2*x1) - (x2*y1))
  The result of this is the area of the parallelogram created by the 
  two given vectors. The Area formula can be written as follows:
  A = |start->end| h
  Therefore the distance or height is the Area divided by the length 
  of the first vector. This division is not done here for performance 
  reasons. The length of the line does not change for each of the 
  comparison cycles, therefore the resulting value can be used to 
  finde the point with the maximal distance without performing the 
  division.

  Because the result is not returned as an absolute value its 
  algebraic sign indicates of the point is right or left of the given 
  line
  ###
  _distance = (start, end, point) ->
    (point[1]-start[1])*(end[0]-start[0])-(point[0]-start[0])*(end[1]-start[1])

  ###
  #._quickHull
  @sign Array _quickHull(Array vertices, Array start, Array end)
  @param vertices - Contains the set of points to calculate the hull for.
                    Each point is an array with the form [x, y].
  @param start - The start point of the line, in the form [x, y].
  @param end - The end point of the line, in the form [x, y].
  @return set of points forming the convex hull, in clockwise order.
  Execute a QuickHull run on the given set of points, using the provided 
  line as delimiter of the search space.
  ###
  _quickHull = (vertices, start, end) ->
    maxPoint = null
    maxDistance = 0

    newPoints = []
    for vertex in vertices when (d = _distance(start, end, vertex)) > 0
      newPoints.push vertex
      continue if d < maxDistance
      maxDistance = d
      maxPoint = vertex

    ###
    The current delimiter line is the only one left and therefore a 
    segment of the convex hull. Only the end of the line is returned 
    to not have points multiple times in the result set.
    ###
    return [end] if not maxPoint?

    ###
    The new maximal point creates a triangle together with start and 
    end, Everything inside this trianlge can be ignored. Everything 
    else needs to handled recursively. Because the quickHull invocation 
    only handles points left of the line we can simply call it for the 
    different line segements to process the right kind of points.
    ###
    _quickHull(newPoints, start, maxPoint)
      .concat _quickHull(newPoints, maxPoint, end)


  ### PUBLIC ###

  ###
  #RotatingCalipers.constructor
  @sign void constructor(Array vertices)
  @sign void RotatingCalipers(Array vertex, Array vertex, Array vertex[, Array vertex...])
  @param vertices - An array contains vertices in form of an array. Can also take 
                    each vertex as arguments
  ###
  constructor: (verticesOrFirst) ->
    throw new Error("Argument required") if not verticesOrFirst?
    throw new Error("Array of vertices required") if not (verticesOrFirst instanceof Array) or verticesOrFirst.length < 3
    [vertex1, vertex2, vertex3, rest...] = verticesOrFirst
    for vertex in verticesOrFirst
      throw new Error("Invalid vertex") if not (vertex instanceof Array) or vertex.length < 2
      throw new Error("Invalid vertex") if isNaN(vertex[0]) or isNaN(vertex[1])

    _inputVertices = verticesOrFirst

  ###
  #RotatingCalipers.convexHull
  @sign Array convexHull(void)
  @return an Array of the points forming the minimal convex set containing all
          input vertices.
  Calculates the convex hull of the arbitrary vertices defined in constructor.
  ###
  convexHull: ->
    finder = (arr) ->
      ret = {}
      ret.min = ret.max = arr[0]
      for el in arr
        ret.min = el if el[0] < ret.min[0]
        ret.max = el if el[0] > ret.max[0]
      ret

    extremeX = finder(_inputVertices)
    _quickHull(_inputVertices, extremeX.min, extremeX.max)
      .concat _quickHull(_inputVertices, extremeX.max, extremeX.min)

  ###
  1. Compute all four extreme points for the polygon, and call them xminP, xmaxP, yminP ymaxP.
  2. Construct four lines of support for P through all four points. 
      These determine two sets of "calipers".
  3. If one (or more) lines coincide with an edge, then compute the area of the rectangle 
      determined by the four lines, and keep as minimum. Otherwise, consider the current 
      minimum area to be infinite.
  4. Rotate the lines clockwise until one of them coincides with an edge of its polygon.
  5. Compute the area of the new rectangle, and compare it to the current minimum area. 
      Update the minimum if necessary, keeping track of the rectangle determining the minimum. 
  6. Repeat steps 4 and 5, until the lines have been rotated an angle greater than 90 degrees.
  7. Output the minimum area enclosing rectangle.
  ###
  minAreaEnclosingRectangle: ->
    hull = @convexHull().reverse()

    # index of vertex with minY, maxY, minX, maxX
    xIndices = [0, 0, 0 ,0]

    getItem = (i) -> hull[i % hull.length]

    # Helper to return the next adjacent edge from an extreme point
    getEdge = (i) ->
      pointA = getItem(i+1)
      pointB = getItem(i)
      [pointA[0]-pointB[0], pointA[1]-pointB[1]]

    getAngle = (v1, v2) ->
      n = v1[0]*v2[0] + v1[1]*v2[1];
      d = Math.sqrt(v1[0]*v1[0] + v1[1]*v1[1])*Math.sqrt(v2[0]*v2[0] + v2[1]*v2[1]);
      return Math.acos(n/d);

    rotate = (v, r) ->
      v2 = [];
      v2[0] = v[0]*Math.cos(r) - v[1]*Math.sin(r);
      v2[1] = v[0]*Math.sin(r) + v[1]*Math.cos(r);
      return v2

    distance = (p, t, v) ->
      return Math.abs(p[0] - t[0]) if v[0] is 0

      a = v[1] / v[0]
      c = t[1] - a*t[0]
      return Math.abs(p[1] - c - a*p[0]) / Math.sqrt(a*a + 1)

    intersection = (p1, v1, p2, v2) ->
      if (v1[0] == 0 && v2[0] == 0)
        return false;

      if (v1[0] != 0)
        m1 = v1[1]/v1[0];
        b1 = p1[1] - m1*p1[0];

      if (v2[0] != 0)
        m2 = v2[1]/v2[0];
        b2 = p2[1] - m2*p2[0];

      if (v1[0] == 0)
        return [p1[0], m2*p1[0] + b2];
      else if (v2[0] == 0)
        return [p2[0], m1*p2[0] + b1];

      if (m1 == m2)
        return false;

      p = [];
      p[0] = (b2 - b1)/(m1 - m2);
      p[1] = m1*p[0] + b1;
      return p

    for point, idx in hull
      xIndices[0] = idx if point[1] < hull[xIndices[0]][1]
      xIndices[1] = idx if point[1] > hull[xIndices[1]][1]
      xIndices[2] = idx if point[0] < hull[xIndices[2]][0]
      xIndices[3] = idx if point[0] > hull[xIndices[3]][0]

    rotatedAngle = 0
    minArea = minWidth = minHeight = null

    # Calipers pointing along +x, -x, -y, +y
    calipers = [ 
      [1, 0], [-1, 0], [0, -1], [0, 1]
    ]

    while rotatedAngle < Math.PI
      angles = (getAngle(getEdge(idx), calipers[i]) for idx, i in xIndices)
      minAngle = Math.min angles...
      calipers = (rotate caliper, minAngle for caliper in calipers)

      idx = angles.indexOf minAngle

      switch idx
        when 0, 2
          width = distance(getItem(xIndices[1]), getItem(xIndices[0]), calipers[0])
          height = distance(getItem(xIndices[3]), getItem(xIndices[2]), calipers[2])
        when 1
          width = distance(getItem(xIndices[0]), getItem(xIndices[1]), calipers[1])
          height = distance(getItem(xIndices[3]), getItem(xIndices[2]), calipers[2])
        when 3
          width = distance(getItem(xIndices[1]), getItem(xIndices[0]), calipers[0])
          height = distance(getItem(xIndices[2]), getItem(xIndices[3]), calipers[3])

      rotatedAngle += minAngle
      area = width * height

      if not minArea? or area < minArea
        minArea = area
        minPairs = ([getItem(xIndices[i]), calipers[i]] for i in [0...4])
        minWidth = width
        minHeight = height

      xIndices[idx]++

      break if isNaN(rotatedAngle)

    vertices = [
      intersection(minPairs[0][0], minPairs[0][1], minPairs[3][0], minPairs[3][1]),
      intersection(minPairs[3][0], minPairs[3][1], minPairs[1][0], minPairs[1][1]),
      intersection(minPairs[1][0], minPairs[1][1], minPairs[2][0], minPairs[2][1]),
      intersection(minPairs[2][0], minPairs[2][1], minPairs[0][0], minPairs[0][1])
    ]

    {vertices: vertices, width: minWidth, height: minHeight, area: minArea}
    



      