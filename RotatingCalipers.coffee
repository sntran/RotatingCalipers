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

    xminP = xmaxP = yminP = ymaxP = hull[0]
    
    idxA = 0 # index of vertex with min Y
    idxB = 0 # index of vertex with max Y
    idxC = 0 # index of vertex with min X
    idxD = 0 # index of vertex with max X

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
      return v2;

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

      xminP = point if point[0] < xminP[0]
      xmaxP = point if point[0] > xmaxP[0]
      yminP = point if point[1] < yminP[1]
      ymaxP = point if point[1] > ymaxP[1]

      idxA = idx if point[1] < hull[idxA][1]
      idxB = idx if point[1] > hull[idxB][1]
      idxC = idx if point[0] < hull[idxC][0]
      idxD = idx if point[0] > hull[idxD][0]

    ###console.log "Four extreme points
                : (#{xminP[0]}, #{xminP[1]})
                , (#{xmaxP[0]}, #{xmaxP[1]})
                , (#{yminP[0]}, #{yminP[1]})
                , (#{ymaxP[0]}, #{ymaxP[1]})"###

    ###
            yminP
            /   \
           /     \___
    xminP__|         \
         \          xmaxP
          \         /
           \       /
            \     /
             \   /
              \ /
              ymaxP
    ###

    rotatedAngle = 0
    minArea = null
    minWidth = null
    minHeight = null
    minAPair = null
    minBPair = null
    minCPair = null
    minDPair = null

    caliperA = [1, 0]   # Caliper A points along the positive x-axis
    caliperB = [-1, 0]  # Caliper B points along the negative x-axis
    caliperC = [0, -1]  # Caliper C points along the negative y-axis
    caliperD = [0, 1]   # Caliper D points along the positive y-axis

    while rotatedAngle < Math.PI

      edgeA = getEdge(idxA)
      edgeB = getEdge(idxB)
      edgeC = getEdge(idxC)
      edgeD = getEdge(idxD)

      angleA = getAngle(edgeA, caliperA)
      angleB = getAngle(edgeB, caliperB)
      angleC = getAngle(edgeC, caliperC)
      angleD = getAngle(edgeD, caliperD)
      area = 0

      minAngle = Math.min(angleA, angleB, angleC, angleD)

      caliperA = rotate(caliperA, minAngle)
      caliperB = rotate(caliperB, minAngle)
      caliperC = rotate(caliperC, minAngle)
      caliperD = rotate(caliperD, minAngle)

      if angleA is minAngle
        width = distance(getItem(idxB), getItem(idxA), caliperA)
        height = distance(getItem(idxD), getItem(idxC), caliperC)
      else if angleB is minAngle
        width = distance(getItem(idxA), getItem(idxB), caliperB);
        height = distance(getItem(idxD), getItem(idxC), caliperC);
      else if (angleC == minAngle)
        width = distance(getItem(idxB), getItem(idxA), caliperA);
        height = distance(getItem(idxD), getItem(idxC), caliperC);
      else
        width = distance(getItem(idxB), getItem(idxA), caliperA);
        height = distance(getItem(idxC), getItem(idxD), caliperD);

      rotatedAngle += minAngle
      area = width * height

      if not minArea? or area < minArea
        minArea = area
        minAPair = [getItem(idxA), caliperA]
        minBPair = [getItem(idxB), caliperB]
        minCPair = [getItem(idxC), caliperC]
        minDPair = [getItem(idxD), caliperD]
        minWidth = width
        minHeight = height

      idxA++ if angleA is minAngle
      idxB++ if angleB is minAngle
      idxC++ if angleC is minAngle
      idxD++ if angleD is minAngle

      break if isNaN(rotatedAngle)

    vertices = [
      intersection(minAPair[0], minAPair[1], minDPair[0], minDPair[1]),
      intersection(minDPair[0], minDPair[1], minBPair[0], minBPair[1]),
      intersection(minBPair[0], minBPair[1], minCPair[0], minCPair[1]),
      intersection(minCPair[0], minCPair[1], minAPair[0], minAPair[1])
    ]
    



      