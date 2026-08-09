using System.Collections.Generic;
using DelaunatorSharp;
using GenericCode;

namespace TriangulationNavigation
{
    public class NavMesh
    {
        public Delaunator delaunator;
        public Constrainautor constrainautor;

        public List<Float2> allPoints;
        public List<Triangle> allTriangles;
        public List<AABB> allTriangleBounds;
        public List<Float2> allTriangleCentroids;
        public KDTree2D allTriangleCentroidsKdTree;
        public List<Edge> allEdges;

        List<List<int>> edgesAroundPointsMap;

        public List<int> unwalkableTriangles;
        public List<int> unwalkableTrianglesObstacleIndices;
        public List<int> walkableTriangles;

        public List<int> trianglesWalkability;
        public List<int> allToUnwalkableTriangleIndices;
        public List<List<int>> obstacleIntersections;
        public List<int> obstacleWalkablityIndices;
        public List<bool> isObstacleCornerIntersectingWithWorldBounds;
        public List<int> obstacleHullEdges;
        public List<List<int>> obstacleHullEdgesByObstacles;
        public List<int> obstacleHullEdgeObstacleIndices;
        public List<int> hullEdgeTriangulationEdgeToObstacleIndices;
        public List<bool> edgesWalkability;
        public float smallestHullEdgeSize;
        public AABB worldBounds;
        public int totalNumberOfWorldBoundCorners;
        ManualRandom random;
        TriangulationGridSearch allTriangulationGridSearch;
        TriangulationGridSearch walkableTriangulationGridSearch;
        TriangulationGridSearch unwalkableTriangulationGridSearch;
        List<bool> visitedTriangles;

        public NavMesh()
        {
            delaunator = new Delaunator();
            constrainautor = new Constrainautor();

            allPoints = new List<Float2>();
            obstacleWalkablityIndices = new List<int>();
            isObstacleCornerIntersectingWithWorldBounds = new List<bool>();

            unwalkableTriangles = new List<int>();
            walkableTriangles = new List<int>();
            unwalkableTrianglesObstacleIndices = new List<int>();
            trianglesWalkability = new List<int>();
            allToUnwalkableTriangleIndices = new List<int>();
            obstacleIntersections = new List<List<int>>();

            allTriangleCentroids = new List<Float2>();

            edgesWalkability = new List<bool>();

            edgesAroundPointsMap = new List<List<int>>();
            hullEdgeTriangulationEdgeToObstacleIndices = new List<int>();

            obstacleHullEdges = new List<int>();
            obstacleHullEdgesByObstacles = new List<List<int>>();
            obstacleHullEdgeObstacleIndices = new List<int>();

            visitedTriangles = new List<bool>();

            random = new ManualRandom(8);
        }

        public void Create(List<Obstacle> obstacles, float worldSize)
        {
            AABB bounds = new AABB
            {
                minX = -0.5f * worldSize,
                maxX = 0.5f * worldSize,
                minY = -0.5f * worldSize,
                maxY = 0.5f * worldSize
            };
            Create(obstacles, bounds);
        }

        public void Create(List<Obstacle> obstacles, AABB bounds)
        {
            worldBounds = bounds;
            SubdivideAndBuildConstrainedTriangulation(obstacles);

            allTriangles = delaunator.GetTriangles();
            allEdges = delaunator.GetEdges();

            allTriangleBounds = new List<AABB>();

            for (int i = 0; i < allTriangles.Count; i++)
            {
                AABB aabb = new AABB
                {
                    minX = float.MaxValue,
                    maxX = float.MinValue,
                    minY = float.MaxValue,
                    maxY = float.MinValue,
                };
                List<int> trianglePoints = allTriangles[i].points;

                for (int j = 0; j < trianglePoints.Count; j++)
                {
                    Float2 point = allPoints[trianglePoints[j]];
                    aabb.minX = MathUtils.Min(aabb.minX, point.x);
                    aabb.maxX = MathUtils.Max(aabb.maxX, point.x);
                    aabb.minY = MathUtils.Min(aabb.minY, point.y);
                    aabb.maxY = MathUtils.Max(aabb.maxY, point.y);
                }

                allTriangleBounds.Add(aabb);
            }

            CalculateTriangleCentroids();
            CalculateEdgesAroundPointsMap();
            FindWalkableEdges(obstacles);
            FindWalkableTriangles(obstacles);

            ResolveObstacleHullEdges(obstacles);

            CalulateHullEdgeTriangulationEdgeToObstacleIndices();
            CalculateSizeOfSmallestHullEdge();
            CreateTriangulationSearch();
            CreateVisitedTriangles();
        }

        List<Float2> GetSubdividedWorldBoundEdges(List<Obstacle> obstacles)
        {
            List<Float2> worldBoundCorners = GetDefaultWorldBounds();
            List<Float2> subdividedWorldBoundCorners = new List<Float2>();

            for (int i = 0; i < worldBoundCorners.Count; i++)
            {
                int iNext = i + 1;
                if (iNext == worldBoundCorners.Count)
                {
                    iNext = 0;
                }

                Float2 p1 = worldBoundCorners[i];
                Float2 p2 = worldBoundCorners[iNext];
                float minDistanceSqr = float.MaxValue;

                for (int j = 0; j < obstacles.Count; j++)
                {
                    List<Float2> obstacleCorners = obstacles[j].obstacleCorners;
                    for (int k = 0; k < obstacleCorners.Count; k++)
                    {
                        if (worldBounds.IsInside(obstacleCorners[k]))
                        {
                            Float2 projectedPoint = MathUtils.FindNearestPointOnLineSegment(p1, p2, obstacleCorners[k]);
                            float distanceSqr = (obstacleCorners[k] - projectedPoint).LengthSquared();

                            if (distanceSqr > 0.0f && distanceSqr < minDistanceSqr)
                            {
                                minDistanceSqr = distanceSqr;
                            }
                        }
                    }
                }

                int nSubdivides = (int)(0.5f * (p2 - p1).Length() / MathUtils.Sqrt(minDistanceSqr));
                if (nSubdivides > 5)
                {
                    nSubdivides = 5;
                }

                Float2 normal = VectorUtils.PerpendicularCounterClockwise(p2 - p1).Normalized() * 0.001f;

                for (int j = 0; j < nSubdivides; j++)
                {
                    float f1 = 1f * (j + 1) / (nSubdivides + 1);
                    float f2 = 1f - f1;

                    Float2 subdividedCorner = p1 * f1 + p2 * f2;
                    bool isInsideObstacle = false;

                    for (int k = 0; k < obstacles.Count; k++)
                    {
                        Float2 subdividedCornerWithOffset = subdividedCorner + normal;

                        if (!isInsideObstacle && VectorUtils.IsPointInPolygon(subdividedCornerWithOffset, obstacles[k].obstacleCorners))
                        {
                            isInsideObstacle = true;
                        }
                    }

                    if (!isInsideObstacle)
                    {
                        subdividedWorldBoundCorners.Add(p1 * f1 + p2 * f2);
                    }
                }
            }

            for (int i = 0; i < subdividedWorldBoundCorners.Count; i++)
            {
                worldBoundCorners.Add(subdividedWorldBoundCorners[i]);
            }

            return worldBoundCorners;
        }

        void SubdivideAndBuildConstrainedTriangulation(List<Obstacle> obstacles)
        {
            List<Float2> worldBoundCorners = GetSubdividedWorldBoundEdges(obstacles);
            totalNumberOfWorldBoundCorners = worldBoundCorners.Count;

            allPoints.Clear();
            obstacleWalkablityIndices.Clear();
            isObstacleCornerIntersectingWithWorldBounds.Clear();

            for (int i = 0; i < worldBoundCorners.Count; i++)
            {
                allPoints.Add(worldBoundCorners[i]);
                obstacleWalkablityIndices.Add(-1);
                isObstacleCornerIntersectingWithWorldBounds.Add(false);
            }

            List<ConstraintEdge> constraintEdges = new List<ConstraintEdge>();
            if (obstacles.Count > 0)
            {
                AddObstaclesWithConstraints(
                    obstacles,
                    constraintEdges,
                    allPoints,
                    obstacleWalkablityIndices,
                    obstacleIntersections,
                    isObstacleCornerIntersectingWithWorldBounds);
            }

            delaunator.Create(allPoints);
            delaunator.ClearTemporaryLists();

            constrainautor.Create(delaunator, constraintEdges);
            constrainautor.ClearTemporaryLists();
        }

        void FindWalkableTriangles(List<Obstacle> obstacles)
        {
            unwalkableTriangles.Clear();
            walkableTriangles.Clear();
            unwalkableTrianglesObstacleIndices.Clear();
            allToUnwalkableTriangleIndices.Clear();

            for (int i = 0; i < allTriangles.Count; i++)
            {
                Triangle triangle = allTriangles[i];
                List<int> trianglePoints = triangle.points;

                bool isWalkable = trianglesWalkability[i] == -1;

                if (!isWalkable)
                {
                    allToUnwalkableTriangleIndices.Add(unwalkableTriangles.Count);
                    unwalkableTriangles.Add(i);
                    unwalkableTrianglesObstacleIndices.Add(trianglesWalkability[i]);
                }
                else
                {
                    allToUnwalkableTriangleIndices.Add(-1);
                    walkableTriangles.Add(i);
                }
            }
        }

        void FindWalkableEdges(List<Obstacle> obstacles)
        {
            edgesWalkability.Resize(delaunator.trianglesLen);

            for (int i = 0; i < edgesWalkability.Count; i++)
            {
                edgesWalkability[i] = true;
            }

            trianglesWalkability.Resize(allTriangles.Count);

            for (int i = 0; i < allTriangles.Count; i++)
            {
                trianglesWalkability[i] = -1;
            }

            float epsilon = 0.001f;
            List<int> neighbours = new List<int>();

            for (int i = 0; i < obstacles.Count; i++)
            {
                Float2 center = obstacles[i].center;
                float largestCornerDistance = obstacles[i].largestCornerDistance;

                neighbours.Clear();
                allTriangleCentroidsKdTree.FindNearestsBall(center, largestCornerDistance + 2.0f * epsilon, neighbours);

                for (int j = 0; j < neighbours.Count; j++)
                {
                    int triangleIndex = neighbours[j];
                    if (trianglesWalkability[triangleIndex] == -1 && VectorUtils.IsPointInPolygon(allTriangleCentroids[triangleIndex], obstacles[i].obstacleCorners))
                    {
                        trianglesWalkability[triangleIndex] = i;
                    }
                }
            }

            for (int i = 0; i < obstacles.Count; i++)
            {
                if (obstacles[i].isWalkable)
                {
                    Float2 center = obstacles[i].center;
                    float largestCornerDistance = obstacles[i].largestCornerDistance;

                    neighbours.Clear();
                    allTriangleCentroidsKdTree.FindNearestsBall(center, largestCornerDistance + 2.0f * epsilon, neighbours);

                    for (int j = 0; j < neighbours.Count; j++)
                    {
                        int triangleIndex = neighbours[j];
                        if (trianglesWalkability[triangleIndex] != -1 && VectorUtils.IsPointInPolygon(allTriangleCentroids[triangleIndex], obstacles[i].obstacleCorners))
                        {
                            trianglesWalkability[triangleIndex] = -1;
                        }
                    }
                }
            }

            for (int i = 0; i < allTriangles.Count; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    int e = 3 * i + j;
                    int opposite = delaunator.halfedges[e];

                    if (opposite >= 0)
                    {
                        int nextTriangle = Delaunator.TriangleOfEdge(opposite);
                        if ((trianglesWalkability[i] != -1 && trianglesWalkability[nextTriangle] != -1))
                        {
                            edgesWalkability[e] = false;
                            edgesWalkability[opposite] = false;
                        }
                    }
                }
            }

            for (int i = 0; i < allEdges.Count; i++)
            {
                int p = allEdges[i].p;
                int q = allEdges[i].q;

                Float2 midPoint = (allPoints[p] + allPoints[q]) * 0.5f;

                if (!worldBounds.IsInside(midPoint))
                {
                    if (isObstacleCornerIntersectingWithWorldBounds[p] && isObstacleCornerIntersectingWithWorldBounds[q])
                    {
                        int e = allEdges[i].index;
                        edgesWalkability[e] = false;
                    }
                    else if (
                        (isObstacleCornerIntersectingWithWorldBounds[p] && q < totalNumberOfWorldBoundCorners) ||
                        (isObstacleCornerIntersectingWithWorldBounds[q] && p < totalNumberOfWorldBoundCorners)
                    )
                    {
                        int e = allEdges[i].index;
                        edgesWalkability[e] = false;
                    }
                }
            }
        }

        void CalculateSizeOfSmallestHullEdge()
        {
            smallestHullEdgeSize = worldBounds.maxX - worldBounds.minX;

            for (int i = 0; i < obstacleHullEdges.Count; i++)
            {
                int edgeIndex = obstacleHullEdges[i];
                int p = allEdges[edgeIndex].p;
                int q = allEdges[edgeIndex].q;

                float edgeSize = (allPoints[p] - allPoints[q]).Length();

                smallestHullEdgeSize = MathUtils.Min(smallestHullEdgeSize, edgeSize);
            }
        }

        void CalculateTriangleCentroids()
        {
            allTriangleCentroids.Resize(allTriangles.Count);

            for (int i = 0; i < allTriangleCentroids.Count; i++)
            {
                List<int> trianglePoints = allTriangles[i].points;

                Float2 p1 = allPoints[trianglePoints[0]];
                Float2 p2 = allPoints[trianglePoints[1]];
                Float2 p3 = allPoints[trianglePoints[2]];

                allTriangleCentroids[i] = (p1 + p2 + p3) / 3.0f;
            }

            allTriangleCentroidsKdTree = KDTree2D.MakeFromPoints(allTriangleCentroids.ToArray());
        }

        void CreateTriangulationSearch()
        {
            List<bool> allTrianglesMask = new List<bool>();
            List<bool> walkableTrianglesMask = new List<bool>();
            List<bool> unwalkableTrianglesMask = new List<bool>();

            int trianglesCount = allTriangles.Count;

            allTrianglesMask.Resize(trianglesCount);
            walkableTrianglesMask.Resize(trianglesCount);
            unwalkableTrianglesMask.Resize(trianglesCount);

            for (int i = 0; i < trianglesCount; i++)
            {
                allTrianglesMask[i] = true;

                bool isCurrentTriangleWalkable = trianglesWalkability[i] == -1;
                walkableTrianglesMask[i] = isCurrentTriangleWalkable;
                unwalkableTrianglesMask[i] = !isCurrentTriangleWalkable;
            }

            int resolution = 40;

            AABB triangulationGridBounds = new AABB
            {
                minX = worldBounds.minX - 1.0f,
                maxX = worldBounds.maxX + 1.0f,
                minY = worldBounds.minY - 1.0f,
                maxY = worldBounds.maxY + 1.0f
            };

            allTriangulationGridSearch = new TriangulationGridSearch();
            allTriangulationGridSearch.Create(triangulationGridBounds, resolution, delaunator, allPoints, trianglesCount, allTrianglesMask);

            walkableTriangulationGridSearch = new TriangulationGridSearch();
            walkableTriangulationGridSearch.Create(triangulationGridBounds, resolution, delaunator, allPoints, trianglesCount, walkableTrianglesMask);

            unwalkableTriangulationGridSearch = new TriangulationGridSearch();
            unwalkableTriangulationGridSearch.Create(triangulationGridBounds, resolution, delaunator, allPoints, trianglesCount, unwalkableTrianglesMask);
        }

        void CalculateEdgesAroundPointsMap()
        {
            edgesAroundPointsMap.Clear();

            for (int i = 0; i < allPoints.Count; i++)
            {
                edgesAroundPointsMap.Add(new List<int>());
            }

            for (int i = 0; i < allEdges.Count; i++)
            {
                int p = allEdges[i].p;
                int q = allEdges[i].q;

                edgesAroundPointsMap[p].Add(i);
                edgesAroundPointsMap[q].Add(i);
            }
        }

        void CalulateHullEdgeTriangulationEdgeToObstacleIndices()
        {
            hullEdgeTriangulationEdgeToObstacleIndices.Clear();
            for (int i = 0; i < delaunator.trianglesLen; i++)
            {
                hullEdgeTriangulationEdgeToObstacleIndices.Add(-1);
            }

            for (int i = 0; i < obstacleHullEdges.Count; i++)
            {
                int edgeIndex = obstacleHullEdges[i];
                hullEdgeTriangulationEdgeToObstacleIndices[allEdges[edgeIndex].index] = obstacleHullEdgeObstacleIndices[i];
            }
        }

        void CreateVisitedTriangles()
        {
            visitedTriangles.Resize(allTriangles.Count);
            for (int i = 0; i < visitedTriangles.Count; i++)
            {
                visitedTriangles[i] = false;
            }
        }

        List<Float2> GetDefaultWorldBounds()
        {
            List<Float2> points = new List<Float2>();
            points.Add(new Float2(worldBounds.minX, worldBounds.minY));
            points.Add(new Float2(worldBounds.maxX, worldBounds.minY));
            points.Add(new Float2(worldBounds.maxX, worldBounds.maxY));
            points.Add(new Float2(worldBounds.minX, worldBounds.maxY));

            return points;
        }

        public void AddObstacles(List<Obstacle> obstacles)
        {
            for (int i = 0; i < obstacles.Count; i++)
            {
                AddObstacle(obstacles, i);
            }
        }

        public void AddObstaclesWithConstraints(
            List<Obstacle> obstacles,
            List<ConstraintEdge> constraintEdges,
            List<Float2> newPoints,
            List<int> newObstacleWalkablityIndices,
            List<List<int>> newObstacleIntersections,
            List<bool> newIsObstacleCornerIntersectingWithWorldBounds)
        {
            List<Float2> segmentStarts = new List<Float2>();
            List<Float2> segmentEnds = new List<Float2>();
            List<Float2> segmentCenters = new List<Float2>();
            List<float> segmentRadii = new List<float>();
            List<int> segmentObstacleIndices = new List<int>();
            List<int> segmentCornerStartInObstacleIndices = new List<int>();

            List<List<List<Float2>>> intersectionPointsByCorners = new List<List<List<Float2>>>();
            List<List<List<int>>> intersectionObstacleIndicesByCorners = new List<List<List<int>>>();
            List<List<List<int>>> intersectionCornerIndicesByCorners = new List<List<List<int>>>();
            List<List<List<int>>> newPointsIndicesByCorners = new List<List<List<int>>>();
            List<List<List<int>>> intersectionCornerIndicesByCornersNeighbours = new List<List<List<int>>>();

            float maxRadius = 0.0f;
            float epsilon = 0.001f;

            int obstaclesCount = obstacles.Count;

            newObstacleIntersections.Resize(obstaclesCount);

            intersectionPointsByCorners.Resize(obstaclesCount);
            intersectionObstacleIndicesByCorners.Resize(obstaclesCount);
            intersectionCornerIndicesByCorners.Resize(obstaclesCount);
            newPointsIndicesByCorners.Resize(obstaclesCount);
            intersectionCornerIndicesByCornersNeighbours.Resize(obstaclesCount);

            int totalCornersCount = 0;
            for (int i = 0; i < obstaclesCount; i++)
            {
                totalCornersCount += obstacles[i].obstacleCorners.Count;
            }

            segmentStarts.Resize(totalCornersCount);
            segmentEnds.Resize(totalCornersCount);
            segmentCenters.Resize(totalCornersCount);
            segmentRadii.Resize(totalCornersCount);
            segmentObstacleIndices.Resize(totalCornersCount);
            segmentCornerStartInObstacleIndices.Resize(totalCornersCount);

            int totalCornerIndex = 0;

            for (int i = 0; i < obstaclesCount; i++)
            {
                newObstacleIntersections[i] = new List<int>();

                intersectionPointsByCorners[i] = new List<List<Float2>>();
                intersectionObstacleIndicesByCorners[i] = new List<List<int>>();
                intersectionCornerIndicesByCorners[i] = new List<List<int>>();
                newPointsIndicesByCorners[i] = new List<List<int>>();
                intersectionCornerIndicesByCornersNeighbours[i] = new List<List<int>>();

                int obstacleCornersCount = obstacles[i].obstacleCorners.Count;

                intersectionPointsByCorners[i].Resize(obstacleCornersCount);
                intersectionObstacleIndicesByCorners[i].Resize(obstacleCornersCount);
                intersectionCornerIndicesByCorners[i].Resize(obstacleCornersCount);
                newPointsIndicesByCorners[i].Resize(obstacleCornersCount);
                intersectionCornerIndicesByCornersNeighbours[i].Resize(obstacleCornersCount);

                for (int j = 0; j < obstacleCornersCount; j++)
                {
                    intersectionPointsByCorners[i][j] = new List<Float2>();
                    intersectionObstacleIndicesByCorners[i][j] = new List<int>();
                    intersectionCornerIndicesByCorners[i][j] = new List<int>();
                    newPointsIndicesByCorners[i][j] = new List<int>();
                    intersectionCornerIndicesByCornersNeighbours[i][j] = new List<int>();

                    int nextCornerIndex = j + 1;
                    if (nextCornerIndex >= obstacleCornersCount)
                    {
                        nextCornerIndex = 0;
                    }

                    Float2 segmentStart = obstacles[i].obstacleCorners[j];
                    Float2 segmentEnd = obstacles[i].obstacleCorners[nextCornerIndex];
                    Float2 segmentCenter = (segmentStart + segmentEnd) * 0.5f;
                    float segmentRadius = (segmentCenter - segmentStart).Length();
                    maxRadius = MathUtils.Max(maxRadius, segmentRadius);

                    segmentStarts[totalCornerIndex] = segmentStart;
                    segmentEnds[totalCornerIndex] = segmentEnd;
                    segmentCenters[totalCornerIndex] = segmentCenter;
                    segmentRadii[totalCornerIndex] = segmentRadius;
                    segmentObstacleIndices[totalCornerIndex] = i;
                    segmentCornerStartInObstacleIndices[totalCornerIndex] = j;

                    totalCornerIndex++;
                }
            }

            KDTree2D segmentCentersKdTree = KDTree2D.MakeFromPoints(segmentCenters.ToArray());

            List<int> neighbours = new List<int>();

            for (int i = 0; i < segmentCenters.Count; i++)
            {
                Float2 startPointI = segmentStarts[i];
                Float2 endPointI = segmentEnds[i];

                float searchDistance = maxRadius + segmentRadii[i] + 6.0f * epsilon;

                Float2 queryPoint = segmentCenters[i];
                neighbours.Clear();
                segmentCentersKdTree.FindNearestsBall(queryPoint, searchDistance, neighbours);
                int neighboursCount = neighbours.Count;

                int iSegmentObstacleIndex = segmentObstacleIndices[i];

                if (neighboursCount > 0)
                {
                    HeapSort.Sort(neighbours, neighboursCount);

                    for (int j = 0; j < neighboursCount; j++)
                    {
                        int neighbour = neighbours[j];
                        int neighbourSegmentObstacleIndex = segmentObstacleIndices[neighbour];

                        if (neighbour > i && iSegmentObstacleIndex != neighbourSegmentObstacleIndex)
                        {
                            Float2 startPointNeighbour = segmentStarts[neighbour];
                            Float2 endPointNeighbour = segmentEnds[neighbour];

                            LineSegmentsIntersectionResult result = VectorUtils.LineSegmentsIntersection(startPointI, endPointI, startPointNeighbour, endPointNeighbour, epsilon);

                            if (result.intersects &&
                                !VectorUtils.PointOnLine2D(startPointNeighbour, startPointI, endPointI, epsilon) &&
                                !VectorUtils.PointOnLine2D(endPointNeighbour, startPointI, endPointI, epsilon) &&
                                !VectorUtils.PointOnLine2D(startPointI, startPointNeighbour, endPointNeighbour, epsilon) &&
                                !VectorUtils.PointOnLine2D(endPointI, startPointNeighbour, endPointNeighbour, epsilon))
                            {
                                int iSegmentCornerStartIndex = segmentCornerStartInObstacleIndices[i];
                                int neighbourSegmentCornerStartIndex = segmentCornerStartInObstacleIndices[neighbour];

                                intersectionPointsByCorners[iSegmentObstacleIndex][iSegmentCornerStartIndex].Add(result.intersection);
                                intersectionObstacleIndicesByCorners[iSegmentObstacleIndex][iSegmentCornerStartIndex].Add(-1);
                                intersectionCornerIndicesByCorners[iSegmentObstacleIndex][iSegmentCornerStartIndex].Add(-1);
                                newPointsIndicesByCorners[iSegmentObstacleIndex][iSegmentCornerStartIndex].Add(-1);
                                intersectionCornerIndicesByCornersNeighbours[iSegmentObstacleIndex][iSegmentCornerStartIndex].Add(-1);

                                intersectionPointsByCorners[neighbourSegmentObstacleIndex][neighbourSegmentCornerStartIndex].Add(result.intersection);
                                intersectionObstacleIndicesByCorners[neighbourSegmentObstacleIndex][neighbourSegmentCornerStartIndex].Add(iSegmentObstacleIndex);
                                intersectionCornerIndicesByCorners[neighbourSegmentObstacleIndex][neighbourSegmentCornerStartIndex].Add(iSegmentCornerStartIndex);
                                newPointsIndicesByCorners[neighbourSegmentObstacleIndex][neighbourSegmentCornerStartIndex].Add(-1);

                                intersectionCornerIndicesByCornersNeighbours[neighbourSegmentObstacleIndex][neighbourSegmentCornerStartIndex].Add(
                                    intersectionPointsByCorners[iSegmentObstacleIndex][iSegmentCornerStartIndex].Count - 1
                                );

                                newObstacleIntersections[iSegmentObstacleIndex].Add(neighbourSegmentObstacleIndex);
                                newObstacleIntersections[neighbourSegmentObstacleIndex].Add(iSegmentObstacleIndex);
                            }
                        }
                    }
                }
            }

            List<int> segmentsBegin = new List<int>();
            List<int> segmentsEnd = new List<int>();

            List<int> sortedIndices = new List<int>();
            List<float> sqrDistancesFromInitialPoint = new List<float>();

            for (int i = 0; i < obstacles.Count; i++)
            {
                int newPointsCountStart = newPoints.Count;
                int obstacleICornersSize = obstacles[i].obstacleCorners.Count;

                int currentSegmentsEndCount = 0;

                for (int j = 0; j < obstacles[i].obstacleCorners.Count; j++)
                {
                    int jNext = j + 1;
                    if (jNext >= obstacles[i].obstacleCorners.Count)
                    {
                        jNext = 0;
                    }

                    newPoints.Add(obstacles[i].obstacleCorners[j]);
                    newObstacleWalkablityIndices.Add(i);
                    newIsObstacleCornerIntersectingWithWorldBounds.Add(obstacles[i].isCornerIntersectingWithWorldBounds[j]);

                    int currentNewPointsCount = newPoints.Count;
                    segmentsBegin.Add(currentNewPointsCount - 1);
                    segmentsEnd.Add(currentNewPointsCount);

                    int intersectionPointsByCornersIJCount = intersectionPointsByCorners[i][j].Count;
                    if (intersectionPointsByCornersIJCount > 0)
                    {
                        sortedIndices.Resize(intersectionPointsByCornersIJCount);
                        sqrDistancesFromInitialPoint.Resize(intersectionPointsByCornersIJCount);

                        for (int k = 0; k < intersectionPointsByCornersIJCount; k++)
                        {
                            sortedIndices[k] = k;
                            sqrDistancesFromInitialPoint[k] = (obstacles[i].obstacleCorners[j] - intersectionPointsByCorners[i][j][k]).LengthSquared();
                        }

                        HeapSort.Sort(sortedIndices, sqrDistancesFromInitialPoint, intersectionPointsByCornersIJCount);

                        for (int k = 0; k < intersectionPointsByCornersIJCount; k++)
                        {
                            int kSorted = sortedIndices[k];

                            if (intersectionObstacleIndicesByCorners[i][j][kSorted] == -1)
                            {
                                newPoints.Add(intersectionPointsByCorners[i][j][kSorted]);
                                newObstacleWalkablityIndices.Add(i);
                                newIsObstacleCornerIntersectingWithWorldBounds.Add(obstacles[i].isCornerIntersectingWithWorldBounds[j] && obstacles[i].isCornerIntersectingWithWorldBounds[jNext]);

                                currentNewPointsCount = newPoints.Count;
                                segmentsBegin.Add(currentNewPointsCount - 1);
                                segmentsEnd.Add(currentNewPointsCount);
                                newPointsIndicesByCorners[i][j][kSorted] = currentNewPointsCount - 1;
                            }
                            else
                            {
                                int iNewPoint = intersectionObstacleIndicesByCorners[i][j][kSorted];
                                int jNewPoint = intersectionCornerIndicesByCorners[i][j][kSorted];
                                int kNewPoint = intersectionCornerIndicesByCornersNeighbours[i][j][kSorted];

                                int newPointsBeginIndex = newPointsIndicesByCorners[iNewPoint][jNewPoint][kNewPoint];

                                currentSegmentsEndCount = segmentsEnd.Count;
                                segmentsEnd[currentSegmentsEndCount - 1] = newPointsBeginIndex;
                                segmentsBegin.Add(newPointsBeginIndex);

                                currentNewPointsCount = newPoints.Count;
                                segmentsEnd.Add(currentNewPointsCount);
                            }
                        }
                    }
                }

                currentSegmentsEndCount = segmentsEnd.Count;
                segmentsEnd[currentSegmentsEndCount - 1] = newPointsCountStart;
            }

            int segmentsBeginCount = segmentsBegin.Count;
            constraintEdges.Resize(segmentsBeginCount);

            for (int i = 0; i < segmentsBegin.Count; i++)
            {
                constraintEdges[i] = new ConstraintEdge
                {
                    p = segmentsBegin[i],
                    q = segmentsEnd[i]
                };
            }

            // Currently causes more problems than solving
            // SplitCollinearConstrainedEdges(constraintEdges, newPoints, epsilon);
        }

        public void SplitCollinearConstrainedEdges(List<ConstraintEdge> constraintEdges, List<Float2> newPoints, float epsilon)
        {
            bool anyEdgesAdded = true;
            int anyEdgesAddedCount = 0;
            int maxAnyEdgesAddedCount = constraintEdges.Count;
            int newPointsCount = newPoints.Count;
            List<List<int>> edgeConnections = new List<List<int>>();
            edgeConnections.Resize(newPointsCount);

            for (int i = 0; i < newPointsCount; i++)
            {
                edgeConnections[i] = new List<int>();
            }

            while (anyEdgesAdded && anyEdgesAddedCount < maxAnyEdgesAddedCount)
            {
                int constraintEdgesCount = constraintEdges.Count;
                anyEdgesAdded = false;
                anyEdgesAddedCount++;
                List<Float2> constrainedEdgeCenters = new List<Float2>();
                List<float> constrainedEdgeRadii = new List<float>();
                List<bool> affectedSegments = new List<bool>();
                constrainedEdgeCenters.Resize(constraintEdgesCount);
                constrainedEdgeRadii.Resize(constraintEdgesCount);
                affectedSegments.Resize(constraintEdgesCount);

                float constrainedEdgeMaxRadius = 0.0f;

                for (int i = 0; i < newPointsCount; i++)
                {
                    edgeConnections[i].Clear();
                }

                for (int i = 0; i < constraintEdgesCount; i++)
                {
                    int p = constraintEdges[i].p;
                    int q = constraintEdges[i].q;
                    Float2 center = (newPoints[p] + newPoints[q]) * 0.5f;
                    float radius = (center - newPoints[p]).Length();

                    constrainedEdgeCenters[i] = center;
                    constrainedEdgeRadii[i] = radius;
                    affectedSegments[i] = false;

                    constrainedEdgeMaxRadius = MathUtils.Max(constrainedEdgeMaxRadius, radius);

                    edgeConnections[p].Add(q);
                    edgeConnections[q].Add(p);
                }

                KDTree2D constrainedEdgeCentersKdTree = KDTree2D.MakeFromPoints(constrainedEdgeCenters.ToArray());

                List<int> neighbours = new List<int>();

                for (int i = 0; i < constraintEdgesCount; i++)
                {
                    if (!affectedSegments[i])
                    {
                        float searchDistance = constrainedEdgeMaxRadius + constrainedEdgeRadii[i] + 6.0f * epsilon;

                        neighbours.Clear();
                        constrainedEdgeCentersKdTree.FindNearestsBall(constrainedEdgeCenters[i], searchDistance, neighbours);
                        int neighboursCount = neighbours.Count;

                        for (int j = 0; j < neighboursCount; j++)
                        {
                            int neighbour = neighbours[j];
                            if (neighbour != i && !affectedSegments[neighbour])
                            {
                                int ip = constraintEdges[i].p;
                                int iq = constraintEdges[i].q;
                                int neighbourp = constraintEdges[neighbour].p;
                                int neighbourq = constraintEdges[neighbour].q;

                                Float2 ipPoint = newPoints[ip];
                                Float2 iqPoint = newPoints[iq];
                                Float2 neighbourpPoint = newPoints[neighbourp];
                                Float2 neighbourqPoint = newPoints[neighbourq];

                                if (VectorUtils.AreLineSegmentsCollinearAndOverlapping(ipPoint, iqPoint, neighbourpPoint, neighbourqPoint, epsilon))
                                {
                                    List<int> selectedPointIndices = new List<int>();
                                    List<Float2> selectedPoints = new List<Float2>();
                                    selectedPointIndices.Resize(4);
                                    selectedPoints.Resize(4);

                                    selectedPointIndices[0] = ip;
                                    selectedPointIndices[1] = iq;
                                    selectedPointIndices[2] = neighbourp;
                                    selectedPointIndices[3] = neighbourq;

                                    selectedPoints[0] = ipPoint;
                                    selectedPoints[1] = iqPoint;
                                    selectedPoints[2] = neighbourpPoint;
                                    selectedPoints[3] = neighbourqPoint;

                                    int masterStartPoint = 0;
                                    float masterEndStartDistanceSqr = 0.0f;

                                    for (int k = 1; k < 4; k++)
                                    {
                                        float distSqr = (selectedPoints[0] - selectedPoints[k]).LengthSquared();
                                        if (distSqr > masterEndStartDistanceSqr)
                                        {
                                            masterEndStartDistanceSqr = distSqr;
                                            masterStartPoint = k;
                                        }
                                    }

                                    List<float> distanceSqrFromMasterPoint = new List<float>();
                                    List<int> sortedIndicesFromMasterPoint = new List<int>();
                                    distanceSqrFromMasterPoint.Resize(4);
                                    sortedIndicesFromMasterPoint.Resize(4);

                                    for (int k = 0; k < 4; k++)
                                    {
                                        distanceSqrFromMasterPoint[k] = (selectedPoints[masterStartPoint] - selectedPoints[k]).LengthSquared();
                                        sortedIndicesFromMasterPoint[k] = k;
                                    }

                                    HeapSort.Sort(sortedIndicesFromMasterPoint, distanceSqrFromMasterPoint, 4);

                                    bool shouldSplit = true;

                                    for (int k = 1; k < 4; k++)
                                    {
                                        int kSortedPrevious = sortedIndicesFromMasterPoint[k - 1];
                                        int kSortedCurrent = sortedIndicesFromMasterPoint[k];

                                        int selectedPointIndexPrevious = selectedPointIndices[kSortedPrevious];
                                        int selectedPointIndexCurrent = selectedPointIndices[kSortedCurrent];

                                        if (selectedPointIndexPrevious == selectedPointIndexCurrent)
                                        {
                                            shouldSplit = false;
                                        }
                                        if (shouldSplit && edgeConnections[selectedPointIndexPrevious].Contains(selectedPointIndexCurrent))
                                        {
                                            shouldSplit = false;
                                        }
                                    }

                                    if (shouldSplit)
                                    {
                                        for (int k = 1; k < 4; k++)
                                        {
                                            int kSortedPrevious = sortedIndicesFromMasterPoint[k - 1];
                                            int kSortedCurrent = sortedIndicesFromMasterPoint[k];

                                            int selectedPointIndexPrevious = selectedPointIndices[kSortedPrevious];
                                            int selectedPointIndexCurrent = selectedPointIndices[kSortedCurrent];

                                            constraintEdges.Add(new ConstraintEdge
                                            {
                                                p = selectedPointIndexPrevious,
                                                q = selectedPointIndexCurrent
                                            });

                                            edgeConnections[selectedPointIndexPrevious].Add(selectedPointIndexCurrent);
                                            edgeConnections[selectedPointIndexCurrent].Add(selectedPointIndexPrevious);

                                            affectedSegments.Add(false);

                                            affectedSegments[i] = true;
                                            affectedSegments[neighbour] = true;
                                            anyEdgesAdded = true;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                if (anyEdgesAdded)
                {
                    int iNew = 0;
                    for (int i = 0; i < constraintEdges.Count; i++)
                    {
                        if (!affectedSegments[i])
                        {
                            constraintEdges[iNew] = constraintEdges[i];
                            iNew++;
                        }
                    }

                    constraintEdges.Resize(iNew);
                }
            }

            DuplicateLineSegmentsTest(constraintEdges);
            int duplicateCount = DuplicateUtils.FindDuplicatesCountKdTree(newPoints, epsilon);
            if (duplicateCount > 0)
            {
                UnityEngine.Debug.Log($"duplicateCount: {duplicateCount}");
            }
        }

        void DuplicateLineSegmentsTest(List<ConstraintEdge> constraintEdges)
        {
            for (int i = 0; i < constraintEdges.Count; i++)
            {
                int p = constraintEdges[i].p;
                int q = constraintEdges[i].q;
                for (int j = 0; j < constraintEdges.Count; j++)
                {
                    if (i != j)
                    {
                        if ((p == constraintEdges[j].p && q == constraintEdges[j].q) || (p == constraintEdges[j].q && q == constraintEdges[j].p))
                        {
                            UnityEngine.Debug.Log($"DuplicateLineSegmentsTest: {i} {j}");
                        }
                    }
                }
            }
        }

        void AddObstacle(List<Obstacle> obstacles, int obstacleIndex)
        {
            Obstacle obstacle = obstacles[obstacleIndex];
            List<Float2> obstacleCorners = obstacle.obstacleCorners;

            obstacle.pointsIndexStart = allPoints.Count;
            obstacle.pointsCount = 0;

            for (int i = 0; i < obstacleCorners.Count; i++)
            {
                allPoints.Add(obstacleCorners[i]);
                obstacleWalkablityIndices.Add(obstacleIndex);
                isObstacleCornerIntersectingWithWorldBounds.Add(obstacle.isCornerIntersectingWithWorldBounds[i]);
                obstacle.pointsCount++;

                int nextCornerIndex = i + 1;
                if (nextCornerIndex >= obstacleCorners.Count)
                {
                    nextCornerIndex = 0;
                }

                int nSplits = obstacle.nSplits[i];

                for (int j = 0; j < nSplits; j++)
                {
                    float p1 = 1.0f * (j + 1) / (nSplits + 1);
                    float p2 = 1.0f - p1;

                    allPoints.Add(obstacleCorners[i] * p2 + obstacleCorners[nextCornerIndex] * p1);
                    obstacleWalkablityIndices.Add(obstacleIndex);
                    isObstacleCornerIntersectingWithWorldBounds.Add(
                        obstacle.isCornerIntersectingWithWorldBounds[i] &&
                        obstacle.isCornerIntersectingWithWorldBounds[nextCornerIndex]);
                    obstacle.pointsCount++;
                }
            }

            obstacles[obstacleIndex] = obstacle;
        }

        void ResolveObstacleHullEdges(List<Obstacle> obstacles)
        {
            obstacleHullEdges.Clear();
            obstacleHullEdgesByObstacles.Resize(obstacles.Count);
            obstacleHullEdgeObstacleIndices.Clear();

            for (int i = 0; i < obstacles.Count; i++)
            {
                obstacleHullEdgesByObstacles[i] = new List<int>();
            }

            for (int i = 0; i < allEdges.Count; i++)
            {
                int e = allEdges[i].index;
                int opposite = delaunator.halfedges[e];
                if (opposite >= 0)
                {
                    int triangle = Delaunator.TriangleOfEdge(e);
                    int nextTriangle = Delaunator.TriangleOfEdge(opposite);

                    if (trianglesWalkability[triangle] != -1 && trianglesWalkability[nextTriangle] == -1)
                    {
                        int obstacleIndex = trianglesWalkability[triangle];

                        obstacleHullEdges.Add(i);
                        obstacleHullEdgesByObstacles[obstacleIndex].Add(i);
                        obstacleHullEdgeObstacleIndices.Add(obstacleIndex);
                    }
                    else if (trianglesWalkability[triangle] == -1 && trianglesWalkability[nextTriangle] != -1)
                    {
                        int obstacleIndex = trianglesWalkability[nextTriangle];

                        obstacleHullEdges.Add(i);
                        obstacleHullEdgesByObstacles[obstacleIndex].Add(i);
                        obstacleHullEdgeObstacleIndices.Add(obstacleIndex);
                    }
                }
            }
        }

        public int FindTriangleForPoint(Float2 point)
        {
            return allTriangulationGridSearch.FindTriangleForPoint(point, delaunator, allPoints);
        }

        public int FindWalkableTriangleForPoint(Float2 point)
        {
            return walkableTriangulationGridSearch.FindTriangleForPoint(point, delaunator, allPoints);
        }

        public int FindUnwalkableTriangleForPoint(Float2 point)
        {
            return unwalkableTriangulationGridSearch.FindTriangleForPoint(point, delaunator, allPoints);
        }

        public GetNearestWalkablePositionResult TryMoveToWalkableArea(Float2 position)
        {
            float epsilon = 0.001f;
            bool wasAdjusted = false;
            position = VectorUtils.AdjustForBoundaries(position, worldBounds.minX, worldBounds.maxX, worldBounds.minY, worldBounds.maxY, epsilon, ref wasAdjusted);

            GetNearestWalkablePositionResult nearestWalkablePositionResult = GetNearestWalkablePosition(position, epsilon);
            if (nearestWalkablePositionResult.wasMoved)
            {
                Float2 randomInsideUnitCircle = VectorUtils.RangomInsideUnitCircle(random);
                nearestWalkablePositionResult.position += randomInsideUnitCircle * epsilon * 0.5f;
            }

            if (wasAdjusted)
            {
                nearestWalkablePositionResult.wasMoved = true;
            }

            return nearestWalkablePositionResult;
        }

        public GetNearestWalkablePositionResult GetNearestWalkablePosition(Float2 position, float epsilon)
        {
            int triangle = unwalkableTriangulationGridSearch.FindTriangleForPoint(position, delaunator, allPoints);

            if (triangle != -1)
            {
                int unwalkableTriangleIndex = allToUnwalkableTriangleIndices[triangle];
                if (unwalkableTriangleIndex != -1)
                {
                    int obstacleIndex = unwalkableTrianglesObstacleIndices[unwalkableTriangleIndex];

                    int nearestHullEdge = -1;
                    float nearestHullEdgeDistanceSqr = float.MaxValue;
                    Float2 nearestHullEdgePoint = position;

                    if (obstacleIndex != -1)
                    {
                        List<int> obstacleIndicesToConsider = new List<int>();
                        int obstacleIndicesToConsiderCount = obstacleIntersections[obstacleIndex].Count + 1;
                        obstacleIndicesToConsider.Resize(obstacleIndicesToConsiderCount);
                        obstacleIndicesToConsider[0] = obstacleIndex;

                        for (int i = 0; i < obstacleIntersections[obstacleIndex].Count; i++)
                        {
                            obstacleIndicesToConsider[i + 1] = obstacleIntersections[obstacleIndex][i];
                        }

                        for (int i = 0; i < obstacleIndicesToConsiderCount; i++)
                        {
                            int nextObstacleIndex = obstacleIndicesToConsider[i];

                            for (int j = 0; j < obstacleHullEdgesByObstacles[nextObstacleIndex].Count; j++)
                            {
                                int edgeIndex = obstacleHullEdgesByObstacles[nextObstacleIndex][j];

                                if (edgesWalkability[allEdges[edgeIndex].index])
                                {
                                    int p = allEdges[edgeIndex].p;
                                    int q = allEdges[edgeIndex].q;

                                    Float2 nearestEdgePoint = MathUtils.FindNearestPointOnLineSegment(allPoints[p], allPoints[q], position);
                                    float sqrDistance = (nearestEdgePoint - position).LengthSquared();

                                    if (sqrDistance < nearestHullEdgeDistanceSqr)
                                    {
                                        nearestHullEdge = edgeIndex;
                                        nearestHullEdgeDistanceSqr = sqrDistance;
                                        nearestHullEdgePoint = nearestEdgePoint;
                                    }
                                }
                            }
                        }
                    }

                    if (nearestHullEdge != -1)
                    {
                        Float2 relativeDirection = nearestHullEdgePoint - position;
                        if (relativeDirection.LengthSquared() > 0.0f)
                        {
                            nearestHullEdgePoint += relativeDirection.Normalized() * epsilon;
                        }
                        else
                        {
                            Float2 centroid = allTriangleCentroids[triangle];
                            Float2 centroidDirection = (nearestHullEdgePoint - centroid).Normalized();

                            int p = allEdges[nearestHullEdge].p;
                            int q = allEdges[nearestHullEdge].q;

                            Float2 diff_p_q = allPoints[p] - allPoints[q];

                            Float2 direction = VectorUtils.PerpendicularCounterClockwise(diff_p_q).Normalized();
                            if (Float2.Dot(direction, centroidDirection) < 0.0f)
                            {
                                direction = -direction;
                            }

                            nearestHullEdgePoint += direction * epsilon;
                        }
                    }

                    return new GetNearestWalkablePositionResult
                    {
                        wasMoved = nearestHullEdge != -1,
                        position = nearestHullEdgePoint
                    };
                }
            }

            return new GetNearestWalkablePositionResult
            {
                wasMoved = false,
                position = position
            };
        }

        public Float2 FindNearestObstacleHullEdgePointToTarget(int lowestHCostNode, Float2 lowestHCostNodePosition, Float2 target)
        {
            List<int> edgesAroundPoint = edgesAroundPointsMap[lowestHCostNode];

            float smallestFinalDistanceSqr = (target - lowestHCostNodePosition).LengthSquared();
            Float2 finalPosition = lowestHCostNodePosition;

            for (int i = 0; i < edgesAroundPoint.Count; i++)
            {
                int edgeIndex = edgesAroundPoint[i];
                int hafEdgeIndex = allEdges[edgeIndex].index;

                if (hullEdgeTriangulationEdgeToObstacleIndices[hafEdgeIndex] != -1 && edgesWalkability[hafEdgeIndex])
                {
                    int p = allEdges[edgeIndex].p;
                    int q = allEdges[edgeIndex].q;
                    Float2 nearestEdgePoint = MathUtils.FindNearestPointOnLineSegment(allPoints[p], allPoints[q], target);

                    float smallestDistanceSqr = (target - nearestEdgePoint).LengthSquared();

                    if (smallestDistanceSqr < smallestFinalDistanceSqr)
                    {
                        smallestFinalDistanceSqr = smallestDistanceSqr;
                        finalPosition = nearestEdgePoint;
                    }
                }
            }

            return finalPosition;
        }

        public bool CanPointsBeReachedInStraightLine(Float2 a, Float2 b)
        {
            Float2 direction = (b - a).Normalized() * 0.001f;
            Float2 a1 = a + direction;

            direction = (a - b).Normalized() * 0.001f;
            Float2 b1 = b + direction;

            List<int> currentlyVisitedTriangles = new List<int>();
            int triangleToVisit = FindTriangleForPoint(a1);

            Float2 lastIntersection = a1;

            while (triangleToVisit != -1)
            {
                int triangle = triangleToVisit;
                visitedTriangles[triangle] = true;
                currentlyVisitedTriangles.Add(triangle);
                triangleToVisit = -1;

                if (trianglesWalkability[triangle] != -1)
                {
                    for (int i = 0; i < currentlyVisitedTriangles.Count; i++)
                    {
                        visitedTriangles[currentlyVisitedTriangles[i]] = false;
                    }
                    return false;
                }

                float shortestDistanceSqr = (lastIntersection - b1).LengthSquared();

                for (int i = 0; i < 3; i++)
                {
                    int e = 3 * triangle + i;
                    int opposite = delaunator.halfedges[e];

                    if (opposite >= 0)
                    {
                        int nextTriangle = Delaunator.TriangleOfEdge(opposite);
                        if (!visitedTriangles[nextTriangle])
                        {
                            int p = delaunator.triangles[e];
                            int q = delaunator.triangles[Delaunator.NextHalfedge(e)];

                            LineSegmentsIntersectionResult intersectionResult = VectorUtils.LineSegmentsIntersection(
                                b1,
                                a1,
                                allPoints[p],
                                allPoints[q]);

                            if (intersectionResult.intersects)
                            {
                                float distanceSqr = (intersectionResult.intersection - b1).LengthSquared();

                                if (distanceSqr < shortestDistanceSqr)
                                {
                                    lastIntersection = intersectionResult.intersection;
                                    triangleToVisit = nextTriangle;
                                    shortestDistanceSqr = distanceSqr;
                                }
                            }
                        }
                    }
                }
            }

            for (int i = 0; i < currentlyVisitedTriangles.Count; i++)
            {
                visitedTriangles[currentlyVisitedTriangles[i]] = false;
            }
            return true;
        }
    }
}
