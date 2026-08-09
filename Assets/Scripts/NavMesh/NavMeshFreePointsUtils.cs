using System.Collections.Generic;
using GenericCode;
using DelaunatorSharp;

namespace TriangulationNavigation
{
    public static class NavMeshFreePointsUtils
    {
        public static List<Float2> CalculateRandomPoints(List<Obstacle> obstacles, int n, int seed, AABB worldBounds)
        {
            List<Float2> points = new List<Float2>();

            ManualRandom rand = new ManualRandom((ulong)seed);
            float epsilon = 0.01f;

            for (int i = 0; i < n; i++)
            {
                float x = rand.next_float(worldBounds.minX + epsilon, worldBounds.maxX - epsilon);
                float y = rand.next_float(worldBounds.minY + epsilon, worldBounds.maxY - epsilon);
                Float2 point = new Float2(x, y);

                if (!IsInsideAnyObstacle(obstacles, point))
                {
                    points.Add(point);
                }
            }

            return points;
        }

        public static List<Float2> CalculateGridPoints(List<Obstacle> obstacles, int res, AABB worldBounds)
        {
            List<Float2> points = new List<Float2>();

            int resMod = res + 1;

            float gapX = (worldBounds.maxX - worldBounds.minX) / resMod;
            float gapY = (worldBounds.maxY - worldBounds.minY) / resMod;

            for (int i = 1; i < resMod; i++)
            {
                for (int j = 1; j < resMod; j++)
                {
                    Float2 point = new Float2(worldBounds.minX + gapX * i, worldBounds.minY + gapY * j);

                    if (!IsInsideAnyObstacle(obstacles, point))
                    {
                        points.Add(point);
                    }
                }
            }

            return points;
        }

        public static List<Float2> CalculateTriangleCentroidPoints(List<Obstacle> obstacles, NavMesh navMesh)
        {
            List<Float2> points = new List<Float2>();

            navMesh.AddObstacles(obstacles);
            navMesh.delaunator.Create(navMesh.allPoints);

            List<Triangle> triangles = navMesh.delaunator.GetTriangles();
            for (int i = 0; i < triangles.Count; i++)
            {
                List<int> trianglePoints = triangles[i].points;

                Float2 p1 = navMesh.allPoints[trianglePoints[0]];
                Float2 p2 = navMesh.allPoints[trianglePoints[1]];
                Float2 p3 = navMesh.allPoints[trianglePoints[2]];

                Float2 point = (p1 + p2 + p3) / 3.0f;

                if (!IsInsideAnyObstacle(obstacles, point))
                {
                    points.Add(point);
                }
            }

            return points;
        }

        public static List<Float2> CalculateTriangleEdgePoints(List<Obstacle> obstacles, NavMesh navMesh)
        {
            List<Float2> points = new List<Float2>();

            navMesh.AddObstacles(obstacles);
            navMesh.delaunator.Create(navMesh.allPoints);

            List<Edge> edges = navMesh.delaunator.GetEdges();
            for (int i = 0; i < edges.Count; i++)
            {
                int ip = edges[i].p;
                int iq = edges[i].q;

                Float2 p = navMesh.allPoints[ip];
                Float2 q = navMesh.allPoints[iq];

                Float2 point = (p + q) * 0.5f;

                if (navMesh.obstacleWalkablityIndices[ip] != navMesh.obstacleWalkablityIndices[iq] && !IsInsideAnyObstacle(obstacles, point))
                {
                    points.Add(point);
                }
            }

            return points;
        }

        public static void AddFreePoints(List<Float2> points, NavMesh navMesh)
        {
            for (int i = 0; i < points.Count; i++)
            {
                navMesh.allPoints.Add(points[i]);
                navMesh.obstacleWalkablityIndices.Add(-1);
                navMesh.isObstacleCornerIntersectingWithWorldBounds.Add(false);
            }
        }

        static bool IsInsideAnyObstacle(List<Obstacle> obstacles, Float2 point)
        {
            for (int i = 0; i < obstacles.Count; i++)
            {
                if (VectorUtils.IsPointInPolygon(point, obstacles[i].obstacleCorners))
                {
                    return true;
                }
            }
            return false;
        }
    }
}
