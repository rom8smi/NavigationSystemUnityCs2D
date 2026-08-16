using System.Collections.Generic;
using System.Diagnostics;
using DelaunatorSharp;
using UnityEngine;
using GenericCode;

namespace TriangulationNavigation
{
    public class PortingTestsScene : MonoBehaviour
    {
        void Start()
        {
            // DelaunatorTest();
            NavMeshTest();
        }

        void DelaunatorTest()
        {
            var rand = new ManualRandom(1);
            List<Float2> points = new List<Float2>();

            for (int i = 0; i < 10000; i++)
            {
                points.Add(new Float2(rand.next_float(0.0f, 1.0f), rand.next_float(0.0f, 1.0f)));
            }

            Delaunator delaunator = new Delaunator();

            var sw = new Stopwatch();
            sw.Start();
            delaunator.Create(points);

            GenericCode.Debug.Log("Number of points: " + delaunator.GetTriangles().Count + ", Calculation time: " + sw.Elapsed.TotalMilliseconds + " ms");
            DelaunatorDebugger.DebugDelaunatorStatistics(delaunator, points);
        }

        void NavMeshTest()
        {
            NavMesh nav_mesh = new NavMesh();
            float worldSize = 100.0f;

            List<RectangularObstacle> rectangularObstacles = new List<RectangularObstacle>
            {
                new RectangularObstacle
                {
                    center = new Float2(25.9f, 26.8f),
                    size = new Float2(16.7f, 13.3f)
                },
                new RectangularObstacle
                {
                    center = new Float2(1.4f, 0.1000004f),
                    size = new Float2(23.3f, 6.7f)
                },
                new RectangularObstacle
                {
                    center = new Float2(-11.8f, -20.0f),
                    size = new Float2(10.0f, 23.3f)
                },
                new RectangularObstacle
                {
                    center = new Float2(28.3f, -19.2f),
                    size = new Float2(10.0f, 10.0f)
                },
                new RectangularObstacle
                {
                    center = new Float2(1.199999f, 24.1f),
                    size = new Float2(10.0f, 23.3f)
                },
                new RectangularObstacle
                {
                    center = new Float2(-25.4f, 23.1f),
                    size = new Float2(10.0f, 10.0f)
                }
            };

            AABB bounds = new AABB
            {
                minX = -0.5f * worldSize,
                maxX = 0.5f * worldSize,
                minY = -0.5f * worldSize,
                maxY = 0.5f * worldSize,
            };

            List<Obstacle> obstacles = new List<Obstacle>();
            
            for(int i=0; i<rectangularObstacles.Count; i++)
            {
                List<Float2> corners = rectangularObstacles[i].GetCorners();
                Obstacle obstacle = ObstacleUtils.Create(corners, bounds, 2.0f * worldSize, false);
                obstacles.Add(obstacle);
            }

		    nav_mesh.Create(obstacles, worldSize);

            Pathfinding pathfinding = new Pathfinding();
            pathfinding.CreateNodes(nav_mesh);

            Float2 startPosition = new Float2(37.7f, 42.2f);
            Float2 targetPosition = new Float2(-34.0f, -44.2f);

            var sw = new Stopwatch();
            sw.Start();
            Path path = pathfinding.FindPath(startPosition, targetPosition, nav_mesh);

            GenericCode.Debug.Log("Number of points: " + nav_mesh.allPoints.Count + " " + path.waypoints.Count + " " + sw.Elapsed.TotalMilliseconds);
        }
    }
}
