using System.Diagnostics;
using UnityEngine;
using GenericCode;

namespace GridNavigation
{
    public class PathTests : MonoBehaviour
    {
        public Float2 startPosition;
        public Float2 targetPosition;

        Grid navigationGrid;
        Pathfinding pathfinding;

        void Awake()
        {
            CreateNavigationGrid();
        }

        void Update()
        {
            if (Input.GetKeyDown(KeyCode.A))
            {
                CalculatePath();
            }
            if (Input.GetKeyDown(KeyCode.B))
            {
                Stopwatch sw = new Stopwatch();
                sw.Start();

                CreateNavigationGrid();

                UnityEngine.Debug.Log($"CreateNavigationGrid time {sw.Elapsed.TotalMilliseconds}");
            }
        }

        void CreateNavigationGrid()
        {
            Float2 gridWorldSize = new Float2(150.0f, 100.0f);
            float nodeRadius = 0.5f;
            int obstacleProximityPenalty = 10;

            navigationGrid = new Grid
            {
                obstacles = GetObstacles(),
                penaltyObstacles = GetPenaltyObstacles(),
                gridWorldSize = gridWorldSize,
                nodeRadius = nodeRadius,
                obstacleProximityPenalty = obstacleProximityPenalty,
                gridWorldOrigin = new Float2(transform.position.x - gridWorldSize.x * 0.5f, transform.position.z - gridWorldSize.y * 0.5f)
            };
            navigationGrid.Setup();
            pathfinding = new Pathfinding(navigationGrid);
            pathfinding.smoothPath = true;
            pathfinding.numberOfSmoothIterations = 10;
        }

        void CalculatePath()
        {
            Stopwatch sw = new Stopwatch();
            sw.Start();
            Path path = pathfinding.FindPath(new Float2(startPosition.x, startPosition.y), new Float2(targetPosition.x, targetPosition.y));

            double calculationTime = sw.Elapsed.TotalMilliseconds;
            float pathLength = 0.0f;

            for (int i = 1; i < path.waypoints.Count; i++)
            {
                pathLength += (path.waypoints[i] - path.waypoints[i - 1]).Length();
            }

            string result = $"CalculatePath time {calculationTime} ms | pathLength {pathLength} | numberOfPoints {path.waypoints.Count}\n";
            for (int i = 0; i < path.waypoints.Count; i++)
            {
                result += $"({path.waypoints[i].x}, {path.waypoints[i].y})\n";
            }

            UnityEngine.Debug.Log(result);
        }

        Obstacle[] GetObstacles()
        {
            return new Obstacle[]
            {
                new Obstacle
                {
                    center = new Float2(25.9f, 26.8f),
                    size = new Float2(16.7f, 13.3f)
                },
                new Obstacle
                {
                    center = new Float2(1.4f, 0.1000004f),
                    size = new Float2(23.3f, 6.7f)
                },
                new Obstacle
                {
                    center = new Float2(-11.8f, -20.0f),
                    size = new Float2(10.0f, 23.3f)
                },
                new Obstacle
                {
                    center = new Float2(28.3f, -19.2f),
                    size = new Float2(10.0f, 10.0f)
                },
                new Obstacle
                {
                    center = new Float2(1.199999f, 24.1f),
                    size = new Float2(10.0f, 23.3f)
                },
                new Obstacle
                {
                    center = new Float2(-25.4f, 23.1f),
                    size = new Float2(10.0f, 10.0f)
                }
            };
        }

        PenaltyObstacle[] GetPenaltyObstacles()
        {
            return new PenaltyObstacle[]
            {
                new PenaltyObstacle
                {
                    center = new Float2(5.1f, -14.9f),
                    size = new Float2(10.0f, 10.0f),
                    penalty = 5
                }
            };
        }
    }
}
