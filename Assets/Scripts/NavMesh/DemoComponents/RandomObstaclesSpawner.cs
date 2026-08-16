using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine;
using GenericCode;

namespace TriangulationNavigation
{
    public class RandomObstaclesSpawner : MonoBehaviour
    {
        public GameObject obstaclePrefab;
        public int numberToSpawn;

        public float minX;
        public float maxX;
        public float minY;
        public float maxY;

        public float minSizeX;
        public float maxSizeX;
        public float minSizeY;
        public float maxSizeY;

        public int seed;
        public bool randomizeRotation;

        public List<Obstacle> CreateObstacles(float worldSize, float largestAgentRadius)
        {
            ManualRandom random = new ManualRandom((ulong)seed);
            List<Obstacle> obstacles = new List<Obstacle>();
            List<List<Float2>> allCorners = new List<List<Float2>>();

            for (int i = 0; i < numberToSpawn; i++)
            {
                float x = random.next_float(minX, maxX);
                float y = random.next_float(minY, maxY);

                float dx = random.next_float(minSizeX, maxSizeX);
                float dy = random.next_float(minSizeY, maxSizeY);

                float rotation = 0.0f;
                if(randomizeRotation)
                {
                    rotation = random.next_float(0.0f, 2.0f * MathUtils.PI);
                }

                RectangularObstacle rectangularObstacle = new RectangularObstacle
                {
                    center = new Float2(x, y),
                    size = new Float2(dx, dy),
                    rotation = rotation,
                    radius = largestAgentRadius
                };

                List<Float2> corners = rectangularObstacle.GetCorners();
                allCorners.Add(corners);

                Quaternion quaternion = Quaternion.Euler(0.0f, -rotation * MathUtils.Rad2Deg(), 0.0f);
                GameObject obstacleInstance = Instantiate(obstaclePrefab, new Vector3(x, 0.0f, y), quaternion);
                obstacleInstance.transform.localScale = new Vector3(dx, 1.0f, dy);
            }

            // var sw = new Stopwatch();
            // sw.Start();
            // GenericCode.Debug.Log(allCorners.Count);

            // UnionPolygonFinder.FindUnionsForMultiplePolygons(allCorners);
            
            // GenericCode.Debug.Log(sw.Elapsed.TotalMilliseconds);

            AABB bounds = new AABB
            {
                minX = -0.5f * worldSize,
                maxX = 0.5f * worldSize,
                minY = -0.5f * worldSize,
                maxY = 0.5f * worldSize,
            };

            for (int i = 0; i < allCorners.Count; i++)
            {
                Obstacle obstacle = ObstacleUtils.Create(allCorners[i], bounds, 2.0f * worldSize, false);
                obstacles.Add(obstacle);
            }

            return obstacles;
        }
    }
}
