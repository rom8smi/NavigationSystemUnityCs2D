using System.Collections.Generic;
using UnityEngine;
using GenericCode;

namespace TriangulationNavigation
{
    public class ChainedObstaclesSpawner : MonoBehaviour
    {
        public GameObject obstaclePrefab;
        public bool instantiatePrefabs;

        public List<Obstacle> CreateObstacles(float worldSize)
        {
            List<Obstacle> obstacles = new List<Obstacle>();
            List<List<Float2>> allCorners = new List<List<Float2>>();

            float obstacleSize = 0.3f;
            float margin = 0.1f * obstacleSize;

            int numberToSpawn = 300;
            // int numberToSpawn = 2;

            for (int i = 0; i < numberToSpawn; i++)
            {
                float yOffset = -0.5f * worldSize + 1.1f * obstacleSize + 2.0f * margin;

                float x = 0.0f;
                float y = i * obstacleSize + yOffset;

                float dx = obstacleSize;
                float dy = obstacleSize;

                RectangularObstacle rectangularObstacle = new RectangularObstacle
                {
                    center = new Float2(x, y),
                    size = new Float2(dx, dy),
                    rotation = 0.0f,
                    radius = margin
                };

                List<Float2> corners = rectangularObstacle.GetCorners();
                allCorners.Add(corners);

                if (instantiatePrefabs)
                {
                    Quaternion quaternion = Quaternion.Euler(0.0f, 0.0f, 0.0f);
                    GameObject obstacleInstance = Instantiate(obstaclePrefab, new Vector3(x, 0.0f, y), quaternion);
                    obstacleInstance.transform.localScale = new Vector3(dx, 1.0f, dy);
                }
            }

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
