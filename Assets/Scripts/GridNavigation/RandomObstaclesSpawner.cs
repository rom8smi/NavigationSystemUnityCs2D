using System.Collections.Generic;
using UnityEngine;
using GenericCode;

namespace GridNavigation
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

        public List<Obstacle> CreateObstacles()
        {
            ManualRandom random = new ManualRandom((ulong)seed);
            List<Obstacle> obstacles = new List<Obstacle>();

            for (int i = 0; i < numberToSpawn; i++)
            {
                float x = random.next_float(minX, maxX);
                float y = random.next_float(minY, maxY);

                float dx = random.next_float(minSizeX, maxSizeX);
                float dy = random.next_float(minSizeY, maxSizeY);

                obstacles.Add(
                    new Obstacle
                    {
                        center = new Float2(x, y),
                        size = new Float2(dx, dy)
                    }
                );

                GameObject obstacleInstance = Instantiate(obstaclePrefab, new Vector3(x, 0.0f, y), Quaternion.identity);
                obstacleInstance.transform.localScale = new Vector3(dx, 1.0f, dy);
            }

            return obstacles;
        }
    }
}
