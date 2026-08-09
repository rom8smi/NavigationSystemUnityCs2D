using System.Collections.Generic;
using UnityEngine;
using GenericCode;

namespace TriangulationNavigation
{
    public class RandomDynamicObstaclesSpawner : MonoBehaviour
    {
        public GameObject obstaclePrefab;
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
        public float creationTimeInterval;
        public float remainingTimeMin;
        public float remainingTimeMax;

        float timePassed;
        List<DynamicObstacle> dynamicObstacles;
        List<GameObject> obstacleGameObjects;

        ObstaclesKdTree obstaclesKdTree;
        ManualRandom random = new ManualRandom(4);
        float fUpdates;

        void Start()
        {
            timePassed = 0.0f;
            dynamicObstacles = new List<DynamicObstacle>();
            obstacleGameObjects = new List<GameObject>();
            random = new ManualRandom(4);

            obstaclesKdTree = new ObstaclesKdTree();
            obstaclesKdTree.Build(NavigationSystemComponent.active.navigationSystem.obstacles);
        }

        void Update()
        {
            fUpdates += RuntimeConstants.updatesPerFrame;
            if (fUpdates > 1.0f)
            {
                int iUpdates = (int)fUpdates;

                for (int i = 0; i < iUpdates; i++)
                {
                    UpdateInner();
                }

                fUpdates -= iUpdates;
            }
        }

        void UpdateInner()
        {
            float deltaTime = RuntimeConstants.deltaTime;
            timePassed += deltaTime;

            if (timePassed > creationTimeInterval)
            {
                timePassed -= creationTimeInterval;
                CreateObstacle();
            }

            for (int i = 0; i < dynamicObstacles.Count; i++)
            {
                dynamicObstacles[i].remainingTime -= deltaTime;
                if (dynamicObstacles[i].remainingTime < 0.0f)
                {
                    RemoveObstacle(dynamicObstacles[i].obstacleIndex);
                    i--;
                }
            }
        }

        void CreateObstacle()
        {
            for (int i = 0; i < 10; i++)
            {
                if (TryCreateObstacle())
                {
                    return;
                }
            }
        }

        bool TryCreateObstacle()
        {
            float worldSize = NavigationSystemComponent.active.worldSize;
            List<Obstacle> obstacles = NavigationSystemComponent.active.navigationSystem.obstacles;

            float x = random.next_float(minX, maxX);
            float y = random.next_float(minY, maxY);

            float dx = random.next_float(minSizeX, maxSizeX);
            float dy = random.next_float(minSizeY, maxSizeY);

            float rotation = 0.0f;
            if (randomizeRotation)
            {
                rotation = random.next_float(0.0f, 2.0f * Mathf.PI);
            }

            RectangularObstacle rectangularObstacle = new RectangularObstacle
            {
                center = new Float2(x, y),
                size = new Float2(dx, dy),
                rotation = rotation,
                radius = NavigationSystemComponent.active.navigationSystem.largestAgentRadius
            };
            List<Float2> corners = rectangularObstacle.GetCorners();

            AABB bounds = new AABB
            {
                minX = -0.5f * worldSize,
                maxX = 0.5f * worldSize,
                minY = -0.5f * worldSize,
                maxY = 0.5f * worldSize,
            };

            Obstacle obstacle = ObstacleUtils.Create(corners, bounds, 5.0f * worldSize, false);

            if (bounds.IsInside(rectangularObstacle.center) && !obstaclesKdTree.Intersects(obstacle, obstacles))
            {
                Quaternion quaternion = Quaternion.Euler(0.0f, -rotation * Mathf.Rad2Deg, 0.0f);
                GameObject obstacleInstance = Instantiate(obstaclePrefab, new Vector3(x, 0.0f, y), quaternion);
                obstacleInstance.transform.localScale = new Vector3(dx, 1.0f, dy);

                obstacles.Add(obstacle);

                NavigationSystemComponent.active.navigationSystem.navMesh.Create(obstacles, worldSize);
                NavigationSystemComponent.active.navigationSystem.navMeshDrawer.CreateNavMeshSurfaceDrawer(NavigationSystemComponent.active.navigationSystem.navMesh);

                NavigationSystemComponent.active.navigationSystem.pathfinding.CreateNodes(NavigationSystemComponent.active.navigationSystem.navMesh);

                dynamicObstacles.Add(new DynamicObstacle
                {
                    obstacleIndex = obstacles.Count - 1,
                    remainingTime = random.next_float(remainingTimeMin, remainingTimeMax)
                });
                obstacleGameObjects.Add(obstacleInstance);

                obstaclesKdTree.Build(obstacles);
                return true;
            }

            return false;
        }

        void RemoveObstacle(int index)
        {
            NavigationSystemComponent.active.navigationSystem.obstacles.RemoveAt(index);
            List<Obstacle> obstacles = NavigationSystemComponent.active.navigationSystem.obstacles;
            dynamicObstacles.RemoveAt(index);
            Destroy(obstacleGameObjects[index]);
            obstacleGameObjects.RemoveAt(index);

            for (int i = index; i < dynamicObstacles.Count; i++)
            {
                dynamicObstacles[i].obstacleIndex--;
            }

            float worldSize = NavigationSystemComponent.active.worldSize;
            NavigationSystemComponent.active.navigationSystem.navMesh.Create(obstacles, worldSize);
            NavigationSystemComponent.active.navigationSystem.navMeshDrawer.CreateNavMeshSurfaceDrawer(NavigationSystemComponent.active.navigationSystem.navMesh);

            NavigationSystemComponent.active.navigationSystem.pathfinding.CreateNodes(NavigationSystemComponent.active.navigationSystem.navMesh);

            obstaclesKdTree.Build(obstacles);
        }
    }

    public class DynamicObstacle
    {
        public int obstacleIndex;
        public float remainingTime;
    }
}
