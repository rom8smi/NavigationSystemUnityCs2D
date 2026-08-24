using System.Collections.Generic;
using UnityEngine;
using GenericCode;

namespace TriangulationNavigation
{
    public class NavigationSystemComponent : MonoBehaviour
    {
        public static NavigationSystemComponent active;

        public NavigationSystem navigationSystem;

        public Material navMeshSurfaceMaterial;
        public List<Transform> obstacleTransforms;
        public List<AgentType> agentTypes;
        public float worldSize;

        GizmoDrawer gizmoDrawer;

        public bool displayNavmeshGizmos;
        public bool displayFunnelGizmos;
        public bool displayFunnelPointNumbers;
        public bool displayUnwalkableEdges;
        public bool debugObstacles;

        public List<Transform> agentTransforms;

        public bool shouldFollowPath;
        public bool showPaths;
        public int maxRepathsCount;
        public float velocitySmoothingMin;
        public float velocitySmoothingMax;
        public int localAvoidancePowerFactor;
        public int maxLocalAvoidanceNeighbours;
        public bool useDebugAgentsMover;
        public bool drawPointNumbers;
        public bool drawTriangleNumbers;

        public bool drawPushDirections;
        List<Float2> pushDirectionsDisplayPosition;
        float pushDirectionsDisplaySize;
        [HideInInspector] public float cameraScaleMultiplier = 1.0f;
        float fUpdates;

        void Awake()
        {
            active = this;

            navigationSystem = new NavigationSystem(agentTypes);
            navigationSystem.use_debug_agents_mover = useDebugAgentsMover;

            gizmoDrawer = new GizmoDrawer();

            List<Obstacle> obstacles = GetObstacles(worldSize);
            AddRandomObstacles(obstacles);
            AddChainedObstacles(obstacles);

            navigationSystem.obstacles = obstacles;
            navigationSystem.navMesh.Create(obstacles, worldSize);
            navigationSystem.navMeshDrawer.CreateNavMeshSurfaceDrawer(navigationSystem.navMesh);

            navigationSystem.pathfinding.CreateNodes(navigationSystem.navMesh);

            navigationSystem.agentsMover.maxRepathsCount = maxRepathsCount;
            navigationSystem.agentsMover.localAvoidancePowerFactor = localAvoidancePowerFactor;
            navigationSystem.agentsMover.maxLocalAvoidanceNeighbours = maxLocalAvoidanceNeighbours;
            navigationSystem.agentsMover.SetVelocitySmoothing(velocitySmoothingMin, velocitySmoothingMax);

            agentTransforms = new List<Transform>();
            pushDirectionsDisplayPosition = new List<Float2>();

            int nPushDirectionsDisplayPosition = 40;
            float step = worldSize / nPushDirectionsDisplayPosition;
            pushDirectionsDisplaySize = 0.5f * step;

            for (int i = 0; i < nPushDirectionsDisplayPosition; i++)
            {
                for (int j = 0; j < nPushDirectionsDisplayPosition; j++)
                {
                    pushDirectionsDisplayPosition.Add(new Float2(-0.5f * worldSize + (i + 0.5f) * step, -0.5f * worldSize + (j + 0.5f) * step));
                }
            }
        }

        void Start()
        {

        }

        void Update()
        {
            if (Input.GetKeyDown(KeyCode.P))
            {
                shouldFollowPath = !shouldFollowPath;
            }

            if (shouldFollowPath)
            {
                fUpdates += RuntimeConstants.updatesPerFrame;
                if (fUpdates > 1.0f)
                {
                    int iUpdates = (int)fUpdates;

                    for (int i = 0; i < iUpdates; i++)
                    {
                        navigationSystem.Update(agentTransforms, RuntimeConstants.deltaTime);
                    }

                    fUpdates -= iUpdates;
                }
            }

            navigationSystem.navMeshDrawer.DrawSurface(navMeshSurfaceMaterial, displayNavmeshGizmos);
        }

        public void AddAgentTransform(Transform agentTransform, int agentTypeIndex, Float2 targetPosition)
        {
            agentTransforms.Add(agentTransform);

            Agent agent = new Agent();
            agent.agentTypeIndex = agentTypeIndex;
            agent.speed = agentTypes[agentTypeIndex].speed;
            navigationSystem.agents.Add(agent);

            Float2 agentPosition = new Float2(agentTransform.position.x, agentTransform.position.z);
            agentTransform.position = new Vector3(agentPosition.x, 0.0f, agentPosition.y);

            Path path = navigationSystem.pathfinding.FindPath(agentPosition, targetPosition, navigationSystem.navMesh);
            if (path.success)
            {
                agent.waypoints = path.waypoints;
                agent.simplifiedWaypoints = path.simplifiedWaypoints;
                agent.followingPath = true;
                agent.destination = targetPosition;
                agent.currentWaypointIndex = 0;
            }
            else
            {
                agent.searchPathLater = true;
                agent.destination = targetPosition;
                agent.currentWaypointIndex = 0;
            }

            navigationSystem.agentPositions.Add(agentPosition);
            navigationSystem.agentNeighbours.Add(new List<int>());
            navigationSystem.agentNeighbourCounts.Add(0);
        }

        public List<Obstacle> GetObstacles(float worldSize)
        {
            int obstaclesLength = obstacleTransforms.Count;
            List<Obstacle> obstacles = new List<Obstacle>();
            List<List<Float2>> allCorners = new List<List<Float2>>();
            List<bool> isObstacleWalkable = new List<bool>();

            for (int i = 0; i < obstaclesLength; i++)
            {
                if (obstacleTransforms[i].gameObject.activeSelf)
                {
                    Vector3 position = obstacleTransforms[i].position;
                    float rotation = -obstacleTransforms[i].rotation.eulerAngles.y * MathUtils.Deg2Rad();
                    Vector3 scale = obstacleTransforms[i].lossyScale;

                    RectangularObstacle rectangularObstacle = new RectangularObstacle
                    {
                        center = new Float2(position.x, position.z),
                        size = new Float2(scale.x, scale.z),
                        rotation = rotation,
                        radius = navigationSystem.largestAgentRadius
                    };

                    List<Float2> corners = rectangularObstacle.GetCorners();
                    allCorners.Add(corners);

                    if (obstacleTransforms[i].GetComponent<WalkableObstacleMonobehaviour>() != null)
                    {
                        isObstacleWalkable.Add(true);
                    }
                    else
                    {
                        isObstacleWalkable.Add(false);
                    }
                }
            }

            // UnionPolygonFinder.FindUnionsForMultiplePolygons(allCorners);

            AABB bounds = new AABB
            {
                minX = -0.5f * worldSize,
                maxX = 0.5f * worldSize,
                minY = -0.5f * worldSize,
                maxY = 0.5f * worldSize,
            };

            for (int i = 0; i < allCorners.Count; i++)
            {
                Obstacle obstacle = ObstacleUtils.Create(allCorners[i], bounds, 2.0f * worldSize, isObstacleWalkable[i]);
                obstacles.Add(obstacle);
            }

            // DebugObstacles(obstacles);
            return obstacles;
        }

        void AddRandomObstacles(List<Obstacle> obstacles)
        {
            RandomObstaclesSpawner obstaclesSpawner = FindAnyObjectByType<RandomObstaclesSpawner>();

            if (obstaclesSpawner != null)
            {
                List<Obstacle> additionalObstacles = obstaclesSpawner.CreateObstacles(worldSize, navigationSystem.largestAgentRadius);
                for (int i = 0; i < additionalObstacles.Count; i++)
                {
                    obstacles.Add(additionalObstacles[i]);
                }
            }
        }

        void AddChainedObstacles(List<Obstacle> obstacles)
        {
            ChainedObstaclesSpawner obstaclesSpawner = FindAnyObjectByType<ChainedObstaclesSpawner>();

            if (obstaclesSpawner != null)
            {
                List<Obstacle> additionalObstacles = obstaclesSpawner.CreateObstacles(worldSize);
                for (int i = 0; i < additionalObstacles.Count; i++)
                {
                    obstacles.Add(additionalObstacles[i]);
                }
            }
        }

        void OnDrawGizmos()
        {
            if (!Application.isPlaying)
            {
                return;
            }

            if (navigationSystem.navMeshDrawer != null)
            {
                navigationSystem.navMeshDrawer.DrawNavMesh(navigationSystem.navMesh, displayNavmeshGizmos, displayUnwalkableEdges, cameraScaleMultiplier);
                navigationSystem.navMeshDrawer.DrawNavMeshFunnelsGraph(navigationSystem.navMesh, navigationSystem.pathfinding, displayFunnelGizmos, displayFunnelPointNumbers, cameraScaleMultiplier);
                navigationSystem.navMeshDrawer.DrawObstaclePushDirections(navigationSystem.navMesh, drawPushDirections, pushDirectionsDisplayPosition, pushDirectionsDisplaySize);
                gizmoDrawer.DrawAgentsGizmos(showPaths, agentTransforms, navigationSystem.agents);
            }

            if (drawPointNumbers)
            {
                DrawPointNumbers();
            }
            if (drawTriangleNumbers)
            {
                DrawTriangleNumbers();
            }
        }

        void DrawPointNumbers()
        {
            for (int i = 0; i < navigationSystem.navMesh.allPoints.Count; i++)
            {
                Float2 pos2d = navigationSystem.navMesh.allPoints[i];
                navigationSystem.navMeshDrawer.DrawNumber(pos2d, i, cameraScaleMultiplier);
            }
        }

        void DrawTriangleNumbers()
        {
            for (int i = 0; i < navigationSystem.navMesh.allTriangleCentroids.Count; i++)
            {
                Float2 pos2d = navigationSystem.navMesh.allTriangleCentroids[i];
                navigationSystem.navMeshDrawer.DrawNumber(pos2d, i, cameraScaleMultiplier);
            }
        }
    }
}
