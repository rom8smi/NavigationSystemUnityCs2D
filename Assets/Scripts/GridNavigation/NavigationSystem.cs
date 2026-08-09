using System.Collections.Generic;
using UnityEngine;
using GenericCode;

namespace GridNavigation
{
    public class NavigationSystem : MonoBehaviour
    {
        public static NavigationSystem active;

        public Transform[] obstacleTransforms;
        public Transform[] penaltyObstacleTransforms;
        public float[] agentRadii;

        public Float2 gridWorldSize;
        public float nodeRadius;
        public int obstacleProximityPenalty;

        Grid grid;
        Pathfinding pathfinding;
        AgentsMover agentsMover;
        GizmosDrawer gizmosDrawer;

        public bool displayGridGizmos;
        public int penaltyMin;
        public int penaltyMax;
        public bool debugObstacles;

        List<Transform> agentTransforms;
        public List<Agent> agents;
        List<Float2> agentPositions;
        public int numberOfSmoothIterations;
        public bool shouldFollowPath;
        public bool showPaths;
        public int maxRepathsCount;
        public float velocitySmoothingMin;
        public float velocitySmoothingMax;
        public int localAvoidancePowerFactor;
        public int maxLocalAvoidanceNeighbours;
        float fUpdates;

        void Awake()
        {
            active = this;
            List<Obstacle> obstacles = GetObstacles();
            RandomObstaclesSpawner randomObstaclesSpawner = FindAnyObjectByType<RandomObstaclesSpawner>();

            if (randomObstaclesSpawner != null)
            {
                List<Obstacle> randomObstacles = randomObstaclesSpawner.CreateObstacles();
                for (int i = 0; i < randomObstacles.Count; i++)
                {
                    obstacles.Add(randomObstacles[i]);
                }
            }

            grid = new Grid
            {
                obstacles = obstacles.ToArray(),
                penaltyObstacles = GetPenaltyObstacles(),
                gridWorldSize = new Float2(gridWorldSize.x, gridWorldSize.y),
                nodeRadius = nodeRadius,
                obstacleProximityPenalty = obstacleProximityPenalty,
                gridWorldOrigin = new Float2(transform.position.x - gridWorldSize.x * 0.5f, transform.position.z - gridWorldSize.y * 0.5f)
            };
            grid.Setup();
            pathfinding = new Pathfinding(grid);
            pathfinding.smoothPath = true;
            pathfinding.numberOfSmoothIterations = numberOfSmoothIterations;

            agentsMover = new AgentsMover();
            agentsMover.maxRepathsCount = maxRepathsCount;
            agentsMover.localAvoidancePowerFactor = localAvoidancePowerFactor;
            agentsMover.maxLocalAvoidanceNeighbours = maxLocalAvoidanceNeighbours;
            agentsMover.SetVelocitySmoothing(velocitySmoothingMin, velocitySmoothingMax);

            agentTransforms = new List<Transform>();
            agents = new List<Agent>();
            agentPositions = new List<Float2>();

            gizmosDrawer = new GizmosDrawer();
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
                        agentsMover.Repath(agents, agentPositions, pathfinding);
                        agentsMover.MoveAgents(agents, agentPositions, agentTransforms, agentRadii, grid, RuntimeConstants.deltaTime);
                    }

                    fUpdates -= iUpdates;
                }
            }
        }

        public void AddAgentTransform(Transform agentTransform, int radiusIndex, float speed, Float2 targetPosition)
        {
            agentTransforms.Add(agentTransform);

            Agent agent = new Agent();
            agent.radiusIndex = radiusIndex;
            agent.speed = speed;
            agents.Add(agent);

            Float2 agentPosition = new Float2(agentTransform.position.x, agentTransform.position.z);

            for (int j = 0; j < 2; j++)
            {
                agentPosition = VectorUtils.AdjustForBoundaries(agentPosition, grid.worldMinX, grid.worldMaxX, grid.worldMinY, grid.worldMaxY, 0.01f);
                agentPosition = grid.GetNearestWalkablePosition(agentPosition);
            }

            agentTransform.position = new Vector3(agentPosition.x, 0.0f, agentPosition.y);

            Path path = pathfinding.FindPath(agentPosition, targetPosition);
            if (path.success)
            {
                agent.waypoints = path.waypoints;
                agent.followingPath = true;
                agent.destination = targetPosition;
                agent.currentWaypointIndex = 0;
            }

            agentPositions.Add(agentPosition);
        }

        List<Obstacle> GetObstacles()
        {
            int obstaclesLength = obstacleTransforms.Length;
            List<Obstacle> obstacles = new List<Obstacle>();

            for (int i = 0; i < obstaclesLength; i++)
            {
                Vector3 position = obstacleTransforms[i].position;
                Vector3 scale = obstacleTransforms[i].lossyScale;

                obstacles.Add(
                    new Obstacle
                    {
                        center = new Float2(position.x, position.z),
                        size = new Float2(scale.x, scale.z)
                    }
                );
            }

            DebugObstacles(obstacles);
            return obstacles;
        }

        PenaltyObstacle[] GetPenaltyObstacles()
        {
            int penaltyObstaclesLength = penaltyObstacleTransforms.Length;
            PenaltyObstacle[] penaltyObstacles = new PenaltyObstacle[penaltyObstaclesLength];

            for (int i = 0; i < penaltyObstaclesLength; i++)
            {
                Vector3 position = penaltyObstacleTransforms[i].position;
                Vector3 scale = penaltyObstacleTransforms[i].lossyScale;

                penaltyObstacles[i] = new PenaltyObstacle
                {
                    center = new Float2(position.x, position.z),
                    size = new Float2(scale.x, scale.z),
                    penalty = 5
                };
            }

            DebugPenaltyObstacles(penaltyObstacles);
            return penaltyObstacles;
        }

        void DebugObstacles(List<Obstacle> obstacles)
        {
            if (debugObstacles)
            {
                string result = "Obstacles:\n";

                for (int i = 0; i < obstacles.Count; i++)
                {
                    Float2 center = obstacles[i].center;
                    Float2 size = obstacles[i].size;
                    result += $"Obstacle {i} - center ({center.x}, {center.y}) | size ({size.x}, {size.y})\n";
                }

                UnityEngine.Debug.Log(result);
            }
        }

        void DebugPenaltyObstacles(PenaltyObstacle[] penaltyObstacles)
        {
            if (debugObstacles)
            {
                string result = "PenaltyObstacles:\n";

                for (int i = 0; i < penaltyObstacles.Length; i++)
                {
                    Float2 center = penaltyObstacles[i].center;
                    Float2 size = penaltyObstacles[i].size;
                    result += $"PenaltyObstacles {i} - center ({center.x}, {center.y}) | size ({size.x}, {size.y})\n";
                }

                UnityEngine.Debug.Log(result);
            }
        }

        void OnDrawGizmos()
        {
            if (!Application.isPlaying)
            {
                return;
            }

            gizmosDrawer.DrawGizmos(grid, transform.position, displayGridGizmos, penaltyMin, penaltyMax, grid.nodeDiameter - 0.1f * grid.nodeDiameter);
            gizmosDrawer.DrawAgentsGizmos(showPaths, agentTransforms, agents);
        }
    }
}
