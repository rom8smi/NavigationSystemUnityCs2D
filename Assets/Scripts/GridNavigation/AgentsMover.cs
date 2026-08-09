using System.Collections.Generic;
using GenericCode;
using UnityEngine;

namespace GridNavigation
{
    public class AgentsMover
    {
        const float largestRadius = 2f;
        const float epsilon = 0.0001f;
        int findPathsIndex = 0;
        public int maxRepathsCount;
        float velocitySmoothingMin;
        float velocitySmoothingMax;
        public int localAvoidancePowerFactor;
        public int maxLocalAvoidanceNeighbours;

        public void SetVelocitySmoothing(float p_velocitySmoothingMin, float p_velocitySmoothingMax)
        {
            velocitySmoothingMin = p_velocitySmoothingMin;
            velocitySmoothingMax = p_velocitySmoothingMax;
        }

        public void MoveAgents(
            List<Agent> agents,
            List<Float2> agentPositions,
            List<Transform> agentTransforms,
            float[] agentRadii,
            Grid grid,
            float deltaTime)
        {
            List<List<int>> radialAgentIndices = new List<List<int>>();
            List<List<Float2>> radialAgentPositions = new List<List<Float2>>();
            List<int> radialAgentCounts = new List<int>();
            List<KDTree2D> radialKdTrees = new List<KDTree2D>();

            for (int i = 0; i < agentRadii.Length; i++)
            {
                radialAgentIndices.Add(new List<int>());
                radialAgentPositions.Add(new List<Float2>());
                radialAgentCounts.Add(0);
                radialKdTrees.Add(null);
            }

            for (int i = 0; i < agents.Count; i++)
            {
                int radiusIndex = agents[i].radiusIndex;
                radialAgentIndices[radiusIndex].Add(i);
                radialAgentPositions[radiusIndex].Add(agentPositions[i]);
                radialAgentCounts[radiusIndex]++;
            }

            for (int i = 0; i < agentRadii.Length; i++)
            {
                if (radialAgentCounts[i] > 0)
                {
                    radialKdTrees[i] = KDTree2D.MakeFromPoints(radialAgentPositions[i].ToArray());
                }
            }

            List<int> relativeNeighbours = new List<int>();

            for (int i = 0; i < agents.Count; i++)
            {
                Float2 pathVelocity = FollowPath(agentPositions[i], agents[i]);

                Float2 avoidanceVelocity = Float2.Zero();
                float powerFactorSum = 0.0f;
                int radiusIndex = agents[i].radiusIndex;
                float radius = agentRadii[radiusIndex];

                for (int j = 0; j < agentRadii.Length; j++)
                {
                    if (radialAgentCounts[j] > 0)
                    {
                        int maxNeighbours = maxLocalAvoidanceNeighbours;
                        if (j != radiusIndex)
                        {
                            maxNeighbours = maxLocalAvoidanceNeighbours - 1;
                        }

                        float radiiSum = radius + agentRadii[j];
                        relativeNeighbours.Clear();
                        radialKdTrees[j].FindNearestsBall(agentPositions[i], radiiSum, relativeNeighbours);

                        for (int k = 0; k < relativeNeighbours.Count; k++)
                        {
                            int relativeNeighbour = relativeNeighbours[k];
                            int neighbour = radialAgentIndices[j][relativeNeighbour];

                            if (neighbour != i)
                            {
                                int neighbourRadiusIndex = agents[neighbour].radiusIndex;
                                float radiiSumSquare = radiiSum * radiiSum;

                                Float2 relative = agentPositions[i] - agentPositions[neighbour];
                                float distanceSquare = relative.LengthSquared();
                                if (distanceSquare < epsilon)
                                {
                                    distanceSquare = epsilon;
                                }

                                float normalizedDistanceSquare = distanceSquare / radiiSumSquare;

                                if (normalizedDistanceSquare < 1.0f)
                                {
                                    float powerFactor = 1.0f;
                                    for (int l = 0; l < localAvoidancePowerFactor; l++)
                                    {
                                        powerFactor *= normalizedDistanceSquare;
                                    }
                                    avoidanceVelocity += relative.Normalized() / powerFactor;
                                    powerFactorSum += 1.0f / powerFactor;
                                }
                            }
                        }
                    }
                }

                float previousDensity = agents[i].density;
                float density = 1.0f - Mathf.Clamp01(1.0f / powerFactorSum);
                density = 0.9f * previousDensity + 0.1f * density;
                agents[i].density = density;

                Float2 finalVelocity = agents[i].finalVelocity;
                float currentVelocitySmoothingFactor = MathUtils.InterpolateClamped(density, 0.0f, 1.0f, velocitySmoothingMin, velocitySmoothingMax);

                finalVelocity = currentVelocitySmoothingFactor * finalVelocity +
                                (1.0f - currentVelocitySmoothingFactor) * (pathVelocity + avoidanceVelocity).Normalized() * agents[i].speed * deltaTime;

                agents[i].finalVelocity = finalVelocity;

                Float2 newPosition = agentPositions[i] + finalVelocity;
                for (int j = 0; j < 2; j++)
                {
                    newPosition = VectorUtils.AdjustForBoundaries(newPosition, grid.worldMinX, grid.worldMaxX, grid.worldMinY, grid.worldMaxY, 0.01f);
                    newPosition = grid.GetNearestWalkablePosition(newPosition);
                }
                agentPositions[i] = newPosition;

                agentTransforms[i].position = new Vector3(agentPositions[i].x, 0f, agentPositions[i].y);
            }
        }

        public void Repath(List<Agent> agents, List<Float2> agentPositions, Pathfinding pathfinding)
        {
            int repathsCount = 0;

            for (int i = 0; i < agents.Count; i++)
            {
                findPathsIndex++;
                if (findPathsIndex >= agents.Count)
                {
                    findPathsIndex = 0;
                }

                if (agents[findPathsIndex].followingPath)
                {
                    int currentWaypointIndex = agents[findPathsIndex].currentWaypointIndex;
                    float remainingPathDistance = (agents[findPathsIndex].waypoints[currentWaypointIndex] - agentPositions[findPathsIndex]).Length();
                    int lastWaypointIndex = agents[findPathsIndex].waypoints.Count - 1;

                    for (int j = currentWaypointIndex; j < lastWaypointIndex; j++)
                    {
                        remainingPathDistance += (agents[findPathsIndex].waypoints[j + 1] - agents[findPathsIndex].waypoints[j]).Length();
                    }

                    if (remainingPathDistance >= agents[findPathsIndex].remainingPathDistance)
                    {
                        agents[findPathsIndex].pathMovementFailuresCount++;

                        if (agents[findPathsIndex].pathMovementFailuresCount > 10)
                        {
                            Float2 targetPosition = agents[findPathsIndex].destination;

                            Path path = pathfinding.FindPath(agentPositions[findPathsIndex], targetPosition);
                            if (path.success)
                            {
                                agents[findPathsIndex].waypoints = path.waypoints;
                                agents[findPathsIndex].currentWaypointIndex = 0;
                            }

                            agents[findPathsIndex].pathMovementFailuresCount = 0;
                            repathsCount++;

                            if (repathsCount >= maxRepathsCount)
                            {
                                return;
                            }
                        }
                    }

                    agents[findPathsIndex].remainingPathDistance = remainingPathDistance;
                }
            }
        }

        Float2 FollowPath(Float2 position, Agent agent)
        {
            if (!agent.followingPath)
            {
                return Float2.Zero();
            }

            Float2 relative = agent.waypoints[agent.currentWaypointIndex] - position;
            float stopDistanceSquare = MathUtils.InterpolateClamped(agent.density, 0.0f, 1.0f, 0.01f, 1.5f);

            if (relative.LengthSquared() < stopDistanceSquare)
            {
                agent.currentWaypointIndex++;
                if (agent.currentWaypointIndex >= agent.waypoints.Count)
                {
                    agent.followingPath = false;
                    return Float2.Zero();
                }
            }

            return relative.Normalized();
        }
    }
}
