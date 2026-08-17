using System.Collections.Generic;
using DelaunatorSharp;
using GenericCode;
using UnityEngine;

namespace TriangulationNavigation
{
    public class AgentsMover
    {
        const float epsilon = 0.001f;

        int findPathsIndex = 0;
        public int maxRepathsCount;
        float velocitySmoothingMin;
        float velocitySmoothingMax;
        public int localAvoidancePowerFactor;
        public int maxLocalAvoidanceNeighbours;
        public Delaunator triangulation;

        public bool useKdTreeForNeighbours = false;

        public AgentsMover()
        {
            triangulation = new Delaunator();
        }

        public void SetVelocitySmoothing(float p_velocitySmoothingMin, float p_velocitySmoothingMax)
        {
            velocitySmoothingMin = p_velocitySmoothingMin;
            velocitySmoothingMax = p_velocitySmoothingMax;
        }

        public void MoveAgents(
            List<Agent> agents,
            List<Float2> agentPositions,
            List<Transform> agentTransforms,
            List<List<int>> agentNeighbours,
            List<int> agentNeighbourCounts,
            List<AgentType> agentTypes,
            float largestAgentRadius,
            NavMesh navMesh,
            Pathfinding pathfinding,
            float deltaTime)
        {
            Repath(agents, agentPositions, pathfinding, navMesh);
            CalculatePathVelocities(agents, agentPositions, navMesh);

            if (useKdTreeForNeighbours)
            {
                FindNeighboursWithKdTree(agents, agentPositions, agentNeighbours, agentNeighbourCounts, agentTypes);
            }
            else
            {
                if (agentPositions.Count < 3)
                {
                    FindNeighboursDirect(agents, agentPositions, agentNeighbours, agentNeighbourCounts, largestAgentRadius);
                }
                else
                {
                    triangulation.Create(agentPositions);
                    FindNeighboursWithTriangulation(agentNeighbours, agentNeighbourCounts);
                }
            }

            CalculateVelocitiesFromNeighbours(agents, agentPositions, agentTransforms, agentNeighbours, agentNeighbourCounts, agentTypes, navMesh, deltaTime);
        }

        public void CalculatePathVelocities(
            List<Agent> agents,
            List<Float2> agentPositions,
            NavMesh navMesh)
        {
            for (int i = 0; i < agents.Count; i++)
            {
                agents[i].pathVelocity = FollowPath(agentPositions[i], agents[i], navMesh);
            }
        }

        public void FindNeighboursDirect(
            List<Agent> agents,
            List<Float2> agentPositions,
            List<List<int>> agentNeighbours,
            List<int> agentNeighbourCounts,
            float largestAgentRadius)
        {
            for (int i = 0; i < agentNeighbourCounts.Count; i++)
            {
                agentNeighbourCounts[i] = 0;
            }

            float maxNeighbourDistanceSquare = 4.0f * largestAgentRadius * largestAgentRadius;

            for (int i = 0; i < agents.Count; i++)
            {
                Float2 agentPosition = agentPositions[i];

                for (int j = i + 1; j < agents.Count; j++)
                {
                    Float2 relative = agentPosition - agentPositions[j];
                    float distanceSquare = relative.LengthSquared();

                    if (distanceSquare <= maxNeighbourDistanceSquare)
                    {
                        if (agentNeighbourCounts[i] < agentNeighbours[i].Count)
                        {
                            agentNeighbours[i][agentNeighbourCounts[i]] = j;
                        }
                        else
                        {
                            agentNeighbours[i].Add(j);
                        }

                        if (agentNeighbourCounts[j] < agentNeighbours[j].Count)
                        {
                            agentNeighbours[j][agentNeighbourCounts[j]] = i;
                        }
                        else
                        {
                            agentNeighbours[j].Add(i);
                        }

                        agentNeighbourCounts[i]++;
                        agentNeighbourCounts[j]++;
                    }
                }
            }

            for (int i = 0; i < agentNeighbourCounts.Count; i++)
            {
                if (agentNeighbours[i].Count > agentNeighbourCounts[i] + 2)
                {
                    agentNeighbours[i].Resize(agentNeighbourCounts[i]);
                }
            }
        }

        public void FindNeighboursWithTriangulation(
            List<List<int>> agentNeighbours,
            List<int> agentNeighbourCounts)
        {
            for (int i = 0; i < agentNeighbourCounts.Count; i++)
            {
                agentNeighbourCounts[i] = 0;
            }

            for (int e = 0; e < triangulation.trianglesLen; e++)
            {
                if (e > triangulation.halfedges[e])
                {
                    int p = triangulation.triangles[e];
                    int q = triangulation.triangles[Delaunator.NextHalfedge(e)];

                    if (agentNeighbourCounts[p] < agentNeighbours[p].Count)
                    {
                        agentNeighbours[p][agentNeighbourCounts[p]] = q;
                    }
                    else
                    {
                        agentNeighbours[p].Add(q);
                    }

                    if (agentNeighbourCounts[q] < agentNeighbours[q].Count)
                    {
                        agentNeighbours[q][agentNeighbourCounts[q]] = p;
                    }
                    else
                    {
                        agentNeighbours[q].Add(p);
                    }

                    agentNeighbourCounts[p]++;
                    agentNeighbourCounts[q]++;
                }
            }

            for (int i = 0; i < agentNeighbourCounts.Count; i++)
            {
                if (agentNeighbours[i].Count > agentNeighbourCounts[i] + 2)
                {
                    agentNeighbours[i].Resize(agentNeighbourCounts[i]);
                }
            }
        }

        public void FindNeighboursWithKdTree(
            List<Agent> agents,
            List<Float2> agentPositions,
            List<List<int>> agentNeighbours,
            List<int> agentNeighbourCounts,
            List<AgentType> agentTypes)
        {
            List<List<int>> radialAgentIndices = new List<List<int>>();
            List<List<Float2>> radialAgentPositions = new List<List<Float2>>();
            List<int> radialAgentCounts = new List<int>();
            List<KDTree2D> radialKdTrees = new List<KDTree2D>();

            for (int i = 0; i < agentTypes.Count; i++)
            {
                radialAgentIndices.Add(new List<int>());
                radialAgentPositions.Add(new List<Float2>());
                radialAgentCounts.Add(0);
                radialKdTrees.Add(null);
            }

            for (int i = 0; i < agents.Count; i++)
            {
                int agentTypeIndex = agents[i].agentTypeIndex;
                radialAgentIndices[agentTypeIndex].Add(i);
                radialAgentPositions[agentTypeIndex].Add(agentPositions[i]);
                radialAgentCounts[agentTypeIndex]++;
            }

            for (int i = 0; i < agentTypes.Count; i++)
            {
                if (radialAgentCounts[i] > 0)
                {
                    radialKdTrees[i] = KDTree2D.MakeFromPoints(radialAgentPositions[i].ToArray());
                }
            }

            for (int i = 0; i < agentNeighbourCounts.Count; i++)
            {
                agentNeighbourCounts[i] = 0;
            }

            List<int> relativeNeighbours = new List<int>();

            for (int i = 0; i < agents.Count; i++)
            {
                Float2 agentPosition = agentPositions[i];
                Float2 avoidanceVelocity = Float2.Zero();
                int agentTypeIndex = agents[i].agentTypeIndex;
                float radius = agentTypes[agentTypeIndex].radius;

                for (int j = 0; j < agentTypes.Count; j++)
                {
                    if (radialAgentCounts[j] > 0)
                    {
                        int maxNeighbours = maxLocalAvoidanceNeighbours;
                        if (j != agentTypeIndex)
                        {
                            maxNeighbours = maxLocalAvoidanceNeighbours - 1;
                        }

                        float radiiSum = radius + agentTypes[j].radius;
                        relativeNeighbours.Clear();
                        radialKdTrees[j].FindNearestsBall(agentPosition, radiiSum, relativeNeighbours);

                        for (int k = 0; k < relativeNeighbours.Count; k++)
                        {
                            int relativeNeighbour = relativeNeighbours[k];
                            int neighbour = radialAgentIndices[j][relativeNeighbour];

                            if (neighbour != i)
                            {
                                if (agentNeighbourCounts[i] < agentNeighbours[i].Count)
                                {
                                    agentNeighbours[i][agentNeighbourCounts[i]] = neighbour;
                                }
                                else
                                {
                                    agentNeighbours[i].Add(neighbour);
                                }

                                agentNeighbourCounts[i]++;
                            }
                        }
                    }
                }
            }
        }

        public void CalculateVelocitiesFromNeighbours(
            List<Agent> agents,
            List<Float2> agentPositions,
            List<Transform> agentTransforms,
            List<List<int>> agentNeighbours,
            List<int> agentNeighbourCounts,
            List<AgentType> agentTypes,
            NavMesh navMesh,
            float deltaTime)
        {
            for (int i = 0; i < agents.Count; i++)
            {
                Float2 agentPosition = agentPositions[i];
                Float2 avoidanceVelocity = Float2.Zero();
                float powerFactorSum = 0.0f;
                int agentTypeIndex = agents[i].agentTypeIndex;
                float radius = agentTypes[agentTypeIndex].radius;

                List<int> neighbours = agentNeighbours[i];
                int neighboursCount = agentNeighbourCounts[i];

                for (int k = 0; k < neighboursCount; k++)
                {
                    int neighbour = neighbours[k];
                    int neighbourAgentTypeIndex = agents[neighbour].agentTypeIndex;
                    float neighbourRadius = agentTypes[neighbourAgentTypeIndex].radius;

                    float radiiSum = radius + neighbourRadius;
                    float radiiSumSquare = radiiSum * radiiSum;

                    Float2 relative = agentPosition - agentPositions[neighbour];
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

                agents[i].localAvoidanceVelocity = avoidanceVelocity;
                agents[i].powerFactorSum = powerFactorSum;

                if (!agentTypes[agentTypeIndex].isStatic)
                {
                    CalculateFinalVelocities(agents, agentPositions, agentTransforms, deltaTime, navMesh, i);
                }
            }
        }

        void CalculateFinalVelocities(
            List<Agent> agents,
            List<Float2> agentPositions,
            List<Transform> agentTransforms,
            float deltaTime,
            NavMesh navMesh,
            int i)
        {
            float previousDensity = agents[i].density;
            float density = 1.0f - MathUtils.Clamp01(1.0f / agents[i].powerFactorSum);
            density = 0.9f * previousDensity + 0.1f * density;
            agents[i].density = density;

            Float2 finalVelocity = agents[i].finalVelocity;
            float currentVelocitySmoothingFactor = MathUtils.InterpolateClamped(density, 0.0f, 1.0f, velocitySmoothingMin, velocitySmoothingMax);
            float speedDeltaTime = agents[i].speed * deltaTime;

            finalVelocity = finalVelocity * currentVelocitySmoothingFactor +
                         (agents[i].pathVelocity + agents[i].localAvoidanceVelocity).Normalized() * (1.0f - currentVelocitySmoothingFactor) * speedDeltaTime;

            Float2 currentPosition = agentPositions[i];
            Float2 newPosition = currentPosition + finalVelocity;
            GetNearestWalkablePositionResult getNearestWalkablePositionResult = navMesh.TryMoveToWalkableArea(newPosition);

            if (getNearestWalkablePositionResult.wasMoved)
            {
                newPosition = getNearestWalkablePositionResult.position;
                finalVelocity = newPosition - currentPosition;

                if (finalVelocity.LengthSquared() > speedDeltaTime * speedDeltaTime)
                {
                    finalVelocity = finalVelocity.Normalized() * speedDeltaTime;
                }
            }
            agents[i].finalVelocity = finalVelocity;
            agentPositions[i] += finalVelocity;
            agentTransforms[i].position = new Vector3(agentPositions[i].x, 0f, agentPositions[i].y);
        }

        void FindNearestsBall(
            List<List<int>> allNeighbours,
            int point,
            List<Float2> positions,
            List<bool> allVisited,
            float radiusSqr,
            List<int> neighbours,
            List<float> relativeNeighboursDistSqr)
        {
            List<int> openSet = new List<int>();
            Float2 pointPosition = positions[point];

            List<int> visited = new List<int>();

            for (int i = 0; i < allNeighbours[point].Count; i++)
            {
                int neighbour = allNeighbours[point][i];
                openSet.Add(neighbour);
                allVisited[neighbour] = true;
                visited.Add(neighbour);
            }

            while (openSet.Count > 0)
            {
                int neighbour = openSet[openSet.Count - 1];
                float sqrDistance = (positions[neighbour] - pointPosition).LengthSquared();

                if (sqrDistance < 2.0f * radiusSqr)
                {
                    if (sqrDistance < radiusSqr)
                    {
                        neighbours.Add(neighbour);
                        relativeNeighboursDistSqr.Add(sqrDistance);
                    }

                    for (int i = 0; i < allNeighbours[neighbour].Count; i++)
                    {
                        int neighbourOfNeighbour = allNeighbours[neighbour][i];
                        if (!allVisited[neighbourOfNeighbour])
                        {
                            openSet.Add(neighbourOfNeighbour);
                            allVisited[neighbourOfNeighbour] = true;
                            visited.Add(neighbourOfNeighbour);
                        }
                    }
                }

                openSet.RemoveAt(openSet.Count - 1);
            }

            for (int i = 0; i < visited.Count; i++)
            {
                allVisited[visited[i]] = false;
            }
        }

        public void Repath(List<Agent> agents, List<Float2> agentPositions, Pathfinding pathfinding, NavMesh navMesh)
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
                    float remainingPathDistance = (agents[findPathsIndex].waypoints[currentWaypointIndex] - agentPositions[findPathsIndex]).Length() +
                        PathUtils.CalculatePathLength(agents[findPathsIndex].waypoints, currentWaypointIndex);

                    if (remainingPathDistance >= agents[findPathsIndex].remainingPathDistance)
                    {
                        agents[findPathsIndex].pathMovementFailuresCount++;

                        if (agents[findPathsIndex].pathMovementFailuresCount > 50)
                        {
                            Float2 targetPosition = agents[findPathsIndex].destination;
                            Path path = pathfinding.FindPath(agentPositions[findPathsIndex], targetPosition, navMesh);

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
                else if (agents[findPathsIndex].searchPathLater)
                {
                    agents[findPathsIndex].pathMovementFailuresCount++;

                    if (agents[findPathsIndex].pathMovementFailuresCount > 100)
                    {
                        Float2 targetPosition = agents[findPathsIndex].destination;
                        Path path = pathfinding.FindPath(agentPositions[findPathsIndex], targetPosition, navMesh);

                        if (path.success)
                        {
                            agents[findPathsIndex].waypoints = path.waypoints;
                            agents[findPathsIndex].currentWaypointIndex = 0;
                            agents[findPathsIndex].followingPath = true;
                            agents[findPathsIndex].searchPathLater = false;
                        }

                        agents[findPathsIndex].pathMovementFailuresCount = 0;
                        repathsCount++;

                        if (repathsCount >= maxRepathsCount)
                        {
                            return;
                        }
                    }
                }
            }
        }

        Float2 FollowPath(Float2 position, Agent agent, NavMesh navMesh)
        {
            if (!agent.followingPath)
            {
                return Float2.Zero();
            }

            Float2 relative = agent.waypoints[agent.currentWaypointIndex] - position;
            float stopDistanceSquare = MathUtils.InterpolateClamped(agent.density, 0.0f, 1.0f, 0.01f, 1.5f);
            float currentDistanceSqr = relative.LengthSquared();

            if (currentDistanceSqr < stopDistanceSquare)
            {
                bool canNextWaypointBeReached = true;
                int nextWaypoint = agent.currentWaypointIndex + 1;

                if (nextWaypoint < agent.waypoints.Count &&
                    currentDistanceSqr > 0.2f * stopDistanceSquare &&
                    !navMesh.CanPointsBeReachedInStraightLine(position, agent.waypoints[nextWaypoint])
                )
                {
                    canNextWaypointBeReached = false;
                }

                if (canNextWaypointBeReached)
                {
                    agent.currentWaypointIndex++;
                    if (agent.currentWaypointIndex >= agent.waypoints.Count)
                    {
                        agent.followingPath = false;
                        return new Float2(0.0f, 0.0f);
                    }
                }
            }

            return relative.Normalized();
        }
    }
}
