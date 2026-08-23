using System.Collections.Generic;
using DelaunatorSharp;
using GenericCode;

namespace TriangulationNavigation
{
    public class Pathfinding
    {
        Heap openSet;
        List<int> closedSet;
        List<PathfindingNode> nodes;
        public List<Float2> nodePositions;
        public List<List<int>> nodeNeighbours;
        List<float> additionalCosts;
        List<bool> additionalCostsModified;
        List<int> addedAdditionalCosts;
        int nodesCount;
        bool useIterations;
        float costIncrement;

        public Pathfinding()
        {
            openSet = new Heap();
            closedSet = new List<int>();
            nodes = new List<PathfindingNode>();
            nodePositions = new List<Float2>();
            nodeNeighbours = new List<List<int>>();
            additionalCosts = new List<float>();
            additionalCostsModified = new List<bool>();
            addedAdditionalCosts = new List<int>();
        }

        public void CreateNodes1(NavMesh navMesh)
        {
            useIterations = true;
            costIncrement = 1.0f;
            openSet.Clear(nodes);
            nodes.Clear();
            nodePositions.Clear();
            nodeNeighbours.Clear();
            additionalCosts.Clear();
            additionalCostsModified.Clear();

            for (int i = 0; i < navMesh.allPoints.Count; i++)
            {
                nodes.Add(new PathfindingNode
                {
                    gCost = 0,
                    hCost = 0,
                    parent = -1,
                    heapIndex = -1,
                    isInClosedSet = false
                });
                nodePositions.Add(navMesh.allPoints[i]);
                nodeNeighbours.Add(new List<int>());
                additionalCosts.Add(0.0f);
                additionalCostsModified.Add(false);
            }

            List<Edge> edges = navMesh.allEdges;

            for (int i = 0; i < edges.Count; i++)
            {
                int p = edges[i].p;
                int q = edges[i].q;

                if (navMesh.edgesWalkability[edges[i].index])
                {
                    nodeNeighbours[p].Add(q);
                    nodeNeighbours[q].Add(p);
                }
            }

            nodesCount = nodes.Count;

            for (int i = 0; i < 2; i++)
            {
                nodes.Add(new PathfindingNode());
                nodePositions.Add(Float2.Zero());
                nodeNeighbours.Add(new List<int>());
                additionalCosts.Add(0.0f);
                additionalCostsModified.Add(false);
            }
        }

        public void CreateNodes(NavMesh navMesh)
        {
            useIterations = true;
            costIncrement = 1.0f;
            openSet.Clear(nodes);
            nodes.Clear();
            nodePositions.Clear();
            nodeNeighbours.Clear();
            additionalCosts.Clear();
            additionalCostsModified.Clear();

            List<int> nodeEdgeRefs = new List<int>();
            nodeEdgeRefs.Resize(navMesh.delaunator.trianglesLen);

            for (int i = 0; i < navMesh.delaunator.trianglesLen; i++)
            {
                nodeEdgeRefs[i] = -1;
            }

            for (int t = 0; t < navMesh.delaunator.trianglesLen / 3; t++)
            {
                List<int> walkableIndices = new List<int>();
                List<bool> repetitive = new List<bool>();
                List<int> tempIndices = new List<int>();

                for (int i = 0; i < 3; i++)
                {
                    int e = t * 3 + i;
                    int opposite = navMesh.delaunator.halfedges[e];

                    if (opposite != -1)
                    {
                        if (navMesh.trianglesWalkability[Delaunator.TriangleOfEdge(e)] == -1 &&
                        navMesh.trianglesWalkability[Delaunator.TriangleOfEdge(opposite)] == -1)
                        {
                            walkableIndices.Add(i);
                            if (e < opposite)
                            {
                                repetitive.Add(false);
                            }
                            else
                            {
                                repetitive.Add(true);
                            }
                        }
                    }
                }

                for (int i = 0; i < walkableIndices.Count; i++)
                {
                    if (!repetitive[i])
                    {
                        int e = t * 3 + walkableIndices[i];
                        int opposite = navMesh.delaunator.halfedges[e];

                        int p = navMesh.delaunator.triangles[e];
                        int q = navMesh.delaunator.triangles[Delaunator.NextHalfedge(e)];

                        Float2 center = (navMesh.allPoints[p] + navMesh.allPoints[q]) * 0.5f;

                        int currentNodesCount = nodes.Count;
                        tempIndices.Add(currentNodesCount);

                        nodes.Add(new PathfindingNode
                        {
                            gCost = 0,
                            hCost = 0,
                            parent = -1,
                            heapIndex = -1,
                            isInClosedSet = false
                        });
                        nodePositions.Add(center);
                        nodeNeighbours.Add(new List<int>());
                        additionalCosts.Add(0.0f);
                        additionalCostsModified.Add(false);
                        nodeEdgeRefs[e] = currentNodesCount;
                        nodeEdgeRefs[opposite] = currentNodesCount;
                    }
                    else
                    {
                        int e = t * 3 + walkableIndices[i];
                        int opposite = navMesh.delaunator.halfedges[e];

                        tempIndices.Add(nodeEdgeRefs[opposite]);
                    }
                }

                for (int i = 0; i < walkableIndices.Count; i++)
                {
                    for (int j = i + 1; j < walkableIndices.Count; j++)
                    {
                        int firstNode = tempIndices[i];
                        int secondNode = tempIndices[j];
                        nodeNeighbours[firstNode].Add(secondNode);
                        nodeNeighbours[secondNode].Add(firstNode);
                    }
                }
            }

            nodesCount = nodes.Count;

            // for (int i = 0; i < 2; i++)
            // {
            //     nodes.Add(new PathfindingNode());
            //     nodePositions.Add(Float2.Zero());
            //     nodeNeighbours.Add(new List<int>());
            //     additionalCosts.Add(0.0f);
            //     additionalCostsModified.Add(false);
            // }
        }

        public Path FindPath(Float2 startPos, Float2 targetPos, NavMesh navMesh)
        {
            Path path = FindPathWithOrWithoutIterations(startPos, targetPos, navMesh);

            if (!path.success && path.lowestHCostNode < nodesCount)
            {
                Float2 newTargetPos = nodePositions[path.lowestHCostNode];
                newTargetPos = navMesh.FindNearestObstacleHullEdgePointToTarget(path.lowestHCostNode, newTargetPos, targetPos);
                path = FindPathWithOrWithoutIterations(startPos, newTargetPos, navMesh);
            }

            return path;
        }

        Path FindPathWithOrWithoutIterations(Float2 startPos, Float2 targetPos, NavMesh navMesh)
        {
            if (useIterations)
            {
                return FindPathWithIterations(startPos, targetPos, navMesh);
            }
            return FindPathWithoutIterations(startPos, targetPos, navMesh);
        }

        Path FindPathWithIterations(Float2 startPos, Float2 targetPos, NavMesh navMesh)
        {
            for (int i = 0; i < 2; i++)
            {
                targetPos = navMesh.TryMoveToWalkableArea(targetPos).position;
            }

            if (nodesCount != navMesh.allPoints.Count)
            {
                GenericCode.Debug.Log($"Pathfinding nodes and triangulation points count does not match: {nodesCount}, {navMesh.allPoints.Count}");
            }

            List<Path> paths = new List<Path>();
            for (int i = 0; i < 2; i++)
            {
                Path path = FindPathToExactTarget(startPos, targetPos, navMesh);

                if (!path.success || path.waypoints.Count < 2)
                {
                    ClearPathSearch();
                    ClearAdditionalCosts();
                    return path;
                }

                for (int j = 0; j < closedSet.Count; j++)
                {
                    int nodeIndex = closedSet[j];
                    additionalCosts[nodeIndex] += costIncrement;

                    if (!additionalCostsModified[nodeIndex])
                    {
                        addedAdditionalCosts.Add(nodeIndex);
                        additionalCostsModified[nodeIndex] = true;
                    }
                }

                ClearPathSearch();
                paths.Add(path);
            }

            ClearAdditionalCosts();

            float largestLength = float.MaxValue;
            Path shortestPath = null;

            for (int i = 0; i < paths.Count; i++)
            {
                float currentLenth = PathUtils.CalculateTotalPathLength(paths[i].waypoints);
                if (currentLenth < largestLength)
                {
                    largestLength = currentLenth;
                    shortestPath = paths[i];
                }
            }

            return shortestPath;
        }

        Path FindPathWithoutIterations(Float2 startPos, Float2 targetPos, NavMesh navMesh)
        {
            for (int i = 0; i < 2; i++)
            {
                targetPos = navMesh.TryMoveToWalkableArea(targetPos).position;
            }

            if (nodesCount != navMesh.allPoints.Count)
            {
                GenericCode.Debug.Log($"Pathfinding nodes and triangulation points count does not match: {nodesCount}, {navMesh.allPoints.Count}");
            }
            Path path = FindPathToExactTarget(startPos, targetPos, navMesh);
            ClearPathSearch();
            return path;
        }

        Path FindPathToExactTarget(Float2 startPos, Float2 targetPos, NavMesh navMesh)
        {
            int startNode = nodesCount;
            int targetNode = nodesCount + 1;

            int startTriangle = UpdatePositionNode(startPos, navMesh, startNode);
            int targetTriangle = UpdatePositionNode(targetPos, navMesh, targetNode);

            bool pathSuccess = false;
            float lowestHCost = float.MaxValue;
            PathfindingNode node;

            int lowestHCostNode = startNode;

            if (startTriangle != -1 && targetTriangle != -1)
            {
                if (startTriangle == targetTriangle)
                {
                    openSet.Clear(nodes);
                    closedSet.Clear();

                    return new Path
                    {
                        waypoints = new List<Float2> { targetPos },
                        success = true,
                        lowestHCostNode = lowestHCostNode
                    };
                }

                openSet.Add(startNode, nodes);

                while (openSet.count > 0)
                {
                    int currentNode = openSet.RemoveFirst(nodes);

                    node = nodes[currentNode];
                    node.isInClosedSet = true;
                    nodes[currentNode] = node;

                    closedSet.Add(currentNode);

                    if (currentNode == targetNode)
                    {
                        pathSuccess = true;
                        break;
                    }

                    List<int> neighbours = nodeNeighbours[currentNode];
                    int neighboursCount = neighbours.Count;

                    for (int i = 0; i < neighboursCount; i++)
                    {
                        int neighbour = neighbours[i];

                        if (nodes[neighbour].isInClosedSet)
                        {
                            continue;
                        }

                        float newMovementCostToNeighbour = nodes[currentNode].gCost + GetDistance(currentNode, neighbour);
                        if (newMovementCostToNeighbour + additionalCosts[currentNode] < nodes[neighbour].gCost || nodes[neighbour].heapIndex == -1)
                        {
                            node = nodes[neighbour];
                            node.gCost = newMovementCostToNeighbour;
                            nodes[neighbour] = node;

                            float hCost = GetDistance(neighbour, targetNode);
                            if (hCost < lowestHCost)
                            {
                                lowestHCostNode = neighbour;
                                lowestHCost = hCost;
                            }

                            node = nodes[neighbour];
                            node.hCost = hCost;
                            node.parent = currentNode;
                            nodes[neighbour] = node;

                            if (nodes[neighbour].heapIndex == -1)
                            {
                                openSet.Add(neighbour, nodes);
                            }
                            else
                            {
                                openSet.UpdateItem(neighbour, nodes);
                            }
                        }
                    }
                }
            }

            List<Float2> waypoints = new List<Float2>();

            if (pathSuccess)
            {
                RetracePath(waypoints, startNode, targetNode);
                // SimplifyPath(waypoints, navMesh);

                waypoints.RemoveAt(waypoints.Count - 1);
                waypoints = ReversePath(waypoints);
            }

            return new Path
            {
                waypoints = waypoints,
                success = pathSuccess,
                lowestHCostNode = lowestHCostNode
            };
        }

        void ClearPathSearch()
        {
            PathfindingNode node;
            for (int i = 0; i < closedSet.Count; i++)
            {
                int nodeIndex = closedSet[i];

                node = nodes[nodeIndex];
                node.gCost = 0;
                node.hCost = 0;
                node.parent = -1;
                node.heapIndex = -1;
                node.isInClosedSet = false;
                nodes[nodeIndex] = node;
            }

            openSet.Clear(nodes);
            closedSet.Clear();

            for (int i = nodesCount; i < nodesCount + 2; i++)
            {
                List<int> currentNodeNeighbours = nodeNeighbours[i];

                for (int j = 0; j < currentNodeNeighbours.Count; j++)
                {
                    int p = currentNodeNeighbours[j];
                    int lastNeighbourNode = nodeNeighbours[p].Count - 1;
                    nodeNeighbours[p].RemoveAt(lastNeighbourNode);
                }
            }
        }

        void ClearAdditionalCosts()
        {
            for (int i = 0; i < addedAdditionalCosts.Count; i++)
            {
                int nodeIndex = addedAdditionalCosts[i];
                additionalCosts[nodeIndex] = 0.0f;
                additionalCostsModified[nodeIndex] = false;
            }

            addedAdditionalCosts.Clear();
        }

        void RetracePath(List<Float2> waypoints, int startNode, int endNode)
        {
            int currentNode = endNode;
            Float2 waypointPosition;

            while (currentNode != startNode)
            {
                waypointPosition = nodePositions[currentNode];
                waypoints.Add(waypointPosition);
                currentNode = nodes[currentNode].parent;
            }

            waypointPosition = nodePositions[startNode];
            waypoints.Add(waypointPosition);
        }

        void SimplifyPath(List<Float2> waypoints, NavMesh navMesh)
        {
            List<bool> mergeConsidered = new List<bool>();
            List<float> straightLineDistancesSqr = new List<float>();

            mergeConsidered.Resize(waypoints.Count - 2);
            straightLineDistancesSqr.Resize(waypoints.Count - 2);

            for (int i = 0; i < mergeConsidered.Count; i++)
            {
                mergeConsidered[i] = false;
                straightLineDistancesSqr[i] = (waypoints[i] - waypoints[i + 2]).LengthSquared();
            }

            bool mergeFound = true;

            while (mergeFound)
            {
                mergeFound = false;
                float largestDistanceSqr = 0.0f;
                int largestDistanceSqrIndex = -1;

                for (int i = 0; i < straightLineDistancesSqr.Count; i++)
                {
                    if (!mergeConsidered[i])
                    {
                        if (straightLineDistancesSqr[i] > largestDistanceSqr)
                        {
                            largestDistanceSqr = straightLineDistancesSqr[i];
                            largestDistanceSqrIndex = i;
                            mergeFound = true;
                        }
                    }
                }

                if (mergeFound)
                {
                    mergeConsidered[largestDistanceSqrIndex] = true;
                    if (CanWaypointsBeMerged(waypoints, largestDistanceSqrIndex, navMesh))
                    {
                        int removalIndex = largestDistanceSqrIndex + 1;

                        waypoints.RemoveAt(removalIndex);

                        mergeConsidered.RemoveAt(largestDistanceSqrIndex);
                        straightLineDistancesSqr.RemoveAt(largestDistanceSqrIndex);

                        int waypointsCount = waypoints.Count;
                        if (removalIndex - 1 >= 0 && removalIndex + 1 < waypointsCount)
                        {
                            straightLineDistancesSqr[largestDistanceSqrIndex] = (waypoints[removalIndex - 1] - waypoints[removalIndex + 1]).LengthSquared();
                            mergeConsidered[largestDistanceSqrIndex] = false;
                        }
                        if (removalIndex - 2 >= 0 && removalIndex < waypointsCount)
                        {
                            straightLineDistancesSqr[largestDistanceSqrIndex - 1] = (waypoints[removalIndex - 2] - waypoints[removalIndex]).LengthSquared();
                            mergeConsidered[largestDistanceSqrIndex - 1] = false;
                        }
                    }
                }
            }
        }

        bool CanWaypointsBeMerged(
            List<Float2> waypoints,
            int i,
            NavMesh navMesh)
        {
            if (i + 2 >= waypoints.Count)
            {
                return false;
            }

            Float2 p1 = waypoints[i];
            Float2 p3 = waypoints[i + 2];

            if (navMesh.CanPointsBeReachedInStraightLine(p1, p3))
            {
                return true;
            }

            return false;
        }

        List<Float2> ReversePath(List<Float2> waypoints)
        {
            List<Float2> reversedWaypoints = new List<Float2>();
            int waypointsCount = waypoints.Count;

            for (int i = waypointsCount - 1; i >= 0; i--)
            {
                reversedWaypoints.Add(waypoints[i]);
            }

            return reversedWaypoints;
        }

        int UpdatePositionNode(Float2 position, NavMesh navMesh, int nodeIndex)
        {
            int triangle = navMesh.FindWalkableTriangleForPoint(position);

            if (triangle != -1 && navMesh.trianglesWalkability[triangle] != -1)
            {
                triangle = -1;
            }

            PathfindingNode node = new PathfindingNode
            {
                gCost = 0,
                hCost = 0,
                parent = -1,
                heapIndex = -1,
                isInClosedSet = false
            };
            List<int> neighbours = new List<int>();

            if (triangle != -1)
            {
                for (int i = 0; i < navMesh.allTriangles[triangle].points.Count; i++)
                {
                    int p = navMesh.allTriangles[triangle].points[i];

                    neighbours.Add(p);
                    nodeNeighbours[p].Add(nodeIndex);
                }
            }

            nodes[nodeIndex] = node;
            nodePositions[nodeIndex] = position;
            nodeNeighbours[nodeIndex] = neighbours;
            return triangle;
        }

        float GetDistance(int nodeA, int nodeB)
        {
            Float2 centerA = nodePositions[nodeA];
            Float2 centerB = nodePositions[nodeB];

            return (centerA - centerB).Length();
        }
    }
}
