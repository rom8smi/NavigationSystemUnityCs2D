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
        List<int> nodeEdgeRefs;
        List<int> nodeEdgeRefsInverted;
        List<float> additionalCosts;
        List<bool> additionalCostsModified;
        List<int> addedAdditionalCosts;
        int nodesCount;
        bool useIterations;
        float costIncrement;
        bool triangleEdgesMode;

        public Pathfinding()
        {
            openSet = new Heap();
            closedSet = new List<int>();
            nodes = new List<PathfindingNode>();
            nodePositions = new List<Float2>();
            nodeNeighbours = new List<List<int>>();
            nodeEdgeRefs = new List<int>();
            nodeEdgeRefsInverted = new List<int>();
            additionalCosts = new List<float>();
            additionalCostsModified = new List<bool>();
            addedAdditionalCosts = new List<int>();

            triangleEdgesMode = true;
        }

        public void CreateNodes(NavMesh navMesh)
        {
            if (triangleEdgesMode)
            {
                CreateNodesEdges(navMesh);
            }
            else
            {
                CreateNodesCorners(navMesh);
            }
        }

        public void CreateNodesCorners(NavMesh navMesh)
        {
            useIterations = true;
            costIncrement = 1.0f;
            openSet.Clear(nodes);
            nodes.Clear();
            nodePositions.Clear();
            nodeNeighbours.Clear();
            additionalCosts.Clear();
            additionalCostsModified.Clear();
            nodeEdgeRefsInverted.Clear();
            nodeEdgeRefs.Clear();

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

        public void CreateNodesEdges(NavMesh navMesh)
        {
            useIterations = false;
            costIncrement = 1.0f;
            openSet.Clear(nodes);
            nodes.Clear();
            nodePositions.Clear();
            nodeNeighbours.Clear();
            additionalCosts.Clear();
            additionalCostsModified.Clear();
            nodeEdgeRefsInverted.Clear();

            nodeEdgeRefs.Resize(navMesh.delaunator.trianglesLen);

            for (int i = 0; i < navMesh.delaunator.trianglesLen; i++)
            {
                nodeEdgeRefs[i] = -1;
            }

            List<int> walkableIndices = new List<int>();
            List<bool> repetitive = new List<bool>();
            List<int> nodeIndicesInTriangle = new List<int>();

            int trianglesCount = navMesh.delaunator.trianglesLen / 3;

            for (int t = 0; t < trianglesCount; t++)
            {
                walkableIndices.Clear();
                repetitive.Clear();

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
                            if (e > opposite)
                            {
                                repetitive.Add(true);
                            }
                            else
                            {
                                repetitive.Add(false);
                            }
                        }
                    }
                }

                int walkableIndicesCount = walkableIndices.Count;
                nodeIndicesInTriangle.Resize(walkableIndicesCount);

                for (int i = 0; i < walkableIndicesCount; i++)
                {
                    if (repetitive[i])
                    {
                        int e = t * 3 + walkableIndices[i];
                        int opposite = navMesh.delaunator.halfedges[e];

                        nodeIndicesInTriangle[i] = nodeEdgeRefs[opposite];
                    }
                    else
                    {
                        int e = t * 3 + walkableIndices[i];
                        int opposite = navMesh.delaunator.halfedges[e];

                        int p = navMesh.delaunator.triangles[e];
                        int q = navMesh.delaunator.triangles[Delaunator.NextHalfedge(e)];

                        Float2 center = (navMesh.allPoints[p] + navMesh.allPoints[q]) * 0.5f;

                        int currentNodesCount = nodes.Count;
                        nodeIndicesInTriangle[i] = currentNodesCount;

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
                        nodeEdgeRefsInverted.Add(e);
                        nodeEdgeRefs[e] = currentNodesCount;
                        nodeEdgeRefs[opposite] = currentNodesCount;
                    }
                }

                for (int i = 0; i < walkableIndicesCount; i++)
                {
                    for (int j = i + 1; j < walkableIndicesCount; j++)
                    {
                        int nodeA = nodeIndicesInTriangle[i];
                        int nodeB = nodeIndicesInTriangle[j];
                        nodeNeighbours[nodeA].Add(nodeB);
                        nodeNeighbours[nodeB].Add(nodeA);
                    }
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
                nodeEdgeRefsInverted.Add(-1);
            }
        }

        public Path FindPath(Float2 startPos, Float2 targetPos, NavMesh navMesh)
        {
            Path path = FindPathWithOrWithoutIterations(startPos, targetPos, navMesh);

            if (!path.success && path.lowestHCostNode < nodesCount)
            {
                Float2 newTargetPos = nodePositions[path.lowestHCostNode];

                if (triangleEdgesMode)
                {
                    int e = nodeEdgeRefsInverted[path.lowestHCostNode];
                    if (e != -1)
                    {
                        int p = navMesh.delaunator.triangles[e];
                        int q = navMesh.delaunator.triangles[Delaunator.NextHalfedge(e)];

                        Float2 newTargetPosP = navMesh.FindNearestObstacleHullEdgePointToTarget(p, newTargetPos, targetPos);
                        Float2 newTargetPosQ = navMesh.FindNearestObstacleHullEdgePointToTarget(q, newTargetPos, targetPos);

                        float distanceSqrP = (newTargetPosP - targetPos).LengthSquared();
                        float distanceSqrQ = (newTargetPosQ - targetPos).LengthSquared();

                        if (distanceSqrP < distanceSqrQ)
                        {
                            newTargetPos = newTargetPosP;
                        }
                        else
                        {
                            newTargetPos = newTargetPosQ;
                        }
                    }
                }

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

            if (!triangleEdgesMode && nodesCount != navMesh.allPoints.Count)
            {
                Debug.Log($"Pathfinding nodes and triangulation points count does not match: {nodesCount}, {navMesh.allPoints.Count}");
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

            if (!triangleEdgesMode && nodesCount != navMesh.allPoints.Count)
            {
                Debug.Log($"Pathfinding nodes and triangulation points count does not match: {nodesCount}, {navMesh.allPoints.Count}");
            }
            Path path = FindPathToExactTarget(startPos, targetPos, navMesh);
            ClearPathSearch();
            return path;
        }

        Path FindPathToExactTarget(Float2 startPos, Float2 targetPos, NavMesh navMesh)
        {
            int startNode = nodesCount;
            int targetNode = nodesCount + 1;

            int startTriangle;
            int targetTriangle;

            if (triangleEdgesMode)
            {
                startTriangle = UpdatePositionNodeEdges(startPos, navMesh, startNode);
                targetTriangle = UpdatePositionNodeEdges(targetPos, navMesh, targetNode);
            }
            else
            {
                startTriangle = UpdatePositionNodeCorners(startPos, navMesh, startNode);
                targetTriangle = UpdatePositionNodeCorners(targetPos, navMesh, targetNode);
            }

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
                List<int> waypointIndices = new List<int>();
                RetracePath(waypoints, waypointIndices, startNode, targetNode);

                if (triangleEdgesMode)
                {
                    SimplifyPathEdges(waypoints, waypointIndices, navMesh);
                }
                else
                {
                    SimplifyPathCorners(waypoints, navMesh);
                }

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

        void RetracePath(
            List<Float2> waypoints,
            List<int> waypointIndices,
            int startNode,
            int endNode)
        {
            int currentNode = endNode;
            Float2 waypointPosition;

            while (currentNode != startNode)
            {
                waypointPosition = nodePositions[currentNode];
                waypoints.Add(waypointPosition);
                waypointIndices.Add(currentNode);
                currentNode = nodes[currentNode].parent;
            }

            waypointPosition = nodePositions[startNode];
            waypoints.Add(waypointPosition);
            waypointIndices.Add(startNode);
        }

        void SimplifyPathEdges(
            List<Float2> waypoints,
            List<int> waypointIndices,
            NavMesh navMesh)
        {
            int waypointIndicesCount = waypointIndices.Count;
            if (waypointIndicesCount < 3)
            {
                return;
            }

            List<int> leftPortalsEdgeIndices = new List<int>();
            List<int> rightPortalsEdgeIndices = new List<int>();

            leftPortalsEdgeIndices.Resize(waypointIndicesCount);
            rightPortalsEdgeIndices.Resize(waypointIndicesCount);

            leftPortalsEdgeIndices[0] = -1;
            rightPortalsEdgeIndices[0] = -1;

            leftPortalsEdgeIndices[waypointIndicesCount - 1] = -1;
            rightPortalsEdgeIndices[waypointIndicesCount - 1] = -1;

            for (int i = 0; i < waypointIndices.Count - 2; i++)
            {
                int previousWaypointIndex = waypointIndices[i];
                int currentWaypointIndex = waypointIndices[i + 1];
                int nextWaypointIndex = waypointIndices[i + 2];

                int edgeIndex = nodeEdgeRefsInverted[currentWaypointIndex];

                Float2 pathDirectionA = (nodePositions[currentWaypointIndex] - nodePositions[previousWaypointIndex]).Normalized();
                Float2 pathDirectionB = (nodePositions[nextWaypointIndex] - nodePositions[currentWaypointIndex]).Normalized();

                Float2 pathDirection = (pathDirectionA + pathDirectionB) * 0.5f;

                int p = navMesh.delaunator.triangles[edgeIndex];
                int q = navMesh.delaunator.triangles[Delaunator.NextHalfedge(edgeIndex)];

                Float2 perpendicularDirection = navMesh.allPoints[p] - nodePositions[currentWaypointIndex];

                if (pathDirection.Cross(perpendicularDirection) < 0.0f)
                {
                    leftPortalsEdgeIndices[i + 1] = p;
                    rightPortalsEdgeIndices[i + 1] = q;
                }
                else
                {
                    leftPortalsEdgeIndices[i + 1] = q;
                    rightPortalsEdgeIndices[i + 1] = p;
                }
            }

            List<Float2> simplifiedWaypoints = new List<Float2>();

            SimplifyPathEdgesInner(
                waypoints,
                leftPortalsEdgeIndices,
                rightPortalsEdgeIndices,
                navMesh,
                simplifiedWaypoints);

            waypoints.Clear();
            for (int i = 0; i < simplifiedWaypoints.Count; i++)
            {
                waypoints.Add(simplifiedWaypoints[i]);
            }
        }

        void SimplifyPathEdgesInner(
            List<Float2> waypoints,
            List<int> leftPortalsEdgeIndices,
            List<int> rightPortalsEdgeIndices,
            NavMesh navMesh,
            List<Float2> simplifiedWaypoints)
        {
            int totalPoints = waypoints.Count;
            Float2 apexPosition = waypoints[0];

            simplifiedWaypoints.Add(apexPosition);

            int apexIndex = 0;
            int leftIndex = 0;
            int rightIndex = 0;

            int apexCornerIndex = -1;
            int lastAddedCornerIndex = -1;

            for (int i = 1; i < totalPoints; i++)
            {
                int currentLeftCornerIndex = leftPortalsEdgeIndices[i];
                int currentRightCornerIndex = rightPortalsEdgeIndices[i];

                int activeLeftCornerIndex = leftPortalsEdgeIndices[leftIndex];
                int activeRightCornerIndex = rightPortalsEdgeIndices[rightIndex];

                Float2 currentRightPosition = GetPortalPosition(i, currentRightCornerIndex, waypoints, navMesh);
                Float2 activeRightPosition = GetPortalPosition(rightIndex, activeRightCornerIndex, waypoints, navMesh);

                bool isRightTightening = (rightIndex == apexIndex) ||
                                         (Orient2D(apexPosition, activeRightPosition, currentRightPosition) <= 0.0f);

                if (isRightTightening)
                {
                    bool sameAsLeft = (currentRightCornerIndex != -1 && currentRightCornerIndex == activeLeftCornerIndex);
                    bool sameAsApex = (currentRightCornerIndex != -1 && apexCornerIndex != -1 && currentRightCornerIndex == apexCornerIndex);

                    Float2 activeLeftPosition = GetPortalPosition(leftIndex, activeLeftCornerIndex, waypoints, navMesh);

                    if (rightIndex == apexIndex || sameAsLeft || sameAsApex ||
                        Orient2D(apexPosition, activeLeftPosition, currentRightPosition) >= 0.0f)
                    {
                        rightIndex = i;
                    }
                    else
                    {
                        int leftCornerIndex = activeLeftCornerIndex;

                        if (leftCornerIndex < 0 || leftCornerIndex != lastAddedCornerIndex)
                        {
                            simplifiedWaypoints.Add(activeLeftPosition);
                            lastAddedCornerIndex = leftCornerIndex;
                        }

                        apexPosition = activeLeftPosition;
                        apexIndex = leftIndex;
                        apexCornerIndex = leftCornerIndex;

                        leftIndex = apexIndex;
                        rightIndex = apexIndex;

                        i = apexIndex;
                        continue;
                    }
                }

                Float2 currentLeftPosition = GetPortalPosition(i, currentLeftCornerIndex, waypoints, navMesh);
                Float2 activeLeftPositionCurrent = GetPortalPosition(leftIndex, activeLeftCornerIndex, waypoints, navMesh);

                bool isLeftTightening = (leftIndex == apexIndex) ||
                                        (Orient2D(apexPosition, activeLeftPositionCurrent, currentLeftPosition) >= 0.0f);

                if (isLeftTightening)
                {
                    bool sameAsRight = (currentLeftCornerIndex != -1 && currentLeftCornerIndex == activeRightCornerIndex);
                    bool sameAsApex = (currentLeftCornerIndex != -1 && apexCornerIndex != -1 && currentLeftCornerIndex == apexCornerIndex);

                    Float2 activeRightPositionCurrent = GetPortalPosition(rightIndex, activeRightCornerIndex, waypoints, navMesh);

                    if (leftIndex == apexIndex || sameAsRight || sameAsApex ||
                        Orient2D(apexPosition, activeRightPositionCurrent, currentLeftPosition) <= 0.0f)
                    {
                        leftIndex = i;
                    }
                    else
                    {
                        int rightCornerIndex = activeRightCornerIndex;

                        if (rightCornerIndex < 0 || rightCornerIndex != lastAddedCornerIndex)
                        {
                            simplifiedWaypoints.Add(activeRightPositionCurrent);
                            lastAddedCornerIndex = rightCornerIndex;
                        }

                        apexPosition = activeRightPositionCurrent;
                        apexIndex = rightIndex;
                        apexCornerIndex = rightCornerIndex;

                        leftIndex = apexIndex;
                        rightIndex = apexIndex;

                        i = apexIndex;
                        continue;
                    }
                }
            }

            if (apexIndex < totalPoints - 1)
            {
                simplifiedWaypoints.Add(waypoints[totalPoints - 1]);
            }
        }

        Float2 GetPortalPosition(int index, int cornerIndex, List<Float2> waypoints, NavMesh navMesh)
        {
            return cornerIndex >= 0 ? navMesh.allPoints[cornerIndex] : waypoints[index];
        }

        float Orient2D(Float2 a, Float2 b, Float2 c)
        {
            Float2 ba = b - a;
            Float2 ca = c - a;
            return ba.Cross(ca);
        }

        void SimplifyPathCorners(List<Float2> waypoints, NavMesh navMesh)
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

        int UpdatePositionNodeEdges(Float2 position, NavMesh navMesh, int nodeIndex)
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
                for (int i = 0; i < 3; i++)
                {
                    int e = 3 * triangle + i;
                    int opposite = navMesh.delaunator.halfedges[e];

                    if (opposite != -1)
                    {
                        if (navMesh.trianglesWalkability[Delaunator.TriangleOfEdge(e)] == -1 &&
                        navMesh.trianglesWalkability[Delaunator.TriangleOfEdge(opposite)] == -1)
                        {
                            neighbours.Add(nodeEdgeRefs[e]);
                            nodeNeighbours[nodeEdgeRefs[e]].Add(nodeIndex);
                        }
                    }
                }
            }

            nodes[nodeIndex] = node;
            nodePositions[nodeIndex] = position;
            nodeNeighbours[nodeIndex] = neighbours;
            return triangle;
        }

        int UpdatePositionNodeCorners(Float2 position, NavMesh navMesh, int nodeIndex)
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
