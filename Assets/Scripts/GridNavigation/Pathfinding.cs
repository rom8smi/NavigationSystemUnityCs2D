using System.Collections.Generic;
using GenericCode;

namespace GridNavigation
{
    public class Pathfinding
    {
        Grid grid;
        Heap openSet;
        public PathfindingNode[] nodes;
        public bool smoothPath;
        public int numberOfSmoothIterations;

        public Pathfinding(Grid g)
        {
            grid = g;
            openSet = new Heap();

            nodes = new PathfindingNode[grid.gridSizeX * grid.gridSizeY];

            for (int i = 0; i < nodes.Length; i++)
            {
                nodes[i] = new PathfindingNode
                {
                    gCost = 0,
                    hCost = 0,
                    parent = -1,
                    heapIndex = -1,
                    isInClosedSet = false
                };
            }
        }

        public Path FindPath(Float2 startPos, Float2 targetPos)
        {
            Path path = FindPathToExactTarget(startPos, targetPos);
            if (!path.success)
            {
                targetPos = grid.GetWorldPosition(grid.nodes[path.lowestHCostNode].gridX, grid.nodes[path.lowestHCostNode].gridY);
                path = FindPathToExactTarget(startPos, targetPos);
            }
            return path;
        }

        Path FindPathToExactTarget(Float2 startPos, Float2 targetPos)
        {
            GridNeighbours gridNeighbours = new GridNeighbours(8);
            bool pathSuccess = false;

            for (int i = 0; i < 2; i++)
            {
                startPos = VectorUtils.AdjustForBoundaries(startPos, grid.worldMinX, grid.worldMaxX, grid.worldMinY, grid.worldMaxY, 0.01f);
                targetPos = VectorUtils.AdjustForBoundaries(targetPos, grid.worldMinX, grid.worldMaxX, grid.worldMinY, grid.worldMaxY, 0.01f);

                startPos = grid.GetNearestWalkablePosition(startPos);
                targetPos = grid.GetNearestWalkablePosition(targetPos);
            }

            int startNode = grid.NodeFromWorldPoint(startPos);
            int targetNode = grid.NodeFromWorldPoint(targetPos);
            List<int> closedSet = new List<int>();

            if (!grid.nodes[startNode].walkable)
            {
                GenericCode.Debug.Log($"Start not walkable {startNode}");
            }
            if (!grid.nodes[targetNode].walkable)
            {
                GenericCode.Debug.Log($"Target not walkable {targetNode}");
            }

            int lowestHCostNode = startNode;
            int lowestHCost = int.MaxValue;

            if (grid.nodes[startNode].walkable && grid.nodes[targetNode].walkable)
            {
                openSet.Add(startNode, nodes);

                while (openSet.count > 0)
                {
                    int currentNode = openSet.RemoveFirst(nodes);
                    nodes[currentNode].isInClosedSet = true;

                    closedSet.Add(currentNode);

                    if (currentNode == targetNode)
                    {
                        pathSuccess = true;
                        break;
                    }

                    grid.GetNeighbours(gridNeighbours, currentNode);
                    int neighboursCount = gridNeighbours.count;

                    for (int i = 0; i < neighboursCount; i++)
                    {
                        int neighbour = gridNeighbours.neighbours[i];

                        if (!grid.nodes[neighbour].walkable || nodes[neighbour].isInClosedSet || !CanMoveDiagonally(currentNode, neighbour))
                        {
                            continue;
                        }

                        int newMovementCostToNeighbour = nodes[currentNode].gCost + GetDistance(currentNode, neighbour) + grid.nodes[neighbour].movementPenalty;
                        if (newMovementCostToNeighbour < nodes[neighbour].gCost || nodes[neighbour].heapIndex == -1)
                        {
                            nodes[neighbour].gCost = newMovementCostToNeighbour;
                            int hCost = GetDistance(neighbour, targetNode);
                            if (hCost < lowestHCost)
                            {
                                lowestHCostNode = neighbour;
                                lowestHCost = hCost;
                            }

                            nodes[neighbour].hCost = hCost;
                            nodes[neighbour].parent = currentNode;

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
                waypoints = RetracePath(startNode, targetNode, startPos, targetPos);
                if (smoothPath)
                {
                    SmoothPath(waypoints);
                }

                float mergeEpsilon = 0.3f * grid.nodeRadius;
                waypoints = SimplifyPath(waypoints, mergeEpsilon * mergeEpsilon);
                waypoints = ReversePath(waypoints);

                pathSuccess = waypoints.Count > 0;
            }

            for (int i = 0; i < closedSet.Count; i++)
            {
                int nodeIndex = closedSet[i];
                nodes[nodeIndex].isInClosedSet = false;
                nodes[nodeIndex].parent = -1;
                nodes[nodeIndex].heapIndex = -1;
            }

            openSet.Clear(nodes);

            return new Path
            {
                waypoints = waypoints,
                success = pathSuccess,
                lowestHCostNode = lowestHCostNode
            };
        }

        bool CanMoveDiagonally(int from, int to)
        {
            int xFrom = grid.nodes[from].gridX;
            int yFrom = grid.nodes[from].gridY;

            int xTo = grid.nodes[to].gridX;
            int yTo = grid.nodes[to].gridY;

            if (xFrom != xTo && yFrom != yTo)
            {
                int nodeIndex = grid.GetFlatIndex(xTo, yFrom);
                if (!grid.nodes[nodeIndex].walkable)
                {
                    return false;
                }

                nodeIndex = grid.GetFlatIndex(xFrom, yTo);
                if (!grid.nodes[nodeIndex].walkable)
                {
                    return false;
                }
            }

            return true;
        }

        List<Float2> RetracePath(int startNode, int endNode, Float2 startPos, Float2 targetPos)
        {
            List<Float2> waypoints = new List<Float2>();
            int currentNode = endNode;
            Float2 waypointPosition;

            while (currentNode != startNode)
            {
                waypointPosition = grid.GetWorldPosition(grid.nodes[currentNode].gridX, grid.nodes[currentNode].gridY);
                waypoints.Add(waypointPosition);
                currentNode = nodes[currentNode].parent;
            }

            waypointPosition = grid.GetWorldPosition(grid.nodes[startNode].gridX, grid.nodes[startNode].gridY);
            waypoints.Add(waypointPosition);

            if (waypoints.Count > 1)
            {
                waypoints[waypoints.Count - 1] = startPos;
                waypoints[0] = targetPos;
            }

            return waypoints;
        }

        // Based on https://stackoverflow.com/questions/68794127/simplifying-a-path-on-a-two-dimensional-grid
        List<Float2> SimplifyPath(List<Float2> waypoints, float epsilon)
        {
            if (waypoints.Count < 3)
            {
                return waypoints;
            }

            List<Float2> simplifiedWaypoints = new List<Float2>();
            simplifiedWaypoints.Add(waypoints[0]);
            float dx = 0;
            float dy = 0;

            for (int i = 0; i < waypoints.Count - 1; i++)
            {
                Float2 p0 = waypoints[i];
                Float2 p1 = waypoints[i + 1];

                float x = p1.x - p0.x;
                float y = p1.y - p0.y;

                if (i > 0 && MathUtils.Abs(dx * y - dy * x) > epsilon)
                {
                    simplifiedWaypoints.Add(p0);
                }

                dx = x;
                dy = y;
            }

            return simplifiedWaypoints;
        }

        void SmoothPath(List<Float2> waypoints)
        {
            if (waypoints.Count < 3)
            {
                return;
            }

            for (int j = 0; j < numberOfSmoothIterations; j++)
            {
                for (int i = 0; i < waypoints.Count - 2; i++)
                {
                    Float2 start = waypoints[i];
                    Float2 middle = waypoints[i + 1];
                    Float2 end = waypoints[i + 2];

                    Float2 projectedPoint = MathUtils.FindNearestPointOnLine(start, end - start, middle);
                    Float2 shiftDirection = projectedPoint - middle;

                    if (shiftDirection.LengthSquared() > 0.0f)
                    {
                        float amountToShift = MathUtils.Min(grid.nodeRadius, shiftDirection.Length());
                        Float2 newMiddle = middle + shiftDirection.Normalized() * amountToShift;

                        int middleNode = grid.NodeFromWorldPoint(middle);
                        int newMiddleNode = grid.NodeFromWorldPoint(newMiddle);

                        if (grid.nodes[middleNode].movementPenalty >= grid.nodes[newMiddleNode].movementPenalty)
                        {
                            waypoints[i + 1] = newMiddle;
                        }
                    }
                }
            }
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

        int GetDistance(int nodeA, int nodeB)
        {
            int dstX = MathUtils.Abs(grid.nodes[nodeA].gridX - grid.nodes[nodeB].gridX);
            int dstY = MathUtils.Abs(grid.nodes[nodeA].gridY - grid.nodes[nodeB].gridY);

            if (dstX > dstY)
            {
                return 14 * dstY + 10 * (dstX - dstY);
            }
            return 14 * dstX + 10 * (dstY - dstX);
        }
    }
}
