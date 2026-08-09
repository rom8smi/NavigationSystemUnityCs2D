using System.Collections.Generic;
using GenericCode;

namespace GridNavigation
{
    public class Grid
    {
        public Obstacle[] obstacles;
        public PenaltyObstacle[] penaltyObstacles;

        public Float2 gridWorldSize;
        public float nodeRadius;
        public int obstacleProximityPenalty = 10;

        public Node[] nodes;
        public List<NodeEdge> nodeEdges;

        public float nodeDiameter;
        public int gridSizeX;
        public int gridSizeY;
        public Float2 gridWorldOrigin;

        public float worldMinX;
        public float worldMaxX;
        public float worldMinY;
        public float worldMaxY;

        public void Setup()
        {
            nodeDiameter = nodeRadius * 2;
            gridSizeX = MathUtils.RoundToInt(gridWorldSize.x / nodeDiameter);
            gridSizeY = MathUtils.RoundToInt(gridWorldSize.y / nodeDiameter);

            worldMinX = gridWorldOrigin.x;
            worldMaxX = worldMinX + gridWorldSize.x;
            worldMinY = gridWorldOrigin.y;
            worldMaxY = worldMinY + gridWorldSize.y;

            CreateGrid();
            // BlurPenaltyMap(3);
        }

        void CreateGrid()
        {
            nodes = new Node[gridSizeX * gridSizeY];

            for (int x = 0; x < gridSizeX; x++)
            {
                for (int y = 0; y < gridSizeY; y++)
                {
                    int gridIndex = GetFlatIndex(x, y);
                    nodes[gridIndex].walkable = true;
                    nodes[gridIndex].movementPenalty = 0;
                }
            }

            for (int i = 0; i < obstacles.Length; i++)
            {
                Float2 center = obstacles[i].center;
                Float2 size = obstacles[i].size;

                int minX = NodeXFromWorldPoint(center.x - 0.5f * size.x);
                int maxX = NodeXFromWorldPoint(center.x + 0.5f * size.x);
                int minY = NodeYFromWorldPoint(center.y - 0.5f * size.y);
                int maxY = NodeYFromWorldPoint(center.y + 0.5f * size.y);

                for (int x = minX; x <= maxX; x++)
                {
                    if (x >= 0 && x < gridSizeX)
                    {
                        for (int y = minY; y <= maxY; y++)
                        {
                            if (y >= 0 && y < gridSizeY)
                            {
                                int gridIndex = GetFlatIndex(x, y);
                                nodes[gridIndex].walkable = false;
                            }
                        }
                    }
                }
            }

            for (int i = 0; i < penaltyObstacles.Length; i++)
            {
                Float2 center = penaltyObstacles[i].center;
                Float2 size = penaltyObstacles[i].size;

                int minX = NodeXFromWorldPoint(center.x - 0.5f * size.x);
                int maxX = NodeXFromWorldPoint(center.x + 0.5f * size.x);
                int minY = NodeYFromWorldPoint(center.y - 0.5f * size.y);
                int maxY = NodeYFromWorldPoint(center.y + 0.5f * size.y);

                for (int x = minX; x <= maxX; x++)
                {
                    if (x >= 0 && x < gridSizeX)
                    {
                        for (int y = minY; y <= maxY; y++)
                        {
                            if (y >= 0 && y < gridSizeY)
                            {
                                int gridIndex = GetFlatIndex(x, y);
                                nodes[gridIndex].movementPenalty = penaltyObstacles[i].penalty;
                            }
                        }
                    }
                }
            }

            for (int x = 0; x < gridSizeX; x++)
            {
                for (int y = 0; y < gridSizeY; y++)
                {
                    int gridIndex = GetFlatIndex(x, y);

                    if (!nodes[gridIndex].walkable)
                    {
                        nodes[gridIndex].movementPenalty = obstacleProximityPenalty;
                    }

                    nodes[gridIndex].gridX = x;
                    nodes[gridIndex].gridY = y;
                    nodes[gridIndex].gridIndex = gridIndex;

                    nodes[gridIndex].nearestWalkableEdgeLeft = -1;
                    nodes[gridIndex].nearestWalkableEdgeRight = -1;
                    nodes[gridIndex].nearestWalkableEdgeBottom = -1;
                    nodes[gridIndex].nearestWalkableEdgeTop = -1;
                }
            }

            CreateNodeEdges();
        }

        void CreateNodeEdges()
        {
            nodeEdges = new List<NodeEdge>();

            for (int x = 0; x < gridSizeX; x++)
            {
                for (int y = 0; y < gridSizeY; y++)
                {
                    int gridIndex = GetFlatIndex(x, y);

                    if (x + 1 < gridSizeX)
                    {
                        int gridIndexRight = GetFlatIndex(x + 1, y);

                        int walkDirection = 0;
                        if (nodes[gridIndex].walkable && !nodes[gridIndexRight].walkable)
                        {
                            walkDirection = -1;
                        }
                        if (!nodes[gridIndex].walkable && nodes[gridIndexRight].walkable)
                        {
                            walkDirection = 1;
                        }

                        Float2 center = GetWorldPosition(x, y) + new Float2(nodeRadius, 0.0f);
                        Float2 start = center + new Float2(0.0f, -nodeRadius);
                        Float2 end = center + new Float2(0.0f, nodeRadius);

                        nodeEdges.Add(new NodeEdge
                        {
                            nodeA = gridIndex,
                            nodeB = gridIndexRight,
                            walkDirection = walkDirection,
                            center = center,
                            start = start,
                            end = end,
                            isVertical = false
                        });
                    }

                    if (y + 1 < gridSizeY)
                    {
                        int gridIndexUp = GetFlatIndex(x, y + 1);

                        int walkDirection = 0;
                        if (nodes[gridIndex].walkable && !nodes[gridIndexUp].walkable)
                        {
                            walkDirection = -1;
                        }
                        if (!nodes[gridIndex].walkable && nodes[gridIndexUp].walkable)
                        {
                            walkDirection = 1;
                        }

                        Float2 center = GetWorldPosition(x, y) + new Float2(0.0f, nodeRadius);
                        Float2 start = center + new Float2(-nodeRadius, 0.0f);
                        Float2 end = center + new Float2(nodeRadius, 0.0f);

                        nodeEdges.Add(new NodeEdge
                        {
                            nodeA = gridIndex,
                            nodeB = gridIndexUp,
                            walkDirection = walkDirection,
                            center = center,
                            start = start,
                            end = end,
                            isVertical = true
                        });
                    }
                }
            }

            List<int> boundaryEdgeIndices = new List<int>();
            List<Float2> boundaryEdgePositions = new List<Float2>();

            for (int i = 0; i < nodeEdges.Count; i++)
            {
                if (nodeEdges[i].walkDirection != 0)
                {
                    boundaryEdgeIndices.Add(i);
                    boundaryEdgePositions.Add(nodeEdges[i].center);
                }
            }

            if (boundaryEdgePositions.Count > 0)
            {
                KDTree2D boundaryEdgesKdTree = KDTree2D.MakeFromPoints(boundaryEdgePositions.ToArray());
                float distanceFromCenter = 0.9f * nodeRadius;

                for (int i = 0; i < nodes.Length; i++)
                {
                    if (!nodes[i].walkable)
                    {
                        Float2 center = GetWorldPosition(i);
                        nodes[i].nearestWalkableEdgeLeft = boundaryEdgeIndices[boundaryEdgesKdTree.FindNearest(center + new Float2(-distanceFromCenter, 0.0f))];
                        nodes[i].nearestWalkableEdgeRight = boundaryEdgeIndices[boundaryEdgesKdTree.FindNearest(center + new Float2(distanceFromCenter, 0.0f))];
                        nodes[i].nearestWalkableEdgeBottom = boundaryEdgeIndices[boundaryEdgesKdTree.FindNearest(center + new Float2(0.0f, -distanceFromCenter))];
                        nodes[i].nearestWalkableEdgeTop = boundaryEdgeIndices[boundaryEdgesKdTree.FindNearest(center + new Float2(0.0f, distanceFromCenter))];
                    }
                }
            }
        }

        public Float2 GetNearestWalkablePosition(Float2 position)
        {
            int nodeIndex = NodeFromWorldPoint(position);

            if (nodeIndex < 0 || nodeIndex >= nodes.Length)
            {
                UnityEngine.Debug.Log($"{nodeIndex} {nodes.Length}");
            }

            if (nodes[nodeIndex].walkable)
            {
                return position;
            }

            Float2 center = GetWorldPosition(nodeIndex);

            List<Float2> edges = new List<Float2>();
            edges.Add(new Float2(center.x - nodeRadius, center.y));
            edges.Add(new Float2(center.x + nodeRadius, center.y));
            edges.Add(new Float2(center.x, center.y - nodeRadius));
            edges.Add(new Float2(center.x, center.y + nodeRadius));

            int minIndex = -1;
            float minSqrDistance = float.MaxValue;

            for (int i = 0; i < edges.Count; i++)
            {
                float currentSqrDistance = (position - edges[i]).LengthSquared();
                if (currentSqrDistance < minSqrDistance)
                {
                    minIndex = i;
                    minSqrDistance = currentSqrDistance;
                }
            }

            int edgeIndex = -1;
            if (minIndex == 0)
            {
                edgeIndex = nodes[nodeIndex].nearestWalkableEdgeLeft;
            }
            if (minIndex == 1)
            {
                edgeIndex = nodes[nodeIndex].nearestWalkableEdgeRight;
            }
            if (minIndex == 2)
            {
                edgeIndex = nodes[nodeIndex].nearestWalkableEdgeBottom;
            }
            if (minIndex == 3)
            {
                edgeIndex = nodes[nodeIndex].nearestWalkableEdgeTop;
            }

            position = MathUtils.FindNearestPointOnLineSegment(nodeEdges[edgeIndex].start, nodeEdges[edgeIndex].end, position);
            float epsilon = 0.01f;

            if (nodeEdges[edgeIndex].isVertical)
            {
                if (nodeEdges[edgeIndex].walkDirection == -1)
                {
                    position.y -= epsilon;
                }
                else if (nodeEdges[edgeIndex].walkDirection == 1)
                {
                    position.y += epsilon;
                }
            }
            else
            {
                if (nodeEdges[edgeIndex].walkDirection == -1)
                {
                    position.x -= epsilon;
                }
                else if (nodeEdges[edgeIndex].walkDirection == 1)
                {
                    position.x += epsilon;
                }
            }

            return position;
        }

        public Float2 GetWorldPosition(int nodeIndex)
        {
            return GetWorldPosition(nodes[nodeIndex].gridX, nodes[nodeIndex].gridY);
        }

        public Float2 GetWorldPosition(int x, int y)
        {
            return new Float2(gridWorldOrigin.x + x * nodeDiameter + nodeRadius, gridWorldOrigin.y + y * nodeDiameter + nodeRadius);
        }

        void BlurPenaltyMap(int blurSize)
        {
            int kernelSize = blurSize * 2 + 1;
            int kernelExtents = (kernelSize - 1) / 2;

            int[] penaltiesHorizontalPass = new int[gridSizeX * gridSizeY];
            int[] penaltiesVerticalPass = new int[gridSizeX * gridSizeY];

            for (int y = 0; y < gridSizeY; y++)
            {
                for (int x = -kernelExtents; x <= kernelExtents; x++)
                {
                    int sampleX = MathUtils.Clamp(x, 0, kernelExtents);
                    penaltiesHorizontalPass[GetFlatIndex(0, y)] += nodes[GetFlatIndex(sampleX, y)].movementPenalty;
                }

                for (int x = 1; x < gridSizeX; x++)
                {
                    int removeIndex = MathUtils.Clamp(x - kernelExtents, 0, gridSizeX);
                    int addIndex = MathUtils.Clamp(x + kernelExtents, 0, gridSizeX - 1);

                    penaltiesHorizontalPass[GetFlatIndex(x, y)] =
                        penaltiesHorizontalPass[GetFlatIndex(x - 1, y)] -
                        nodes[GetFlatIndex(removeIndex, y)].movementPenalty +
                        nodes[GetFlatIndex(addIndex, y)].movementPenalty;
                }
            }

            for (int x = 0; x < gridSizeX; x++)
            {
                for (int y = -kernelExtents; y <= kernelExtents; y++)
                {
                    int sampleY = MathUtils.Clamp(y, 0, kernelExtents);
                    penaltiesVerticalPass[GetFlatIndex(x, 0)] += penaltiesHorizontalPass[GetFlatIndex(x, sampleY)];
                }

                int bluredPenalty = MathUtils.RoundToInt((float)penaltiesVerticalPass[GetFlatIndex(x, 0)] / (kernelSize * kernelSize));
                nodes[GetFlatIndex(x, 0)].movementPenalty = bluredPenalty;

                for (int y = 1; y < gridSizeY; y++)
                {
                    int removeIndex = MathUtils.Clamp(y - kernelExtents, 0, gridSizeY);
                    int addIndex = MathUtils.Clamp(y + kernelExtents, 0, gridSizeY - 1);

                    penaltiesVerticalPass[GetFlatIndex(x, y)] =
                        penaltiesVerticalPass[GetFlatIndex(x, y - 1)] -
                        penaltiesHorizontalPass[GetFlatIndex(x, removeIndex)] +
                        penaltiesHorizontalPass[GetFlatIndex(x, addIndex)];
                    bluredPenalty = MathUtils.RoundToInt((float)penaltiesVerticalPass[GetFlatIndex(x, y)] / (kernelSize * kernelSize));
                    nodes[GetFlatIndex(x, y)].movementPenalty = bluredPenalty;
                }
            }
        }

        public void GetNeighbours(GridNeighbours gridNeighbours, int node)
        {
            gridNeighbours.Clear();

            for (int x = -1; x <= 1; x++)
            {
                for (int y = -1; y <= 1; y++)
                {
                    if (x == 0 && y == 0)
                    {
                        continue;
                    }

                    int checkX = nodes[node].gridX + x;
                    int checkY = nodes[node].gridY + y;

                    if (checkX >= 0 && checkX < gridSizeX && checkY >= 0 && checkY < gridSizeY)
                    {
                        gridNeighbours.Add(GetFlatIndex(checkX, checkY));
                    }
                }
            }
        }

        public int NodeXFromWorldPoint(float worldPositionX)
        {
            return (int)((worldPositionX - gridWorldOrigin.x) / nodeDiameter);
        }

        public int NodeYFromWorldPoint(float worldPositionY)
        {
            return (int)((worldPositionY - gridWorldOrigin.y) / nodeDiameter);
        }

        public int NodeFromWorldPoint(Float2 worldPosition)
        {
            int x = NodeXFromWorldPoint(worldPosition.x);
            int y = NodeYFromWorldPoint(worldPosition.y);
            return GetFlatIndex(x, y);
        }

        public int GetFlatIndex(int x, int y)
        {
            return x * gridSizeY + y;
        }
    }
}
