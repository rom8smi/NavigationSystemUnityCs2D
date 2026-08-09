using System.Collections.Generic;
using DelaunatorSharp;
using GenericCode;

namespace TriangulationNavigation
{
    public class TriangulationGridSearch
    {
        AABB triangulationBounds;
        float nodeDiameterX;
        float nodeDiameterY;
        int resolution;

        List<int> trianglesStart;
        List<int> trianglesCount;
        List<int> trianglesByCells;

        public void Create(
            AABB p_triangulationBounds,
            int p_resolution,
            Delaunator p_delaunator,
            List<Float2> p_delaunatorPoints,
            int p_trianglesCount,
            List<bool> p_trianglesMask)
        {
            resolution = p_resolution;
            triangulationBounds = p_triangulationBounds;

            int resolutionSqr = resolution * resolution;

            List<List<int>> trianglesGrid = new List<List<int>>();
            trianglesGrid.Resize(resolutionSqr);

            for (int i = 0; i < resolutionSqr; i++)
            {
                trianglesGrid[i] = new List<int>();
            }

            nodeDiameterX = (triangulationBounds.maxX - triangulationBounds.minX) / resolution;
            nodeDiameterY = (triangulationBounds.maxY - triangulationBounds.minY) / resolution;

            int cellsCount = 0;

            for (int i = 0; i < p_trianglesCount; i++)
            {
                if (p_trianglesMask[i])
                {
                    int e0 = 3 * i;
                    int tp1 = p_delaunator.triangles[e0];
                    int tp2 = p_delaunator.triangles[e0 + 1];
                    int tp3 = p_delaunator.triangles[e0 + 2];

                    Float2 p1 = p_delaunatorPoints[tp1];
                    Float2 p2 = p_delaunatorPoints[tp2];
                    Float2 p3 = p_delaunatorPoints[tp3];

                    int ip1x = GetGridIndexX(p1);
                    int ip2x = GetGridIndexX(p2);
                    int ip3x = GetGridIndexX(p3);

                    int ip1y = GetGridIndexY(p1);
                    int ip2y = GetGridIndexY(p2);
                    int ip3y = GetGridIndexY(p3);

                    int ip1 = ip1x * resolution + ip1y;
                    int ip2 = ip2x * resolution + ip2y;
                    int ip3 = ip3x * resolution + ip3y;

                    cellsCount++;
                    trianglesGrid[ip1].Add(i);

                    if (ip2 != ip1)
                    {
                        cellsCount++;
                        trianglesGrid[ip2].Add(i);
                    }
                    if (ip3 != ip1 && ip3 != ip2)
                    {
                        cellsCount++;
                        trianglesGrid[ip3].Add(i);
                    }

                    int minX = MathUtils.Min(ip1x, ip2x);
                    minX = MathUtils.Min(minX, ip3x);
                    int maxX = MathUtils.Max(ip1x, ip2x);
                    maxX = MathUtils.Max(maxX, ip3x);

                    int minY = MathUtils.Min(ip1y, ip2y);
                    minY = MathUtils.Min(minY, ip3y);
                    int maxY = MathUtils.Max(ip1y, ip2y);
                    maxY = MathUtils.Max(maxY, ip3y);

                    for (int ix = minX; ix <= maxX; ix++)
                    {
                        for (int iy = minY; iy <= maxY; iy++)
                        {
                            int ik = ix * resolution + iy;

                            if (ik != ip1 && ik != ip2 && ik != ip3)
                            {
                                Float2 cellPos00 = new Float2(nodeDiameterX * ix + triangulationBounds.minX, nodeDiameterY * iy + triangulationBounds.minY);
                                Float2 cellPos01 = new Float2(nodeDiameterX * ix + triangulationBounds.minX, nodeDiameterY * (iy + 1) + triangulationBounds.minY);
                                Float2 cellPos11 = new Float2(nodeDiameterX * (ix + 1) + triangulationBounds.minX, nodeDiameterY * (iy + 1) + triangulationBounds.minY);
                                Float2 cellPos10 = new Float2(nodeDiameterX * (ix + 1) + triangulationBounds.minX, nodeDiameterY * iy + triangulationBounds.minY);

                                if (
                                    VectorUtils.AreLineSegmentsIntersecting(p1, p2, cellPos00, cellPos01) ||
                                    VectorUtils.AreLineSegmentsIntersecting(p2, p3, cellPos00, cellPos01) ||
                                    VectorUtils.AreLineSegmentsIntersecting(p3, p1, cellPos00, cellPos01) ||

                                    VectorUtils.AreLineSegmentsIntersecting(p1, p2, cellPos01, cellPos11) ||
                                    VectorUtils.AreLineSegmentsIntersecting(p2, p3, cellPos01, cellPos11) ||
                                    VectorUtils.AreLineSegmentsIntersecting(p3, p1, cellPos01, cellPos11) ||

                                    VectorUtils.AreLineSegmentsIntersecting(p1, p2, cellPos11, cellPos10) ||
                                    VectorUtils.AreLineSegmentsIntersecting(p2, p3, cellPos11, cellPos10) ||
                                    VectorUtils.AreLineSegmentsIntersecting(p3, p1, cellPos11, cellPos10) ||

                                    VectorUtils.AreLineSegmentsIntersecting(p1, p2, cellPos10, cellPos00) ||
                                    VectorUtils.AreLineSegmentsIntersecting(p2, p3, cellPos10, cellPos00) ||
                                    VectorUtils.AreLineSegmentsIntersecting(p3, p1, cellPos10, cellPos00)
                                )
                                {
                                    cellsCount++;
                                    trianglesGrid[ik].Add(i);
                                }
                                else if (VectorUtils.PointInTriangle(cellPos00, p1, p2, p3))
                                {
                                    cellsCount++;
                                    trianglesGrid[ik].Add(i);
                                }
                            }
                        }
                    }
                }
            }

            trianglesStart = new List<int>();
            trianglesCount = new List<int>();
            trianglesByCells = new List<int>();

            trianglesStart.Resize(resolutionSqr);
            trianglesCount.Resize(resolutionSqr);
            trianglesByCells.Resize(cellsCount);

            cellsCount = 0;

            for (int i = 0; i < resolutionSqr; i++)
            {
                int start = cellsCount;
                int count = trianglesGrid[i].Count;

                trianglesStart[i] = start;
                trianglesCount[i] = count;

                for (int j = 0; j < count; j++)
                {
                    trianglesByCells[cellsCount] = trianglesGrid[i][j];
                    cellsCount++;
                }
            }
        }

        public int FindTriangleForPoint(
            Float2 p_position,
            Delaunator p_delaunator,
            List<Float2> p_delaunatorPoints)
        {
            int ix = GetGridIndexX(p_position);
            int iy = GetGridIndexY(p_position);

            if (ix > -1 && ix < resolution && iy > -1 && iy < resolution)
            {
                int k = ix * resolution + iy;

                int start = trianglesStart[k];
                int count = trianglesCount[k];

                for (int i = 0; i < count; i++)
                {
                    int triangle = trianglesByCells[start + i];

                    int e0 = 3 * triangle;
                    int ip1 = p_delaunator.triangles[e0];
                    int ip2 = p_delaunator.triangles[e0 + 1];
                    int ip3 = p_delaunator.triangles[e0 + 2];

                    if (VectorUtils.PointInTriangle(p_position, p_delaunatorPoints[ip1], p_delaunatorPoints[ip2], p_delaunatorPoints[ip3]))
                    {
                        return triangle;
                    }
                }
            }

            return -1;
        }

        int GetGridIndexX(Float2 position)
        {
            return (int)((position.x - triangulationBounds.minX) / nodeDiameterX);
        }

        int GetGridIndexY(Float2 position)
        {
            return (int)((position.y - triangulationBounds.minY) / nodeDiameterY);
        }
    }
}
