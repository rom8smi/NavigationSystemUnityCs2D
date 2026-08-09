using System.Collections.Generic;
using DelaunatorSharp;
using GenericCode;

namespace TriangulationNavigation
{
    public class TriangulationSearch
    {
        KDTree2D kdtree;
        List<int> trianglesGrid;

        AABB triangulationBounds;
        float nodeDiameterX;
        float nodeDiameterY;
        int resolution;

        public void Create(
            AABB p_triangulationBounds,
            int p_resolution,
            Delaunator p_delaunator,
            List<Float2> p_delaunatorPoints,
            List<DelaunatorSharp.Triangle> p_triangles,
            List<AABB> p_triangleBounds,
            List<Float2> p_triangleCentroids)
        {
            kdtree = KDTree2D.MakeFromPoints(p_triangleCentroids.ToArray());
            resolution = p_resolution;
            triangulationBounds = p_triangulationBounds;

            BuildTrianglesGrid(p_delaunator, p_delaunatorPoints, p_triangles, p_triangleBounds, p_triangleCentroids);
        }

        void BuildTrianglesGrid(
            Delaunator p_delaunator,
            List<Float2> p_delaunatorPoints,
            List<DelaunatorSharp.Triangle> p_triangles,
            List<AABB> p_triangleBounds,
            List<Float2> p_triangleCentroids)
        {
            int resolutionSqr = resolution * resolution;

            trianglesGrid = new List<int>();
            trianglesGrid.Resize(resolutionSqr);

            nodeDiameterX = (triangulationBounds.maxX - triangulationBounds.minX) / resolution;
            nodeDiameterY = (triangulationBounds.maxY - triangulationBounds.minY) / resolution;

            for (int i = 0; i < resolution; i++)
            {
                for (int j = 0; j < resolution; j++)
                {
                    int k = i * resolution + j;

                    float x = nodeDiameterX * (i + 0.5f) + triangulationBounds.minX;
                    float y = nodeDiameterY * (j + 0.5f) + triangulationBounds.minY;

                    trianglesGrid[k] = FindTriangleForPointByKdTree(new Float2(x, y), p_delaunator, p_delaunatorPoints, p_triangles, p_triangleBounds, p_triangleCentroids);
                }
            }
        }

        public int FindTriangleForPointByGrid(
            Float2 position,
            Delaunator p_delaunator,
            List<Float2> p_delaunatorPoints,
            List<DelaunatorSharp.Triangle> p_triangles,
            List<AABB> p_triangleBounds,
            List<Float2> p_triangleCentroids,
            bool d = false)
        {
            if (!triangulationBounds.IsInside(position))
            {
                int t = FindTriangleForPointByKdTree(position, p_delaunator, p_delaunatorPoints, p_triangles, p_triangleBounds, p_triangleCentroids);
                if(d)
                {
                    UnityEngine.Debug.Log($"aaa21 {t}");
                }
                return t;
            }

            int i = (int)((position.x - triangulationBounds.minX) / nodeDiameterX);
            int j = (int)((position.y - triangulationBounds.minY) / nodeDiameterY);

            int k = i * resolution + j;
            int initialTriangle = trianglesGrid[k];

            if (initialTriangle == -1)
            {
                int t = FindTriangleForPointByKdTree(position, p_delaunator, p_delaunatorPoints, p_triangles, p_triangleBounds, p_triangleCentroids, d);
                if(d)
                {
                    UnityEngine.Debug.Log($"aaa22 {t}");
                }
                return t;
            }

            int t1 = FindTriangleForPointByWalking(position, initialTriangle, p_delaunator, p_delaunatorPoints, p_triangles, p_triangleBounds, p_triangleCentroids);
            if(d)
            {
                UnityEngine.Debug.Log($"aaa23 {t1}");
            }
            return t1;
        }

        public int FindTriangleForPointByKdTree(
            Float2 position,
            Delaunator p_delaunator,
            List<Float2> p_delaunatorPoints,
            List<DelaunatorSharp.Triangle> p_triangles,
            List<AABB> p_triangleBounds,
            List<Float2> p_triangleCentroids,
            bool d = false)
        {
            int initialTriangle = kdtree.FindNearest(position);
            int t = FindTriangleForPointByWalking(position, initialTriangle, p_delaunator, p_delaunatorPoints, p_triangles, p_triangleBounds, p_triangleCentroids, d);

            if(d)
            {
                UnityEngine.Debug.Log($"aaa31 {initialTriangle} {t}");
            }

            return t;
        }
        
        public int nVisitsFindTriangleForPointByWalking;
        public int FindTriangleForPointByWalking(
            Float2 position,
            int initialTriangle,
            Delaunator p_delaunator,
            List<Float2> p_delaunatorPoints,
            List<DelaunatorSharp.Triangle> p_triangles,
            List<AABB> p_triangleBounds,
            List<Float2> p_triangleCentroids,
            bool d = false)
        {
            int triangleToVisit = initialTriangle;

            while (triangleToVisit != -1)
            {
                int triangle = triangleToVisit;
                triangleToVisit = -1;

                if(d)
                {
                    UnityEngine.Debug.Log($"aaa41 {triangle}");
                }

                if (p_triangleBounds[triangle].IsInsideOrOnTheBoundary(position))
                {
                    List<int> trianglePoints = p_triangles[triangle].points;

                    Float2 p1 = p_delaunatorPoints[trianglePoints[0]];
                    Float2 p2 = p_delaunatorPoints[trianglePoints[1]];
                    Float2 p3 = p_delaunatorPoints[trianglePoints[2]];

                    if (VectorUtils.PointInTriangle(position, p1, p2, p3))
                    {
                        return triangle;
                    }
                }

                nVisitsFindTriangleForPointByWalking++;

                Float2 center = p_triangleCentroids[triangle];
                bool neighbourAdded = false;

                for (int i = 0; i < 3; i++)
                {
                    int e = 3 * triangle + i;
                    int opposite = p_delaunator.halfedges[e];

                    if(d)
                    {
                        UnityEngine.Debug.Log($"aaa42 {opposite}");
                    }

                    if (opposite >= 0)
                    {
                        int nextTriangle = Delaunator.TriangleOfEdge(opposite);
                        if (!neighbourAdded)
                        {
                            int p = p_delaunator.triangles[e];
                            int q = p_delaunator.triangles[Delaunator.NextHalfedge(e)];

                            if(d)
                            {
                                UnityEngine.Debug.Log($"aaa43 {VectorUtils.AreLineSegmentsIntersecting(position, center, p_delaunatorPoints[p], p_delaunatorPoints[q])}");
                                UnityEngine.Debug.Log($"aaa44 {VectorUtils.PointOnLine2D(position, p_delaunatorPoints[p], p_delaunatorPoints[q], 0.0001f)} {VectorUtils.PointOnLine2D(center, p_delaunatorPoints[p], p_delaunatorPoints[q], 0.0001f)}");
                            }

                            if (VectorUtils.AreLineSegmentsIntersecting(position, center, p_delaunatorPoints[p], p_delaunatorPoints[q]))
                            {
                                triangleToVisit = nextTriangle;
                                neighbourAdded = true;
                            }
                        }
                    }
                }
            }

            return -1;
        }
    }
}
