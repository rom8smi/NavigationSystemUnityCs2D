using System.Collections.Generic;
using System.Diagnostics;
using DelaunatorSharp;
using UnityEngine;
using GenericCode;

namespace TriangulationNavigation
{
    public class TriangleSearchTests : MonoBehaviour
    {
        Delaunator delaunator;
        List<Float2> delaunatorPoints;
        List<DelaunatorSharp.Triangle> triangles;
        List<AABB> triangleBounds;
        List<Float2> triangleCentroids;
        TriangulationSearch triangulationSearch;
        TriangulationGridSearch triangulationGridSearch;

        void Start()
        {
            ManualRandom random = new ManualRandom(2);
            delaunatorPoints = PointsGenerator.GetRandomPointsInsideCircle(random, 300, 400.0f);
            delaunator = new Delaunator();
            delaunator.Create(delaunatorPoints);
            triangles = delaunator.GetTriangles();

            triangleBounds = new List<AABB>();

            for (int i = 0; i < triangles.Count; i++)
            {
                AABB aabb = new AABB
                {
                    minX = float.MaxValue,
                    maxX = float.MinValue,
                    minY = float.MaxValue,
                    maxY = float.MinValue,
                };
                List<int> trianglePoints = triangles[i].points;

                for (int j = 0; j < trianglePoints.Count; j++)
                {
                    Float2 point = delaunatorPoints[trianglePoints[j]];
                    aabb.minX = MathUtils.Min(aabb.minX, point.x);
                    aabb.maxX = MathUtils.Max(aabb.maxX, point.x);
                    aabb.minY = MathUtils.Min(aabb.minY, point.y);
                    aabb.maxY = MathUtils.Max(aabb.maxY, point.y);
                }

                triangleBounds.Add(aabb);
            }

            triangleCentroids = new List<Float2>();
            triangleCentroids.Resize(triangles.Count);

            for (int i = 0; i < triangles.Count; i++)
            {
                List<int> trianglePoints = triangles[i].points;

                Float2 p1 = delaunatorPoints[trianglePoints[0]];
                Float2 p2 = delaunatorPoints[trianglePoints[1]];
                Float2 p3 = delaunatorPoints[trianglePoints[2]];

                triangleCentroids[i] = (p1 + p2 + p3) / 3.0f;
            }

            AABB triangulationBounds = new AABB
            {
                minX = -401f,
                maxX = 401f,
                minY = -401f,
                maxY = 401f
            };

            int resolution = 40;

            triangulationSearch = new TriangulationSearch();
            triangulationSearch.Create(triangulationBounds, resolution, delaunator, delaunatorPoints, triangles, triangleBounds, triangleCentroids);

            int trianglesCount = triangles.Count;
            triangulationGridSearch = new TriangulationGridSearch();

            List<bool> trianglesMask = new List<bool>();
            trianglesMask.Resize(trianglesCount);
            for (int i = 0; i < trianglesCount; i++)
            {
                trianglesMask[i] = true;
            }

            triangulationGridSearch.Create(triangulationBounds, resolution, delaunator, delaunatorPoints, trianglesCount, trianglesMask);


            int nQueryPoints = 2000;
            List<Float2> queryPoint = PointsGenerator.GetRandomPointsInsideCircle(random, nQueryPoints, 400.0f);

            List<int> findTriangleForPointResults = new List<int>();
            List<int> findTriangleForPointWithBoxResults = new List<int>();
            List<int> findTriangleForPointByWalkingResults = new List<int>();
            List<int> findTriangleForPointByKdTreeResults = new List<int>();
            List<int> findTriangleForPointByGridResults = new List<int>();
            List<int> findTriangleForPointByGridSearchResults = new List<int>();

            findTriangleForPointResults.Resize(nQueryPoints);
            findTriangleForPointWithBoxResults.Resize(nQueryPoints);
            findTriangleForPointByWalkingResults.Resize(nQueryPoints);
            findTriangleForPointByKdTreeResults.Resize(nQueryPoints);
            findTriangleForPointByGridResults.Resize(nQueryPoints);
            findTriangleForPointByGridSearchResults.Resize(nQueryPoints);

            Stopwatch sw = new Stopwatch();
            sw.Start();

            for (int i = 0; i < nQueryPoints; i++)
            {
                findTriangleForPointResults[i] = FindTriangleForPoint(queryPoint[i]);
            }

            double t1 = sw.Elapsed.TotalMilliseconds;

            for (int i = 0; i < nQueryPoints; i++)
            {
                findTriangleForPointWithBoxResults[i] = FindTriangleForPointWithBox(queryPoint[i]);
            }

            double t2 = sw.Elapsed.TotalMilliseconds;

            for (int i = 0; i < nQueryPoints; i++)
            {
                findTriangleForPointByWalkingResults[i] = triangulationSearch.FindTriangleForPointByWalking(queryPoint[i], 0, delaunator, delaunatorPoints, triangles, triangleBounds, triangleCentroids);
            }

            double t3 = sw.Elapsed.TotalMilliseconds;

            int visitsDirectWalking = triangulationSearch.nVisitsFindTriangleForPointByWalking;
            triangulationSearch.nVisitsFindTriangleForPointByWalking = 0;

            for (int i = 0; i < nQueryPoints; i++)
            {
                findTriangleForPointByKdTreeResults[i] = triangulationSearch.FindTriangleForPointByKdTree(queryPoint[i], delaunator, delaunatorPoints, triangles, triangleBounds, triangleCentroids);
            }

            double t4 = sw.Elapsed.TotalMilliseconds;

            int visitsWithKdTree = triangulationSearch.nVisitsFindTriangleForPointByWalking;

            triangulationSearch.nVisitsFindTriangleForPointByWalking = 0;

            for (int i = 0; i < nQueryPoints; i++)
            {
                findTriangleForPointByGridResults[i] = triangulationSearch.FindTriangleForPointByGrid(queryPoint[i], delaunator, delaunatorPoints, triangles, triangleBounds, triangleCentroids);
            }

            int visitsGrid = triangulationSearch.nVisitsFindTriangleForPointByWalking;

            double t5 = sw.Elapsed.TotalMilliseconds;

            for (int i = 0; i < nQueryPoints; i++)
            {
                findTriangleForPointByGridSearchResults[i] = triangulationGridSearch.FindTriangleForPoint(queryPoint[i], delaunator, delaunatorPoints);
            }

            double t6 = sw.Elapsed.TotalMilliseconds;


            int missmatchesFindTriangleForPointWithBox = 0;
            for (int i = 0; i < nQueryPoints; i++)
            {
                if (findTriangleForPointResults[i] != findTriangleForPointWithBoxResults[i])
                {
                    missmatchesFindTriangleForPointWithBox++;
                }
            }

            int missmatchesFindTriangleForPointByWalking = 0;
            for (int i = 0; i < nQueryPoints; i++)
            {
                if (findTriangleForPointResults[i] != findTriangleForPointByWalkingResults[i])
                {
                    missmatchesFindTriangleForPointByWalking++;
                }
            }

            int missmatchesFindTriangleForPointByKdTree = 0;
            for (int i = 0; i < nQueryPoints; i++)
            {
                if (findTriangleForPointResults[i] != findTriangleForPointByKdTreeResults[i])
                {
                    missmatchesFindTriangleForPointByKdTree++;
                }
            }

            int missmatchesFindTriangleForPointGrid = 0;
            for (int i = 0; i < nQueryPoints; i++)
            {
                if (findTriangleForPointResults[i] != findTriangleForPointByGridResults[i])
                {
                    missmatchesFindTriangleForPointGrid++;
                }
            }

            int missmatchesFindTriangleForPointGridSearch = 0;
            for (int i = 0; i < nQueryPoints; i++)
            {
                if (findTriangleForPointResults[i] != findTriangleForPointByGridSearchResults[i])
                {
                    missmatchesFindTriangleForPointGridSearch++;
                }
            }

            UnityEngine.Debug.Log(
                missmatchesFindTriangleForPointWithBox + " " + missmatchesFindTriangleForPointByWalking + " " + missmatchesFindTriangleForPointByKdTree + " " + missmatchesFindTriangleForPointGrid + " " + missmatchesFindTriangleForPointGridSearch +
                " | " + t1 + " " + (t2 - t1) + " " + (t3 - t2) + " " + (t4 - t3) + " " + (t5 - t4) + " " + (t6 - t5) +
                " | " + nVisitsFindTriangleForPoint + " " + nVisitsFindTriangleForPointWithBox + " " + visitsDirectWalking + " " + visitsWithKdTree + " " + visitsGrid);
        }

        int nVisitsFindTriangleForPoint = 0;
        int FindTriangleForPoint(Float2 position)
        {
            for (int i = 0; i < triangles.Count; i++)
            {
                nVisitsFindTriangleForPoint++;

                List<int> trianglePoints = triangles[i].points;
                Float2 p1 = delaunatorPoints[trianglePoints[0]];
                Float2 p2 = delaunatorPoints[trianglePoints[1]];
                Float2 p3 = delaunatorPoints[trianglePoints[2]];

                if (VectorUtils.PointInTriangle(position, p1, p2, p3))
                {
                    return i;
                }
            }

            return -1;
        }

        int nVisitsFindTriangleForPointWithBox = 0;
        int FindTriangleForPointWithBox(Float2 position)
        {
            for (int i = 0; i < triangles.Count; i++)
            {
                nVisitsFindTriangleForPointWithBox++;

                if (triangleBounds[i].IsInsideOrOnTheBoundary(position))
                {
                    List<int> trianglePoints = triangles[i].points;
                    Float2 p1 = delaunatorPoints[trianglePoints[0]];
                    Float2 p2 = delaunatorPoints[trianglePoints[1]];
                    Float2 p3 = delaunatorPoints[trianglePoints[2]];

                    if (VectorUtils.PointInTriangle(position, p1, p2, p3))
                    {
                        return i;
                    }
                }
            }

            return -1;
        }

        void Update()
        {

        }
    }
}
