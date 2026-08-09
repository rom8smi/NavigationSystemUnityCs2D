using System.Collections.Generic;
using TriangulationNavigation;

namespace GenericCode
{
    public static class UnionPolygonFinder
    {
        public static void FindUnionsForMultiplePolygons(List<List<Float2>> polygons)
        {
            bool anyUnionFound = true;

            while (anyUnionFound)
            {
                anyUnionFound = false;

                List<AABB> allBounds = new List<AABB>();
                for (int i = 0; i < polygons.Count; i++)
                {
                    allBounds.Add(GetPolygonBounds(polygons[i]));
                }

                for (int i = 0; i < polygons.Count; i++)
                {
                    for (int j = i + 1; j < polygons.Count; j++)
                    {
                        if (
                            !anyUnionFound &&
                            (
                                AABB.AreBoundsOverlapping(allBounds[i], allBounds[j]) ||
                                AABB.AreBoundsOverlapping(allBounds[j], allBounds[i])
                            )
                        )
                        {
                            if (IsPolygonInAnotherPolygon(polygons[j], polygons[i]))
                            {
                                polygons.RemoveAt(j);
                                anyUnionFound = true;
                            }
                        }
                    }
                }

                for (int i = 0; i < polygons.Count; i++)
                {
                    for (int j = i + 1; j < polygons.Count; j++)
                    {
                        if (
                            !anyUnionFound &&
                            (
                                AABB.AreBoundsOverlapping(allBounds[i], allBounds[j]) ||
                                AABB.AreBoundsOverlapping(allBounds[j], allBounds[i])
                            )
                        )
                        {
                            List<List<Float2>> polygonsPair = new List<List<Float2>>
                            {
                                polygons[i],
                                polygons[j]
                            };

                            bool unionResult = TryFindTwoPolygonsUnion(polygonsPair, out List<Float2> union);
                            if (unionResult)
                            {
                                polygons[i] = union;
                                polygons.RemoveAt(j);
                                anyUnionFound = true;
                            }
                        }
                    }
                }
            }
        }

        public static void FindUnionsForMultiplePolygonsNoBoundsCheck(List<List<Float2>> polygons)
        {
            bool anyUnionFound = true;

            while (anyUnionFound)
            {
                anyUnionFound = false;

                List<AABB> allBounds = new List<AABB>();
                for (int i = 0; i < polygons.Count; i++)
                {
                    allBounds.Add(GetPolygonBounds(polygons[i]));
                }

                for (int i = 0; i < polygons.Count; i++)
                {
                    for (int j = i + 1; j < polygons.Count; j++)
                    {
                        if (!anyUnionFound)
                        {
                            if (IsPolygonInAnotherPolygon(polygons[j], polygons[i]))
                            {
                                polygons.RemoveAt(j);
                                anyUnionFound = true;
                            }
                        }
                    }
                }

                for (int i = 0; i < polygons.Count; i++)
                {
                    for (int j = i + 1; j < polygons.Count; j++)
                    {
                        if (!anyUnionFound)
                        {
                            List<List<Float2>> polygonsPair = new List<List<Float2>>
                            {
                                polygons[i],
                                polygons[j]
                            };

                            bool unionResult = TryFindTwoPolygonsUnion(polygonsPair, out List<Float2> union);
                            if (unionResult)
                            {
                                polygons[i] = union;
                                polygons.RemoveAt(j);
                                anyUnionFound = true;
                            }
                        }
                    }
                }
            }
        }

        static bool TryFindTwoPolygonsUnion(List<List<Float2>> polygons, out List<Float2> union)
        {
            List<List<bool>> visited = new List<List<bool>>();
            for (int i = 0; i < 2; i++)
            {
                visited.Add(new List<bool>());
                for (int j = 0; j < polygons[i].Count; j++)
                {
                    visited[i].Add(false);
                }
            }

            int currentPolygon = 0;
            int comparingPolygon = 1;
            int currentIndex = 0;

            float minX = float.MaxValue;
            for (int i = 0; i < polygons.Count; i++)
            {
                for (int j = 0; j < polygons[i].Count; j++)
                {
                    if (polygons[i][j].x < minX)
                    {
                        minX = polygons[i][j].x;

                        currentPolygon = i;
                        comparingPolygon = i + 1;
                        if (comparingPolygon >= 2)
                        {
                            comparingPolygon = 0;
                        }
                        currentIndex = j;
                    }
                }
            }

            union = new List<Float2>();
            bool resultFound = false;

            while (!visited[currentPolygon][currentIndex])
            {
                int nextIndex = currentIndex + 1;
                if (nextIndex >= polygons[currentPolygon].Count)
                {
                    nextIndex = 0;
                }

                Float2 currentPoint = polygons[currentPolygon][currentIndex];
                Float2 nextPoint = polygons[currentPolygon][nextIndex];

                visited[currentPolygon][currentIndex] = true;
                currentIndex = nextIndex;

                int intersectionIndex = -1;
                float minIntersectionDistanceSqr = float.MaxValue;
                Float2 intersectionPosition = Float2.Zero();

                for (int j = 0; j < polygons[comparingPolygon].Count; j++)
                {
                    Float2 comparingPolygonPoint = polygons[comparingPolygon][j];
                    int jNext = j + 1;
                    if (jNext >= polygons[comparingPolygon].Count)
                    {
                        jNext = 0;
                    }

                    Float2 nextComparingPolygonPoint = polygons[comparingPolygon][jNext];
                    LineSegmentsIntersectionResult lineSegmentsIntersectionResult = VectorUtils.LineSegmentsIntersection(
                        currentPoint,
                        nextPoint,
                        comparingPolygonPoint,
                        nextComparingPolygonPoint
                    );

                    if (lineSegmentsIntersectionResult.intersects)
                    {
                        float intersectionDistance = (currentPoint - lineSegmentsIntersectionResult.intersection).LengthSquared();
                        if (intersectionDistance < minIntersectionDistanceSqr)
                        {
                            intersectionIndex = jNext;
                            minIntersectionDistanceSqr = intersectionDistance;
                            intersectionPosition = lineSegmentsIntersectionResult.intersection;
                        }
                    }
                }

                if (intersectionIndex != -1)
                {
                    int oldCurrentPolygon = currentPolygon;
                    currentPolygon = comparingPolygon;
                    comparingPolygon = oldCurrentPolygon;
                    currentIndex = intersectionIndex;

                    union.Add(intersectionPosition);
                    resultFound = true;
                }

                union.Add(polygons[currentPolygon][currentIndex]);
            }

            return resultFound;
        }

        static bool IsPolygonInAnotherPolygon(List<Float2> polygonA, List<Float2> polygonB)
        {
            int insidePointsCount = 0;
            for (int i = 0; i < polygonA.Count; i++)
            {
                if (VectorUtils.IsPointInPolygon(polygonA[i], polygonB))
                {
                    insidePointsCount++;
                }
            }

            if (insidePointsCount == polygonA.Count)
            {
                return true;
            }
            return false;
        }

        static AABB GetPolygonBounds(List<Float2> polygon)
        {
            AABB aabb = new AABB
            {
                minX = float.MaxValue,
                maxX = float.MinValue,
                minY = float.MaxValue,
                maxY = float.MinValue,
            };

            for (int i = 0; i < polygon.Count; i++)
            {
                Float2 point = polygon[i];
                aabb.minX = MathUtils.Min(aabb.minX, point.x);
                aabb.maxX = MathUtils.Max(aabb.maxX, point.x);
                aabb.minY = MathUtils.Min(aabb.minY, point.y);
                aabb.maxY = MathUtils.Max(aabb.maxY, point.y);
            }

            return aabb;
        }
    }
}
