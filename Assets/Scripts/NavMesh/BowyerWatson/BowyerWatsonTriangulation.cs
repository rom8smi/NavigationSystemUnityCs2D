using System.Collections.Generic;
using System.Linq;
using GenericCode;

namespace BowyerWatsonTriangulationNamespace
{
    public class BowyerWatsonTriangulation
    {
        public List<Triangle> triangulation;

        public List<Point> points;
        public float minX;
        public float maxX;
        public float minY;
        public float maxY;

        public Float2 tri1;
        public Float2 tri2;
        public Float2 tri3;
        KDTree2D kdtree;

        public void Triangulate(List<Float2> p_points)
        {
            // p_points = Sort.SortByX(p_points);
            // kdtree = KDTree2D.MakeFromPoints(p_points.ToArray());

            points = new List<Point>();
            for (int i = 0; i < p_points.Count; i++)
            {
                points.Add(Float2ToPoint(p_points[i], i));
            }

            minX = float.MaxValue;
            maxX = float.MinValue;
            minY = float.MaxValue;
            maxY = float.MinValue;

            for (int i = 0; i < points.Count; i++)
            {
                minX = MathUtils.Min(minX, points[i].x);
                maxX = MathUtils.Max(maxX, points[i].x);
                minY = MathUtils.Min(minY, points[i].y);
                maxY = MathUtils.Max(maxY, points[i].y);
            }

            Float2 center = new Float2(0.5f * (minX + maxX), 0.5f * (minY + maxY));

            tri1 = new Float2(center.x - 1.4f * (center.x - minX), center.y - 1.0f * (center.y - minY));
            tri2 = new Float2(center.x + 1.4f * (maxX - center.x), center.y - 1.0f * (center.y - minY));
            tri3 = new Float2(center.x, center.y + 1.4f * (maxY - center.y));

            tri1 -= center;
            tri2 -= center;
            tri3 -= center;

            tri1 *= 3f;
            tri2 *= 3f;
            tri3 *= 3f;

            tri1 += center;
            tri2 += center;
            tri3 += center;

            points.Add(Float2ToPoint(tri1, points.Count));
            points.Add(Float2ToPoint(tri2, points.Count));
            points.Add(Float2ToPoint(tri3, points.Count));

            var superTriangle = new Triangle(points[points.Count - 3], points[points.Count - 2], points[points.Count - 1]);
            triangulation = new List<Triangle>();

            triangulation.Add(superTriangle);

            for (int i = 0; i < points.Count; i++)
            {
                Point point = points[i];

                List<Triangle> badTriangles = FindBadTriangles(point, triangulation);
                List<Edge> polygon = FindHoleBoundaries(badTriangles);

                for (int j = 0; j < badTriangles.Count; j++)
                {
                    Triangle triangle = badTriangles[j];

                    for (int k = 0; k < 3; k++)
                    {
                        Point pointK = triangle.points[k];
                        pointK.adjacentTriangles.Remove(triangle);
                    }

                    triangulation.Remove(triangle);
                }

                for (int j = 0; j < polygon.Count; j++)
                {
                    Edge edge = polygon[j];
                    if (edge.Point1.index != i && edge.Point2.index != i)
                    {
                        Triangle triangle = new Triangle(point, edge.Point1, edge.Point2);
                        triangulation.Add(triangle);
                    }
                }
            }

            for (int i = 0; i < 3; i++)
            {
                points.RemoveAt(points.Count - 1);
            }

            List<Triangle> removals = new List<Triangle>();
            for (int i = 0; i < triangulation.Count; i++)
            {
                Triangle t = triangulation[i];
                bool shouldRemove = false;

                for (int j = 0; j < 3; j++)
                {
                    int pIndex = t.points[j].index;
                    for (int k = 0; k < 3; k++)
                    {
                        if (pIndex == superTriangle.points[k].index)
                        {
                            shouldRemove = true;
                        }
                    }
                }

                if (shouldRemove)
                {
                    removals.Add(t);
                }
            }

            for (int i = 0; i < removals.Count; i++)
            {
                Triangle t = removals[i];
                triangulation.Remove(t);
            }

            // UnityEngine.Debug.Log($"{badTrianglesFoundCount} {trianglesVisitedCount} {(1f * badTrianglesFoundCount) / trianglesVisitedCount}");
        }

        private List<Edge> FindHoleBoundaries(List<Triangle> badTriangles)
        {
            List<Edge> edges = new List<Edge>();
            for (int i = 0; i < badTriangles.Count; i++)
            {
                Triangle triangle = badTriangles[i];
                edges.Add(new Edge(triangle.points[0], triangle.points[1]));
                edges.Add(new Edge(triangle.points[1], triangle.points[2]));
                edges.Add(new Edge(triangle.points[2], triangle.points[0]));
            }

            List<int> edgeCounts = new List<int>();
            for (int i = 0; i < edges.Count; i++)
            {
                edgeCounts.Add(0);
            }

            for (int i = 0; i < edges.Count; i++)
            {
                int count = 0;
                for (int j = 0; j < edges.Count; j++)
                {
                    if (Edge.IsTheSame(edges[i], edges[j]))
                    {
                        count++;
                    }
                }
                edgeCounts[i] = count;
            }

            List<Edge> uniqueEdges = new List<Edge>();
            for (int i = 0; i < edges.Count; i++)
            {
                if (edgeCounts[i] == 1)
                {
                    uniqueEdges.Add(edges[i]);
                }
            }

            return uniqueEdges;
        }

        private List<Triangle> FindBadTriangles(Point point, List<Triangle> triangles)
        {
            List<Triangle> badTriangles = new List<Triangle>();
            for (int i = 0; i < triangles.Count; i++)
            {
                Triangle t = triangles[i];
                if (t.IsPointInsideCircumcircle(point))
                {
                    badTriangles.Add(t);
                }
            }
            return badTriangles;
        }

        private List<Triangle> FindBadTriangles2(Point point, List<Triangle> triangles)
        {
            List<Triangle> badTriangles = new List<Triangle>();
            HashSet<Triangle> trianglesToVisit = new HashSet<Triangle>();
            HashSet<Triangle> trianglesVisited = new HashSet<Triangle>();

            for (int i = 0; i < triangles.Count; i++)
            {
                Triangle t = triangles[i];

                if (t.IsPointInsideCircumcircle(point))
                {
                    trianglesToVisit.Add(t);
                    break;
                }
            }

            while (trianglesToVisit.Count > 0)
            {
                Triangle t = trianglesToVisit.First();
                trianglesVisited.Add(t);
                trianglesToVisit.Remove(t);

                if (t.IsPointInsideCircumcircle(point))
                {
                    badTriangles.Add(t);

                    for (int i = 0; i < 3; i++)
                    {
                        Point p = t.points[i];
                        foreach (var ta in p.adjacentTriangles)
                        {
                            if (!trianglesToVisit.Contains(ta) && !trianglesVisited.Contains(ta))
                            {
                                trianglesToVisit.Add(ta);
                            }
                        }
                    }
                }
            }

            return badTriangles;
        }

        int badTrianglesFoundCount = 0;
        int trianglesVisitedCount = 0;
        private List<Triangle> FindBadTriangles3(Point point, List<Triangle> triangles, int currentIndex)
        {
            List<Triangle> badTriangles = new List<Triangle>();
            HashSet<Triangle> trianglesToVisit = new HashSet<Triangle>();
            HashSet<Triangle> trianglesVisited = new HashSet<Triangle>();
            HashSet<Point> visitedPoints = new HashSet<Point>();

            bool found = false;

            if (currentIndex > 0)
            {
                int nearestPoint = kdtree.FindNearestWithMaxIndex(new Float2(point.x, point.y), currentIndex);

                if (nearestPoint != -1)
                {
                    trianglesToVisit.Add(points[nearestPoint].adjacentTriangles.Last());
                }
            }
            else
            {
                trianglesToVisit.Add(triangles[0]);
            }

            while (trianglesToVisit.Count > 0)
            {
                Triangle t = trianglesToVisit.First();
                trianglesVisited.Add(t);
                trianglesToVisit.Remove(t);

                bool isAdded = false;
                if (t.IsPointInsideCircumcircle(point))
                {
                    badTriangles.Add(t);
                    isAdded = true;
                    found = true;
                }

                if (!found || isAdded)
                {
                    for (int i = 0; i < 3; i++)
                    {
                        Point p = t.points[i];
                        foreach (var ta in p.adjacentTriangles)
                        {
                            if (!trianglesToVisit.Contains(ta) && !trianglesVisited.Contains(ta))
                            {
                                trianglesToVisit.Add(ta);
                            }
                        }
                    }
                }
            }

            badTrianglesFoundCount += badTriangles.Count;
            trianglesVisitedCount += trianglesVisited.Count();

            return badTriangles;
        }

        private List<Triangle> FindBadTriangles4(Point point, List<Triangle> triangles, int currentIndex)
        {
            List<Triangle> badTriangles = new List<Triangle>();
            List<Triangle> trianglesVisited = new List<Triangle>();

            List<Point> pointsToVisit = new List<Point>();
            List<Point> pointsVisited = new List<Point>();

            bool found = false;

            if (currentIndex > 0)
            {
                int nearestPoint = kdtree.FindNearestWithMaxIndex(new Float2(point.x, point.y), currentIndex);

                if (nearestPoint != -1)
                {
                    Point pt = points[nearestPoint];
                    pt.toVisit = true;
                    pointsToVisit.Add(pt);
                }
            }
            else
            {
                Point pt = triangles[0].points[0];
                pt.toVisit = true;
                pointsToVisit.Add(pt);
            }

            int pointsToVisitIndex = 0;
            while (pointsToVisitIndex < pointsToVisit.Count)
            {
                Point pt = pointsToVisit[pointsToVisitIndex];
                Triangle triangleToVisit = pt.adjacentTriangles[pt.visitedAdjestantTrianglesCount];
                bool triangleToVisitFound = false;

                if (!triangleToVisit.wasVisited)
                {
                    triangleToVisitFound = true;
                }

                if (triangleToVisitFound)
                {
                    triangleToVisit.wasVisited = true;
                    trianglesVisited.Add(triangleToVisit);

                    bool isAdded = false;
                    if (triangleToVisit.IsPointInsideCircumcircle(point))
                    {
                        badTriangles.Add(triangleToVisit);
                        isAdded = true;
                        found = true;
                    }

                    if (!found || isAdded)
                    {
                        for (int i = 0; i < 3; i++)
                        {
                            Point p = triangleToVisit.points[i];
                            if (!p.toVisit && !p.visited)
                            {
                                p.toVisit = true;
                                pointsToVisit.Add(p);
                            }
                        }
                    }
                }

                pt.visitedAdjestantTrianglesCount++;
                if (pt.visitedAdjestantTrianglesCount >= pt.adjacentTriangles.Count)
                {
                    pt.toVisit = false;
                    pt.visited = true;
                    pointsToVisitIndex++;

                    pointsVisited.Add(pt);
                }
            }

            foreach(var t in trianglesVisited)
            {
                t.wasVisited = false;
            }

            foreach (var p in pointsVisited)
            {
                p.visitedAdjestantTrianglesCount = 0;
                p.toVisit = false;
                p.visited = false;
            }

            badTrianglesFoundCount += badTriangles.Count;
            trianglesVisitedCount += trianglesVisited.Count;

            return badTriangles;
        }

        Point Float2ToPoint(Float2 v, int i)
        {
            return new Point(v.x, v.y, i);
        }
    }
}
