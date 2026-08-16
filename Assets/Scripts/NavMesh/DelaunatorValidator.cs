using DelaunatorSharp;
using System.Collections.Generic;
using GenericCode;

namespace TriangulationNavigation
{
    public static class DelaunatorValidator
    {
        public static void Validate(Delaunator del)
        {
            int invalidCount = 0;

            List<Triangle> triangles = del.GetTriangles();
            for (int i = 0; i < triangles.Count; i++)
            {
                Float2 p1 = GetPoint(triangles[i].points[0], del);
                Float2 p2 = GetPoint(triangles[i].points[1], del);
                Float2 p3 = GetPoint(triangles[i].points[2], del);

                if (IsPointOnLineSegment(p1, p2, p3) || IsPointOnLineSegment(p1, p3, p2) || IsPointOnLineSegment(p2, p3, p1))
                {
                    invalidCount++;
                }
            }

            GenericCode.Debug.Log(invalidCount.ToString());
        }

        public static void ValidateConstraintEdges(Delaunator del, List<ConstraintEdge> constraintEdges)
        {
            // float epsilon = 0.001f;
            int numPoints = del.coords.Count / 2;
            int invalidCount = 0;

            for (int i = 0; i < constraintEdges.Count; i++)
            {
                int p = constraintEdges[i].p;
                int q = constraintEdges[i].q;

                Float2 pxy = GetPoint(p, del);
                Float2 qxy = GetPoint(q, del);

                // Float2 mid = (pxy + qxy) * 0.5f;
                // float radius = (pxy - mid).magnitude;

                for (int j = 0; j < numPoints; j++)
                {
                    if (j != p && j != q)
                    {
                        Float2 jxy = GetPoint(j, del);

                        if (IsPointOnLineSegment(pxy, qxy, jxy))
                        {
                            invalidCount++;
                        }
                    }
                }
            }

            GenericCode.Debug.Log($"{invalidCount} {numPoints} {constraintEdges.Count}");
        }

        static bool IsPointOnLineSegment(Float2 linePointA, Float2 linePointB, Float2 point)
        {
            float pa = (linePointA - point).Length();
            float pb = (linePointB - point).Length();
            float ab = (linePointA - linePointB).Length();

            if ((pa + pb - ab) == 0)
            {
                return true;
            }

            return false;
        }

        static Float2 GetPoint(int i, Delaunator del)
        {
            return new Float2(del.coords[i * 2], del.coords[i * 2 + 1]);
        }
    }
}
