using System.Collections.Generic;
using DelaunatorSharp;
using GenericCode;

namespace TriangulationNavigation
{
    public static class DelaunatorDebugger
    {
        public static void DebugDelaunatorStatistics(Delaunator triangulation, List<Float2> points)
        {
            DelaunatorStatistics delaunatorStatistics = GetDelaunatorStatistics(triangulation, points);

            string outputResult = "---- Delaunator statistics ----\n";
            outputResult += "   Total Edges Length: " + delaunatorStatistics.totalEdgesLength.ToString() + "\n";
            outputResult += "   Max Points Count: " + delaunatorStatistics.maxPointsCount.ToString() + "\n";
            outputResult += "   Points Counts: \n";

            for(int i=0; i<delaunatorStatistics.maxPointsCount; i++)
            {
                outputResult += "       " + i.ToString() + ": " + delaunatorStatistics.pointsCountSums[i].ToString() + "\n";
            }

            outputResult += "--------------------------";

            GenericCode.Debug.Log(outputResult);
        }

        public static DelaunatorStatistics GetDelaunatorStatistics(Delaunator triangulation, List<Float2> points)
        {
            List<Triangle> triangles = triangulation.GetTriangles();
            int nTriangles = triangles.Count;
            int nPoints = points.Count;

            float totalEdgesLength = 0.0f;

            for (int i = 0; i < nTriangles; i++)
            {
                List<int> trianglePoints = triangles[i].points;
                float l1 = (points[trianglePoints[0]] - points[trianglePoints[1]]).Length();
                float l2 = (points[trianglePoints[1]] - points[trianglePoints[2]]).Length();
                float l3 = (points[trianglePoints[2]] - points[trianglePoints[0]]).Length();

                totalEdgesLength += (l1 + l2 + l3);
            }

            List<int> pointsCount = new List<int>();
            pointsCount.Resize(nPoints);

            for (int i = 0; i < nPoints; i++)
            {
                pointsCount[i] = 0;
            }

            for (int i = 0; i < nTriangles; i++)
            {
                List<int> trianglePoints = triangles[i].points;

                for (int j = 0; j < 3; j++)
                {
                    int pointIndex = trianglePoints[j];
                    pointsCount[pointIndex]++;
                }
            }

            int maxPointsCount = 0;

            for (int i = 0; i < nPoints; i++)
            {
                maxPointsCount = MathUtils.Max(maxPointsCount, pointsCount[i]);
            }

            List<int> pointsCountSums = new List<int>();
            pointsCountSums.Resize(maxPointsCount + 1);

            for (int i = 0; i < nPoints; i++)
            {
                pointsCountSums[pointsCount[i]]++;
            }

            return new DelaunatorStatistics
            {
                totalEdgesLength = totalEdgesLength,
                pointsCount = pointsCount,
                pointsCountSums = pointsCountSums,
                maxPointsCount = maxPointsCount
            };
        }
    }
}
