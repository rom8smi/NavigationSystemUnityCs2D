using System.Collections.Generic;

namespace GenericCode
{
    public static class LineSegmentUtils
    {
        public static void GetAllIntersectionsSimple(
            List<Float2> dataPoints,
            List<int> segmentStarts,
            List<int> segmentEnds,
            List<Float2> intersectionPoints,
            List<int> firstIntersectionSegments,
            List<int> secondIntersectionSegments)
        {
            int nSegments = segmentStarts.Count;

            for (int i = 0; i < nSegments; i++)
            {
                int startIndexI = segmentStarts[i];
                int endIndexI = segmentEnds[i];
                Float2 startPointI = dataPoints[startIndexI];
                Float2 endPointI = dataPoints[endIndexI];

                for (int j = i + 1; j < nSegments; j++)
                {
                    int startIndexJ = segmentStarts[j];
                    int endIndexJ = segmentEnds[j];
                    Float2 startPointJ = dataPoints[startIndexJ];
                    Float2 endPointJ = dataPoints[endIndexJ];

                    LineSegmentsIntersectionResult result = VectorUtils.LineSegmentsIntersection(startPointI, endPointI, startPointJ, endPointJ);
                    if (result.intersects)
                    {
                        intersectionPoints.Add(result.intersection);
                        firstIntersectionSegments.Add(i);
                        secondIntersectionSegments.Add(j);
                    }
                }
            }
        }

        public static void GetAllIntersectionsKdTreeWithRandomOffset(
            List<Float2> dataPoints,
            List<int> segmentStarts,
            List<int> segmentEnds,
            List<Float2> intersectionPoints,
            List<int> firstIntersectionSegments,
            List<int> secondIntersectionSegments)
        {
            int nSegments = segmentStarts.Count;

            List<Float2> centers = new List<Float2>();
            List<float> radii = new List<float>();
            centers.Resize(nSegments);
            radii.Resize(nSegments);

            float maxRadius = 0.0f;
            float epsilon = 0.001f;

            for (int i = 0; i < nSegments; i++)
            {
                int startIndexI = segmentStarts[i];
                int endIndexI = segmentEnds[i];

                Float2 center = (dataPoints[startIndexI] + dataPoints[endIndexI]) * 0.5f;
                float radius = (center - dataPoints[startIndexI]).Length();
                maxRadius = MathUtils.Max(maxRadius, radius);

                centers[i] = center;
                radii[i] = radius;
            }

            KDTree2D kdTree = KDTree2D.MakeFromPoints(centers.ToArray());
            List<int> neighbours = new List<int>();

            for (int i = 0; i < nSegments; i++)
            {
                int startIndexI = segmentStarts[i];
                int endIndexI = segmentEnds[i];
                Float2 startPointI = dataPoints[startIndexI];
                Float2 endPointI = dataPoints[endIndexI];

                float searchDistance = maxRadius + radii[i] + 6.0f * epsilon;

                neighbours.Clear();
                kdTree.FindNearestsBall(centers[i], searchDistance, neighbours);
                int neighboursCount = neighbours.Count;

                if (neighboursCount > 0)
                {
                    HeapSort.Sort(neighbours, neighboursCount);

                    for (int j = 0; j < neighboursCount; j++)
                    {
                        int neighbour = neighbours[j];
                        if (neighbour > i)
                        {
                            int startIndexNeighbour = segmentStarts[neighbour];
                            int endIndexNeighbour = segmentEnds[neighbour];
                            Float2 startPointNeighbour = dataPoints[startIndexNeighbour];
                            Float2 endPointNeighbour = dataPoints[endIndexNeighbour];

                            LineSegmentsIntersectionResult result = VectorUtils.LineSegmentsIntersection(startPointI, endPointI, startPointNeighbour, endPointNeighbour);
                            if (result.intersects)
                            {
                                intersectionPoints.Add(result.intersection);
                                firstIntersectionSegments.Add(i);
                                secondIntersectionSegments.Add(neighbour);
                            }
                        }
                    }
                }
            }
        }
    }
}
