using System.Collections.Generic;

namespace TriangulationNavigation
{
    public struct DelaunatorStatistics
    {
        public float totalEdgesLength;
        public List<int> pointsCount;
        public List<int> pointsCountSums;
        public int maxPointsCount;
    }
}
