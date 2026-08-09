using System.Collections.Generic;
using GenericCode;

namespace TriangulationNavigation
{
    public struct Obstacle
    {
        public int pointsIndexStart;
        public int pointsCount;

        public List<Float2> obstacleCorners;
        public List<bool> isCornerIntersectingWithWorldBounds;
        public List<int> nSplits;

        public Float2 center;
        public float largestCornerDistance;
        public bool isWalkable;
    }
}
