using GenericCode;

namespace TriangulationNavigation
{
    public struct AABB
    {
        public float minX;
        public float maxX;
        public float minY;
        public float maxY;

        public bool IsInside(Float2 point)
        {
            return point.x > minX && point.x < maxX && point.y > minY && point.y < maxY;
        }

        public bool IsInsideOrOnTheBoundary(Float2 point)
        {
            return point.x >= minX && point.x <= maxX && point.y >= minY && point.y <= maxY;
        }

        public static bool AreBoundsOverlapping(AABB boundsA, AABB boundsB)
        {
            if (
                boundsA.IsInsideOrOnTheBoundary(new Float2(boundsB.minX, boundsB.minY)) ||
                boundsA.IsInsideOrOnTheBoundary(new Float2(boundsB.maxX, boundsB.minY)) ||
                boundsA.IsInsideOrOnTheBoundary(new Float2(boundsB.minX, boundsB.maxY)) ||
                boundsA.IsInsideOrOnTheBoundary(new Float2(boundsB.maxX, boundsB.maxY))
            )
            {
                return true;
            }
            return false;
        }

        public override string ToString()
        {
            return $"(x: {minX} {maxX}, y: {minY} {maxY})";
        }
    }
}
