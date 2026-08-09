using System.Collections.Generic;
using GenericCode;

namespace TriangulationNavigation
{
    public static class PathUtils
    {
        public static float CalculateTotalPathLength(List<Float2> waypoints)
        {
            return CalculatePathLength(waypoints, 0);
        }

        public static float CalculatePathLength(List<Float2> waypoints, int startIndex)
        {
            float pathLength = 0.0f;

            for (int i = startIndex; i < waypoints.Count - 1; i++)
            {
                pathLength += (waypoints[i + 1] - waypoints[i]).Length();
            }

            return pathLength;
        }
    }
}
