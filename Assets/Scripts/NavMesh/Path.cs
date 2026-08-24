using System.Collections.Generic;
using GenericCode;

namespace TriangulationNavigation
{
    public class Path
    {
        public List<Float2> waypoints;
        public bool success;
        public int lowestHCostNode;
    }
}
