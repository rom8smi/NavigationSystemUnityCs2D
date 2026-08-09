using System.Collections.Generic;

namespace DelaunatorSharp
{
    public struct Triangle
    {
        public int index;

        public List<int> points;

        public Triangle(int p_t, List<int> p_points)
        {
            points = p_points;
            index = p_t;
        }
    }
}
