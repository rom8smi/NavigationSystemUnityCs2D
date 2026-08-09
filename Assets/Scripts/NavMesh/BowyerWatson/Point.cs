using System.Collections.Generic;

namespace BowyerWatsonTriangulationNamespace
{
    public class Point
    {
        public float x;
        public float y;
        public int index;
        public List<Triangle> adjacentTriangles = new List<Triangle>();
        public int visitedAdjestantTrianglesCount;
        public bool toVisit;
        public bool visited;

        public Point(float p_x, float p_y, int p_index)
        {
            x = p_x;
            y = p_y;
            index = p_index;
        }

        public override string ToString()
        {
            return $"({x}; {y})";
        }
    }
}
