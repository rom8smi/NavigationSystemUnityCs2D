using System.Collections.Generic;
using GenericCode;

namespace TriangulationNavigation
{
    public struct RectangularObstacle
    {
        public Float2 center;
        public Float2 size;
        public float rotation;
        public float radius;

        public List<Float2> GetCorners()
        {
            return new List<Float2>
            {
                Corner(-1, -1),
                Corner(1, -1),
                Corner(1, 1),
                Corner(-1, 1),
            };
        }

        Float2 Corner(int xSign, int ySign)
        {
            Float2 position = new Float2(0.5f * xSign * (size.x + 2.0f * radius), 0.5f * ySign * (size.y + 2.0f * radius));
            position = VectorUtils.Rotate(position, rotation);
            return position + center;
        }
    }
}
