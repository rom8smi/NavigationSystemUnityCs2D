using GenericCode;

namespace DelaunatorSharp
{
    public struct VoronoiCell
    {
        public Float2[] points;
        public int index;
        public VoronoiCell(int p_triangleIndex, Float2[] p_points)
        {
            points = p_points;
            index = p_triangleIndex;
        }
    }
}
