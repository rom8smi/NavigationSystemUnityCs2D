namespace BowyerWatsonTriangulationNamespace
{
    public class Edge
    {
        public Point Point1;
        public Point Point2;

        public Edge(Point point1, Point point2)
        {
            Point1 = point1;
            Point2 = point2;
        }

        public override bool Equals(object obj)
        {
            if (obj == null) return false;
            if (obj.GetType() != GetType()) return false;
            var edge = obj as Edge;

            var samePoints = Point1 == edge.Point1 && Point2 == edge.Point2;
            var samePointsReversed = Point1 == edge.Point2 && Point2 == edge.Point1;
            return samePoints || samePointsReversed;
        }

        public static bool IsTheSame(Edge e1, Edge e2)
        {
            if(e1.Point1.index == e2.Point1.index && e1.Point2.index == e2.Point2.index)
            {
                return true;
            }
            if(e1.Point1.index == e2.Point2.index && e1.Point2.index == e2.Point1.index)
            {
                return true;
            }
            return false;
        }

        public override int GetHashCode()
        {
            int hCode = (int)Point1.x ^ (int)Point1.y ^ (int)Point2.x ^ (int)Point2.y;
            return hCode.GetHashCode();
        }

        public override string ToString()
        {
            return $"{Point1.index} - {Point2.index}";
        }
    }
}
