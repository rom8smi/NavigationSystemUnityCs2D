using GenericCode;

namespace BowyerWatsonTriangulationNamespace
{
    public class Triangle
    {
        public Point[] points = new Point[3];
        public Float2 circumcenter;
        public float radiusSquared;
        public bool wasVisited;

        public Triangle(Point point1, Point point2, Point point3)
        {
            if (!IsCounterClockwise(point1, point2, point3))
            {
                points[0] = point1;
                points[1] = point3;
                points[2] = point2;
            }
            else
            {
                points[0] = point1;
                points[1] = point2;
                points[2] = point3;
            }

            points[0].adjacentTriangles.Add(this);
            points[1].adjacentTriangles.Add(this);
            points[2].adjacentTriangles.Add(this);
            UpdateCircumcircle();
        }

        void UpdateCircumcircle()
        {
            // https://codefound.wordpress.com/2013/02/21/how-to-compute-a-circumcircle/#more-58
            // https://en.wikipedia.org/wiki/Circumscribed_circle
            Point p0 = points[0];
            Point p1 = points[1];
            Point p2 = points[2];
            float dA = p0.x * p0.x + p0.y * p0.y;
            float dB = p1.x * p1.x + p1.y * p1.y;
            float dC = p2.x * p2.x + p2.y * p2.y;

            float aux1 = (dA * (p2.y - p1.y) + dB * (p0.y - p2.y) + dC * (p1.y - p0.y));
            float aux2 = -(dA * (p2.x - p1.x) + dB * (p0.x - p2.x) + dC * (p1.x - p0.x));
            float div = (2.0f * (p0.x * (p2.y - p1.y) + p1.x * (p0.y - p2.y) + p2.x * (p1.y - p0.y)));

            // if (div == 0.0f)
            // {
            //     GenericCode.Debug.Log("UpdateCircumcircle div is 0");
            // }

            Float2 center = new Float2(aux1 / div, aux2 / div);
            circumcenter = center;
            radiusSquared = (center.x - p0.x) * (center.x - p0.x) + (center.y - p0.y) * (center.y - p0.y);
        }

        bool IsCounterClockwise(Point point1, Point point2, Point point3)
        {
            float result = (point2.x - point1.x) * (point3.y - point1.y) - (point3.x - point1.x) * (point2.y - point1.y);
            return result > 0;
        }

        public bool IsPointInsideCircumcircle(Point point)
        {
            float px = point.x;
            float py = point.y;
            float cx = circumcenter.x;
            float cy = circumcenter.y;

            float dSquared = (px - cx) * (px - cx) + (py - cy) * (py - cy);
            return dSquared < radiusSquared;
        }
    }
}
