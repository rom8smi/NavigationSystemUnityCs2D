using System.Collections.Generic;

namespace GenericCode
{
    public static class PointsGenerator
    {
        public static List<Float2> GetTestTrianglePoint()
        {
            List<Float2> points = new List<Float2>();
            points.Add(new Float2(0.0f, 0.0f));
            points.Add(new Float2(10.0f, 0.0f));
            points.Add(new Float2(10.0f, 10.0f));
            points.Add(new Float2(15.0f, 15.0f));
            return points;
        }

        public static List<Float2> GetRandomPointsInsideCircle(int numberOfPoints, int seed, float radius)
        {
            ManualRandom random = new ManualRandom((ulong)seed);
            return GetRandomPointsInsideCircle(random, numberOfPoints, radius);
        }

        public static List<Float2> GetRandomPointsInsideCircle(ManualRandom random, int numberOfPoints, float radius)
        {
            List<Float2> points = new List<Float2>();

            for (int i = 0; i < numberOfPoints; i++)
            {
                points.Add(VectorUtils.RangomInsideUnitCircle(random) * radius);
            }

            return points;
        }
    }
}
