using UnityEngine;

namespace GenericCode
{
    public static class MathUtils
    {
        public const float PI = 3.1415927f;

        public static float InterpolateClamped(float x, float x0, float x1, float y0, float y1)
        {
            float y = (y0 + (y1 - y0) * (x - x0) / (x1 - x0));

            if (y0 < y1)
            {
                if (y < y0)
                {
                    y = y0;
                }
                if (y > y1)
                {
                    y = y1;
                }
            }
            else if (y0 > y1)
            {
                if (y > y0)
                {
                    y = y0;
                }
                if (y < y1)
                {
                    y = y1;
                }
            }

            return y;
        }

        // Based on https://stackoverflow.com/questions/51905268/how-to-find-closest-point-on-line
        public static Float2 FindNearestPointOnLine(Float2 origin, Float2 direction, Float2 point)
        {
            direction.Normalize();
            Float2 lhs = point - origin;

            float dotProduct = Float2.Dot(lhs, direction);
            return origin + direction * dotProduct;
        }

        // Based on https://gamedev.stackexchange.com/questions/172001/shortest-distance-to-chain-of-line-segments
        public static Float2 FindNearestPointOnLineSegment(Float2 start, Float2 end, Float2 point)
        {
            // Shift the problem to the origin to simplify the math.
            Float2 startToPoint = point - start;
            Float2 startToEnd = end - start;

            // Compute how far along the line is the closest approach to our point.
            float projectedDistance = Float2.Dot(startToPoint, startToEnd) / startToEnd.LengthSquared();

            // Restrict this point to within the line segment from start to end.
            projectedDistance = Mathf.Clamp01(projectedDistance);

            // Return this point.
            return start + startToEnd * projectedDistance;
        }

        // Based on https://stackoverflow.com/questions/51905268/how-to-find-closest-point-on-line
        public static Float2 FindNearestPointOnLineSegment2(Float2 start, Float2 end, Float2 point)
        {
            //Get heading
            Float2 startToEnd = end - start;
            float magnitudeMax = startToEnd.Length();
            startToEnd.Normalize();

            //Do projection from the point but clamp it
            Float2 startToPoint = point - start;
            float dotP = Float2.Dot(startToPoint, startToEnd);
            dotP = Mathf.Clamp(dotP, 0f, magnitudeMax);
            return start + startToEnd * dotP;
        }

        public static float Sqrt(float f)
        {
            return Mathf.Sqrt(f);
        }

        public static float Pow(float f, float p)
        {
            return Mathf.Pow(f, p);
        }

        public static float Sin(float f)
        {
            return Mathf.Sin(f);
        }

        public static float Cos(float f)
        {
            return Mathf.Cos(f);
        }

        public static int Min(params int[] values)
        {
            return Mathf.Min(values);
        }

        public static float Min(params float[] values)
        {
            return Mathf.Min(values);
        }

        public static int Max(params int[] values)
        {
            return Mathf.Max(values);
        }

        public static float Max(params float[] values)
        {
            return Mathf.Max(values);
        }

        public static int RoundToInt(float f)
        {
            return Mathf.RoundToInt(f);
        }

        public static int Clamp(int value, int min, int max)
        {
            return Mathf.Clamp(value, min, max);
        }

        public static int Abs(int value)
        {
            return Mathf.Abs(value);
        }

        public static float Abs(float f)
        {
            return Mathf.Abs(f);
        }

        public static float Clamp01(float value)
        {
            return Mathf.Clamp01(value);
        }

        public static float Floor(float v)
        {
            return Mathf.Floor(v);
        }

        public static float Deg2Rad()
        {
            return Mathf.Deg2Rad;
        }

        public static float Rad2Deg()
        {
            return Mathf.Rad2Deg;
        }
    }
}
