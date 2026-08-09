using System.Collections.Generic;

namespace GenericCode
{
    public static class VectorUtils
    {
        public static Float2 AdjustForBoundaries(Float2 p_position, float minX, float maxX, float minY, float maxY, float epsilon)
        {
            Float2 position = p_position;

            if (position.x < minX + epsilon)
            {
                position.x = minX + epsilon;
            }
            if (position.x > maxX - epsilon)
            {
                position.x = maxX - epsilon;
            }

            if (position.y < minY + epsilon)
            {
                position.y = minY + epsilon;
            }
            if (position.y > maxY - epsilon)
            {
                position.y = maxY - epsilon;
            }

            return position;
        }

        public static Float2 AdjustForBoundaries(Float2 p_position, float minX, float maxX, float minY, float maxY, float epsilon, ref bool wasAdjusted)
        {
            Float2 position = p_position;
            wasAdjusted = false;

            if (position.x < minX + epsilon)
            {
                position.x = minX + epsilon;
                wasAdjusted = true;
            }
            if (position.x > maxX - epsilon)
            {
                position.x = maxX - epsilon;
                wasAdjusted = true;
            }

            if (position.y < minY + epsilon)
            {
                position.y = minY + epsilon;
                wasAdjusted = true;
            }
            if (position.y > maxY - epsilon)
            {
                position.y = maxY - epsilon;
                wasAdjusted = true;
            }

            return position;
        }

        // Based on https://github.com/setchi/Unity-LineSegmentsIntersection
        public static LineSegmentsIntersectionResult LineSegmentsIntersection(Float2 a1, Float2 a2, Float2 b1, Float2 b2)
        {
            Float2 intersection = new Float2(0.0f, 0.0f);

            float d = (a2.x - a1.x) * (b2.y - b1.y) - (a2.y - a1.y) * (b2.x - b1.x);

            if (d == 0.0f)
            {
                return new LineSegmentsIntersectionResult
                {
                    intersects = false,
                    intersection = intersection
                };
            }

            float u = ((b1.x - a1.x) * (b2.y - b1.y) - (b1.y - a1.y) * (b2.x - b1.x)) / d;
            float v = ((b1.x - a1.x) * (a2.y - a1.y) - (b1.y - a1.y) * (a2.x - a1.x)) / d;

            if (u < 0.0f || u > 1.0f || v < 0.0f || v > 1.0f)
            {
                return new LineSegmentsIntersectionResult
                {
                    intersects = false,
                    intersection = intersection
                };
            }

            intersection.x = a1.x + u * (a2.x - a1.x);
            intersection.y = a1.y + u * (a2.y - a1.y);

            return new LineSegmentsIntersectionResult
            {
                intersects = true,
                intersection = intersection
            };
        }

        public static LineSegmentsIntersectionResult LineSegmentsIntersection(Float2 a1, Float2 a2, Float2 b1, Float2 b2, float epsilon)
        {
            Float2 intersection = new Float2(0.0f, 0.0f);

            float d = (a2.x - a1.x) * (b2.y - b1.y) - (a2.y - a1.y) * (b2.x - b1.x);

            if (d < epsilon && d > -epsilon)
            {
                return new LineSegmentsIntersectionResult
                {
                    intersects = false,
                    intersection = intersection
                };
            }

            float u = ((b1.x - a1.x) * (b2.y - b1.y) - (b1.y - a1.y) * (b2.x - b1.x)) / d;
            float v = ((b1.x - a1.x) * (a2.y - a1.y) - (b1.y - a1.y) * (a2.x - a1.x)) / d;

            if (u < epsilon || u > 1.0f - epsilon || v < epsilon || v > 1.0f - epsilon)
            {
                return new LineSegmentsIntersectionResult
                {
                    intersects = false,
                    intersection = intersection
                };
            }

            intersection.x = a1.x + u * (a2.x - a1.x);
            intersection.y = a1.y + u * (a2.y - a1.y);

            return new LineSegmentsIntersectionResult
            {
                intersects = true,
                intersection = intersection
            };
        }

        public static bool AreLineSegmentsCollinearAndOverlapping(Float2 ap, Float2 aq, Float2 bp, Float2 bq, float epsilon)
        {
            return
                IsPointCollinearToLineSegment(bp, ap, aq, epsilon) &&
                IsPointCollinearToLineSegment(bq, ap, aq, epsilon) &&
                (IsPointWithinLineSegment(bp, ap, aq, epsilon) ||
                IsPointWithinLineSegment(bq, ap, aq, epsilon));
        }

        // Based on https://stackoverflow.com/questions/7050186/find-if-point-lies-on-line-segment
        public static bool PointOnLine2D(Float2 p, Float2 a, Float2 b, float epsilon)
        {
            return IsPointCollinearToLineSegment(p, a, b, epsilon) && IsPointWithinLineSegment(p, a, b, epsilon);
        }

        public static bool IsPointCollinearToLineSegment(Float2 p, Float2 a, Float2 b, float epsilon)
        {
            // ensure points are collinear
            float zero = (b.x - a.x) * (p.y - a.y) - (p.x - a.x) * (b.y - a.y);
            if (zero > epsilon || zero < -epsilon)
            {
                return false;
            }
            return true;
        }

        public static bool IsPointWithinLineSegment(Float2 p, Float2 a, Float2 b, float epsilon)
        {
            // check if x-coordinates are not equal
            if (a.x - b.x > epsilon || b.x - a.x > epsilon)
            {
                // ensure x is between a.x & b.x (use tolerance)
                return a.x > b.x
                    ? p.x + epsilon > b.x && p.x - epsilon < a.x
                    : p.x + epsilon > a.x && p.x - epsilon < b.x;
            }

            // ensure y is between a.y & b.y (use tolerance)
            return a.y > b.y
                ? p.y + epsilon > b.y && p.y - epsilon < a.y
                : p.y + epsilon > a.y && p.y - epsilon < b.y;
        }

        // Based on https://github.com/setchi/Unity-LineSegmentsIntersection
        public static bool AreLineSegmentsIntersecting(in Float2 a1, in Float2 a2, in Float2 b1, in Float2 b2)
        {
            float a1x = a1.x;
            float a1y = a1.y;
            float a2x = a2.x;
            float a2y = a2.y;
            float b1x = b1.x;
            float b1y = b1.y;
            float b2x = b2.x;
            float b2y = b2.y;

            float a2x_a1x = a2x - a1x;
            float b2y_b1y = b2y - b1y;
            float a2y_a1y = a2y - a1y;
            float b2x_b1x = b2x - b1x;

            float d = a2x_a1x * b2y_b1y - a2y_a1y * b2x_b1x;

            if (d == 0.0f)
            {
                return false;
            }

            float b1x_a1x = b1x - a1x;
            float b1y_a1y = b1y - a1y;

            float u = (b1x_a1x * b2y_b1y - b1y_a1y * b2x_b1x) / d;

            if (u < 0.0f || u > 1.0f)
            {
                return false;
            }

            float v = (b1x_a1x * a2y_a1y - b1y_a1y * a2x_a1x) / d;

            if (v < 0.0f || v > 1.0f)
            {
                return false;
            }

            return true;
        }

        // Based on https://stackoverflow.com/questions/2049582/how-to-determine-if-a-point-is-in-a-2d-triangle
        public static bool PointInTriangle(Float2 p, Float2 p0, Float2 p1, Float2 p2)
        {
            float s = (p0.x - p2.x) * (p.y - p2.y) - (p0.y - p2.y) * (p.x - p2.x);
            float t = (p1.x - p0.x) * (p.y - p0.y) - (p1.y - p0.y) * (p.x - p0.x);

            if ((s < 0) != (t < 0) && s != 0 && t != 0)
                return false;

            float d = (p2.x - p1.x) * (p.y - p1.y) - (p2.y - p1.y) * (p.x - p1.x);
            return d == 0 || (d < 0) == (s + t <= 0);
        }

        // Based on https://forum.unity.com/threads/whats-the-best-way-to-rotate-a-vector2-in-unity.729605/
        public static Float2 Rotate(Float2 v, float delta)
        {
            return new Float2(
                v.x * MathUtils.Cos(delta) - v.y * MathUtils.Sin(delta),
                v.x * MathUtils.Sin(delta) + v.y * MathUtils.Cos(delta)
            );
        }

        // Based on https://stackoverflow.com/questions/217578/how-can-i-determine-whether-a-2d-point-is-within-a-polygon
        public static bool IsPointInPolygon(Float2 p, List<Float2> polygon)
        {
            float minX = polygon[0].x;
            float maxX = polygon[0].x;
            float minY = polygon[0].y;
            float maxY = polygon[0].y;

            for (int i = 1; i < polygon.Count; i++)
            {
                Float2 q = polygon[i];
                minX = MathUtils.Min(q.x, minX);
                maxX = MathUtils.Max(q.x, maxX);
                minY = MathUtils.Min(q.y, minY);
                maxY = MathUtils.Max(q.y, maxY);
            }

            if (p.x < minX || p.x > maxX || p.y < minY || p.y > maxY)
            {
                return false;
            }

            // https://wrf.ecse.rpi.edu/Research/Short_Notes/pnpoly.html
            bool inside = false;
            for (int i = 0, j = polygon.Count - 1; i < polygon.Count; j = i++)
            {
                if ((polygon[i].y > p.y) != (polygon[j].y > p.y) &&
                    p.x < (polygon[j].x - polygon[i].x) * (p.y - polygon[i].y) / (polygon[j].y - polygon[i].y) + polygon[i].x)
                {
                    inside = !inside;
                }
            }

            return inside;
        }

        // Based on https://gamedev.stackexchange.com/questions/70075/how-can-i-find-the-perpendicular-to-a-2d-vector
        public static Float2 PerpendicularCounterClockwise(Float2 v)
        {
            return new Float2(-v.y, v.x);
        }

        // Based on https://stackoverflow.com/questions/49678042/get-random-point-on-a-unit-circle-circle-at-0-0
        public static Float2 RangomOnUnitCircle(ManualRandom random)
        {
            var a = random.next_float(0.0f, 1.0f) * (2 * MathUtils.PI) - MathUtils.PI;
            var x = MathUtils.Cos(a);
            var y = MathUtils.Sin(a);

            return new Float2(x, y);
        }

        // Based on https://stackoverflow.com/questions/5837572/generate-a-random-point-within-a-circle-uniformly
        public static Float2 RangomInsideUnitCircle(ManualRandom random)
        {
            float r = MathUtils.Sqrt(random.next_float(0.0f, 1.0f));
            float theta = random.next_float(0.0f, 1.0f) * 2.0f * MathUtils.PI;

            float x = r * MathUtils.Cos(theta);
            float y = r * MathUtils.Sin(theta);

            return new Float2(x, y);
        }
    }
}
