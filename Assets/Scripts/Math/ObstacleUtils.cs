using System.Collections.Generic;
using TriangulationNavigation;

namespace GenericCode
{
    public static class ObstacleUtils
    {
        public static Obstacle Create(
            List<Float2> p_obstacleCorners,
            AABB p_worldBounds,
            float clip_line_length,
            bool p_isWalkable)
        {
            Obstacle obstacle = new Obstacle();
            obstacle.obstacleCorners = p_obstacleCorners;
            obstacle.isWalkable = p_isWalkable;
            int minSplits = 0;

            obstacle.isCornerIntersectingWithWorldBounds = new List<bool>();
            obstacle.nSplits = new List<int>();

            for (int i = 0; i < obstacle.obstacleCorners.Count; i++)
            {
                obstacle.isCornerIntersectingWithWorldBounds.Add(false);
            }

            Clip(obstacle.obstacleCorners, obstacle.isCornerIntersectingWithWorldBounds, p_worldBounds.minY, 0.0f, clip_line_length);
            Clip(obstacle.obstacleCorners, obstacle.isCornerIntersectingWithWorldBounds, -p_worldBounds.maxY, MathUtils.PI, clip_line_length);
            Clip(obstacle.obstacleCorners, obstacle.isCornerIntersectingWithWorldBounds, p_worldBounds.minX, -0.5f * MathUtils.PI, clip_line_length);
            Clip(obstacle.obstacleCorners, obstacle.isCornerIntersectingWithWorldBounds, -p_worldBounds.maxX, 0.5f * MathUtils.PI, clip_line_length);

            int obstacleCornersCount = obstacle.obstacleCorners.Count;

            for (int i = 0; i < obstacle.obstacleCorners.Count; i++)
            {
                obstacle.nSplits.Add(minSplits);
            }

            obstacle.center = new Float2(0.0f, 0.0f);
            for (int i = 0; i < obstacleCornersCount; i++)
            {
                obstacle.center += obstacle.obstacleCorners[i];
            }
            obstacle.center = obstacle.center / obstacleCornersCount;

            float largest_corner_distance_sqr = 0.0f;
            for (int i = 0; i < obstacleCornersCount; i++)
            {
                float corner_distance_sqr = (obstacle.obstacleCorners[i] - obstacle.center).LengthSquared();
                if(corner_distance_sqr > largest_corner_distance_sqr)
                {
                    largest_corner_distance_sqr = corner_distance_sqr;
                }
            }
            obstacle.largestCornerDistance = MathUtils.Sqrt(largest_corner_distance_sqr);

            return obstacle;
        }

        static void Clip(
            List<Float2> p_obstacleCorners,
            List<bool> p_isCornerIntersectingWithWorldBounds,
            float minX,
            float rotation,
            float clip_line_length)
        {
            List<Float2> potentialCorners = new List<Float2>();
            List<bool> potentialIntersectionPoints = new List<bool>();
            List<bool> potentialOldIntersectionPoints = new List<bool>();

            int obstacleCornersCount = p_obstacleCorners.Count;

            for (int i = 0; i < obstacleCornersCount; i++)
            {
                int i1 = i;
                int i2 = i1 + 1;
                if (i2 >= obstacleCornersCount)
                {
                    i2 -= obstacleCornersCount;
                }

                Float2 p1 = p_obstacleCorners[i1];
                Float2 p2 = p_obstacleCorners[i2];

                Float2 p1rotated = VectorUtils.Rotate(p1, rotation);
                Float2 p2rotated = VectorUtils.Rotate(p2, rotation);

                bool isIntersecting = p_isCornerIntersectingWithWorldBounds[i1];

                Float2 left = new Float2(minX, -clip_line_length);
                Float2 right = new Float2(minX, clip_line_length);

                LineSegmentsIntersectionResult lineSegmentsIntersectionResult = VectorUtils.LineSegmentsIntersection(
                    p1rotated,
                    p2rotated,
                    left,
                    right
                );

                if (lineSegmentsIntersectionResult.intersects)
                {
                    if (p1rotated.x > minX)
                    {
                        potentialCorners.Add(p1rotated);
                        potentialIntersectionPoints.Add(isIntersecting);
                        potentialOldIntersectionPoints.Add(true);

                        potentialCorners.Add(lineSegmentsIntersectionResult.intersection);
                        potentialIntersectionPoints.Add(true);
                        potentialOldIntersectionPoints.Add(false);
                    }
                    else if (p1rotated.x < minX)
                    {
                        potentialCorners.Add(lineSegmentsIntersectionResult.intersection);
                        potentialIntersectionPoints.Add(true);
                        potentialOldIntersectionPoints.Add(false);

                        potentialCorners.Add(p1rotated);
                        potentialIntersectionPoints.Add(isIntersecting);
                        potentialOldIntersectionPoints.Add(true);
                    }
                    else
                    {
                        potentialCorners.Add(p1rotated);
                        potentialIntersectionPoints.Add(isIntersecting);
                        potentialOldIntersectionPoints.Add(true);
                    }
                }
                else
                {
                    potentialCorners.Add(p1rotated);
                    potentialIntersectionPoints.Add(isIntersecting);
                    potentialOldIntersectionPoints.Add(true);
                }
            }

            p_obstacleCorners.Clear();
            p_isCornerIntersectingWithWorldBounds.Clear();

            for (int i = 0; i < potentialCorners.Count; i++)
            {
                if (potentialCorners[i].x > minX || (potentialIntersectionPoints[i] && !potentialOldIntersectionPoints[i]))
                {
                    Float2 corner = VectorUtils.Rotate(potentialCorners[i], -rotation);
                    p_obstacleCorners.Add(corner);
                    p_isCornerIntersectingWithWorldBounds.Add(potentialIntersectionPoints[i]);
                }
            }
        }

        public static bool AreObstaclesIntersecting(List<Float2> cornersA, List<Float2> cornersB)
        {
            int cornersACount = cornersA.Count;

            for (int i = 0; i < cornersACount; i++)
            {
                if (VectorUtils.IsPointInPolygon(cornersA[i], cornersB))
                {
                    return true;
                }
            }

            int cornersBCount = cornersB.Count;

            for (int i = 0; i < cornersBCount; i++)
            {
                if (VectorUtils.IsPointInPolygon(cornersB[i], cornersA))
                {
                    return true;
                }
            }

            for (int i = 0; i < cornersACount; i++)
            {
                int iNext = i + 1;
                if (iNext == cornersACount)
                {
                    iNext = 0;
                }
                Float2 pa = cornersA[i];
                Float2 qa = cornersA[iNext];

                for (int j = 0; j < cornersBCount; j++)
                {
                    int jNext = j + 1;
                    if (jNext == cornersBCount)
                    {
                        jNext = 0;
                    }
                    Float2 pb = cornersB[j];
                    Float2 qb = cornersB[jNext];

                    if (VectorUtils.AreLineSegmentsIntersecting(pa, qa, pb, qb))
                    {
                        return true;
                    }
                }
            }

            return false;
        }
    }
}
