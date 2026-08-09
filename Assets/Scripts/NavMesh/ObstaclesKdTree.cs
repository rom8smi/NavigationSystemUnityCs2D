using System.Collections.Generic;
using GenericCode;

namespace TriangulationNavigation
{
    public class ObstaclesKdTree
    {
        public KDTree2D kdTree;
        public List<Float2> centers;
        float largestCornersDistance;

        public ObstaclesKdTree()
        {
            centers = new List<Float2>();
        }

        public void Build(List<Obstacle> obstacles)
        {
            int obstaclesCount = obstacles.Count;
            centers.Resize(obstaclesCount);

            largestCornersDistance = 0.0f;

            for (int i = 0; i < obstaclesCount; i++)
            {
                Float2 center = obstacles[i].center;
                centers[i] = center;
                largestCornersDistance = MathUtils.Max(largestCornersDistance, obstacles[i].largestCornerDistance);
            }

            if (obstaclesCount > 0)
            {
                kdTree = KDTree2D.MakeFromPoints(centers.ToArray());
            }
        }

        public bool Intersects(Obstacle obstacle, List<Obstacle> obstacles)
        {
            int cornersCount = obstacle.obstacleCorners.Count;
            int obstaclesCount = obstacles.Count;

            if (obstacles.Count == 0 || cornersCount == 0)
            {
                return false;
            }

            Float2 center = obstacle.center;
            float obstacleLargestCornerDistance = obstacle.largestCornerDistance;
            float distanceToSearch = largestCornersDistance + obstacleLargestCornerDistance + 0.1f;

            List<int> centerNeighbours = new List<int>();
            kdTree.FindNearestsBall(center, distanceToSearch, centerNeighbours);

            for (int i = 0; i < centerNeighbours.Count; i++)
            {
                int neighbourIndex = centerNeighbours[i];

                if (ObstacleUtils.AreObstaclesIntersecting(obstacle.obstacleCorners, obstacles[neighbourIndex].obstacleCorners))
                {
                    return true;
                }
            }

            return false;
        }

        bool intersects_linear(Obstacle obstacle, List<Obstacle> obstacles)
        {
            Float2 centerA = obstacle.center;
            float ra = obstacle.largestCornerDistance;

            for (int i = 0; i < obstacles.Count; i++)
            {
                Float2 centerB = obstacles[i].center;
                float rb = obstacles[i].largestCornerDistance;

                float centersDistance = (centerA - centerB).Length();
                float largestCenterDistance = ra + rb + 0.1f;

                if (centersDistance < largestCenterDistance)
                {
                    if (ObstacleUtils.AreObstaclesIntersecting(obstacle.obstacleCorners, obstacles[i].obstacleCorners))
                    {
                        return true;
                    }
                }
            }
            return false;
        }
    }
}
