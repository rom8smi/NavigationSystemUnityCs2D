using System.Collections.Generic;

namespace GenericCode
{
    public static class DuplicateUtils
    {
        public static int FindDuplicatesCount(List<Float2> points, float epsilon)
        {
            int count = 0;
            float epsilonSqr = epsilon * epsilon;
            List<bool> considered = new List<bool>();
            int pointsCount = points.Count;
            considered.Resize(pointsCount);

            for (int i = 0; i < pointsCount; i++)
            {
                considered[i] = false;
            }

            for (int i = 0; i < pointsCount; i++)
            {
                for (int j = i + 1; j < pointsCount; j++)
                {
                    if (!considered[j])
                    {
                        float rSqr = (points[i] - points[j]).LengthSquared();
                        if (rSqr < epsilonSqr)
                        {
                            considered[j] = true;
                            count++;
                        }
                    }
                }
            }

            return count;
        }

        public static int FindDuplicatesCountKdTree(List<Float2> points, float epsilon)
        {
            KDTree2D kdTree = KDTree2D.MakeFromPoints(points.ToArray());

            int count = 0;
            float epsilonSqr = epsilon * epsilon;
            List<bool> considered = new List<bool>();

            int pointsCount = points.Count;

            considered.Resize(pointsCount);

            for (int i = 0; i < pointsCount; i++)
            {
                considered[i] = false;
            }

            List<int> neighbours = new List<int>();

            for (int i = 0; i < pointsCount; i++)
            {
                neighbours.Clear();
                kdTree.FindNearestsBall(points[i], 2.0f * epsilon, neighbours);

                for (int j = 0; j < neighbours.Count; j++)
                {
                    int neighbour = neighbours[j];
                    if (neighbour > i && !considered[neighbour])
                    {
                        float rSqr = (points[i] - points[neighbour]).LengthSquared();
                        if (rSqr < epsilonSqr)
                        {
                            considered[neighbour] = true;
                            count++;
                        }
                    }
                }
            }

            return count;
        }

        public static void RemoveDuplicates(List<Float2> points, float epsilon)
        {
            KDTree2D kdTree = KDTree2D.MakeFromPoints(points.ToArray());

            float epsilonSqr = epsilon * epsilon;
            List<bool> duplicates = new List<bool>();

            int pointsCount = points.Count;

            duplicates.Resize(pointsCount);

            for (int i = 0; i < pointsCount; i++)
            {
                duplicates[i] = false;
            }

            List<int> neighbours = new List<int>();

            for (int i = 0; i < pointsCount; i++)
            {
                neighbours.Clear();
                kdTree.FindNearestsBall(points[i], 2.0f * epsilon, neighbours);

                for (int j = 0; j < neighbours.Count; j++)
                {
                    int neighbour = neighbours[j];
                    if (neighbour > i && !duplicates[neighbour])
                    {
                        float rSqr = (points[i] - points[neighbour]).LengthSquared();
                        if (rSqr < epsilonSqr)
                        {
                            duplicates[neighbour] = true;
                        }
                    }
                }
            }

            int iNew = 0;
            for (int i = 0; i < pointsCount; i++)
            {
                if (!duplicates[i])
                {
                    points[iNew] = points[i];
                    iNew++;
                }
            }
            points.Resize(iNew);
        }
    }
}
