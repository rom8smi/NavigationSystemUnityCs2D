using UnityEngine;
using GenericCode;
using System.Collections.Generic;

namespace TriangulationNavigation
{
    public class KdTreeSearchBallTests : MonoBehaviour
    {
        void Start()
        {
            FindDuplicatesTest();
        }

        void FindDuplicatesTest()
        {
            ManualRandom random = new ManualRandom(0);

            List<Float2> points = new List<Float2>();

            int nPoints = 1000;
            points.Resize(nPoints);

            for (int i = 0; i < nPoints; i++)
            {
                points[i] = VectorUtils.RangomInsideUnitCircle(random) * 10.0f;
            }

            int nDuplicates = 200;
            for (int i = 0; i < nDuplicates; i++)
            {
                int duplicateIndex = random.next_int(i, nPoints - 1);

                points.Add(points[duplicateIndex]);
            }

            RandomUtils.Shuffle(points, random);

            float epsilon = 0.0000001f;
            int nDuplicatesFoundBeforeRemoval1 = DuplicateUtils.FindDuplicatesCount(points, epsilon);
            int nDuplicatesFoundBeforeRemoval2 = DuplicateUtils.FindDuplicatesCountKdTree(points, epsilon);

            DuplicateUtils.RemoveDuplicates(points, epsilon);

            int nDuplicatesFoundAfterRemoval1 = DuplicateUtils.FindDuplicatesCount(points, epsilon);
            int nDuplicatesFoundAfterRemoval2 = DuplicateUtils.FindDuplicatesCountKdTree(points, epsilon);

            GenericCode.Debug.Log(
                $"FindDuplicatesTest: {nDuplicatesFoundBeforeRemoval1} {nDuplicatesFoundBeforeRemoval2} |" +
                $" {nDuplicatesFoundAfterRemoval1} {nDuplicatesFoundAfterRemoval2} |" +
                $" {nDuplicates}");
        }
    }
}
