using UnityEngine;
using GenericCode;
using System.Collections.Generic;
using System.Diagnostics;

namespace TriangulationNavigation
{
    public class UnionPolygonFinderTests : MonoBehaviour
    {
        void Start()
        {
            RunTestNoBoundsComparison();
        }

        void RunTestNoBoundsComparison()
        {
            RunTestNoBoundsComparison(1.0f);
            RunTestNoBoundsComparison(1.1f);
            RunTestNoBoundsComparison(1.2f);
            RunTestNoBoundsComparison(1.3f);
        }

        void RunTestNoBoundsComparison(float scale)
        {
            RunTestRegular(
                scale,
                out float runningTimeRegular,
                out List<List<Float2>> allCornersWithUnionRegular);
            
            RunTestNoBoundsCheck(
                scale,
                out float runningTimeNoBoundsCheck,
                out List<List<Float2>> allCornersWithUnionNoBoundsCheck);

            int correctness = AreTheSame(allCornersWithUnionRegular, allCornersWithUnionNoBoundsCheck);

            UnityEngine.Debug.Log(
                runningTimeRegular.ToString() + " " +
                runningTimeNoBoundsCheck.ToString() + " " +
                correctness.ToString()
            );
        }

        void RunTestRegular(float scale, out float runningTime, out List<List<Float2>> allCornersWithUnion)
        {
            List<List<Float2>> allCorners = CreateInitialCorners(scale);

            allCornersWithUnion = Copy(allCorners);

            Stopwatch sw = new Stopwatch();
            sw.Start();

            UnionPolygonFinder.FindUnionsForMultiplePolygons(allCornersWithUnion);

            runningTime = (float)sw.Elapsed.TotalMilliseconds;
        }

        void RunTestNoBoundsCheck(float scale, out float runningTime, out List<List<Float2>> allCornersWithUnion)
        {
            List<List<Float2>> allCorners = CreateInitialCorners(scale);

            allCornersWithUnion = Copy(allCorners);

            Stopwatch sw = new Stopwatch();
            sw.Start();

            UnionPolygonFinder.FindUnionsForMultiplePolygonsNoBoundsCheck(allCornersWithUnion);

            runningTime = (float)sw.Elapsed.TotalMilliseconds;
        }

        List<List<Float2>> CreateInitialCorners(float scale)
        {
            int numberToSpawn = (int)(40.0f * scale * scale);
            float worldHalfSize = 50.0f * scale;

            float minSize = 0.3f;
            float maxSize = 9.3f;

            ManualRandom random = new ManualRandom(0);
            List<List<Float2>> allCorners = new List<List<Float2>>();

            for (int i = 0; i < numberToSpawn; i++)
            {
                float x = random.next_float(-worldHalfSize, worldHalfSize);
                float y = random.next_float(-worldHalfSize, worldHalfSize);

                float dx = random.next_float(minSize, maxSize);
                float dy = random.next_float(minSize, maxSize);

                RectangularObstacle rectangularObstacle = new RectangularObstacle
                {
                    center = new Float2(x, y),
                    size = new Float2(dx, dy),
                    rotation = 0.0f,
                    radius = 1.0f
                };

                List<Float2> corners = rectangularObstacle.GetCorners();
                allCorners.Add(corners);
            }

            return allCorners;
        }

        List<List<Float2>> Copy(List<List<Float2>> input)
        {
            List<List<Float2>> output = new List<List<Float2>>();
            output.Resize(input.Count);

            for (int i = 0; i < input.Count; i++)
            {
                List<Float2> innerOutput = new List<Float2>();
                innerOutput.Resize(input[i].Count);
                output[i] = innerOutput;
            }

            for (int i = 0; i < input.Count; i++)
            {
                for (int j = 0; j < input[i].Count; j++)
                {
                    output[i][j] = input[i][j];
                }
            }

            return output;
        }

        int AreTheSame(List<List<Float2>> inputA, List<List<Float2>> inputB)
        {
            if (inputA.Count != inputB.Count)
            {
                return 1;
            }
            for (int i = 0; i < inputA.Count; i++)
            {
                if (inputA[i].Count != inputB[i].Count)
                {
                    return 2;
                }
            }

            float epsilon = 0.0001f;
            for (int i = 0; i < inputA.Count; i++)
            {
                for (int j = 0; j < inputA[i].Count; j++)
                {
                    float diff = (inputA[i][j] - inputB[i][j]).Length();
                    if (diff > epsilon)
                    {
                        return 3;
                    }
                }
            }

            return 0;
        }
    }
}
