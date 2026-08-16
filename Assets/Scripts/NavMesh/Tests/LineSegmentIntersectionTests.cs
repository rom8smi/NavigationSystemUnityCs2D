using UnityEngine;
using GenericCode;
using System.Collections.Generic;
using System.Diagnostics;

namespace TriangulationNavigation
{
    public class LineSegmentIntersectionTests : MonoBehaviour
    {
        IntersectionsData drawableIntersectionsData;
        ManualRandom random = new ManualRandom(0);

        void Start()
        {
            drawableIntersectionsData = new IntersectionsData();
            random = new ManualRandom(0);

            InitDrawableData();

            for (int i = 0; i < 20; i++)
            {
                InitRandomDataStress(1.0f + i * 0.2f);
            }
        }

        void InitDrawableData()
        {
            drawableIntersectionsData.dataPoints.Add(new Float2(0, 0));
            drawableIntersectionsData.dataPoints.Add(new Float2(10, 10));
            drawableIntersectionsData.dataPoints.Add(new Float2(10, 0));
            drawableIntersectionsData.dataPoints.Add(new Float2(0, 10));

            drawableIntersectionsData.dataPoints.Add(new Float2(20, 0));
            drawableIntersectionsData.dataPoints.Add(new Float2(30, 0));
            drawableIntersectionsData.dataPoints.Add(new Float2(30, 0));
            drawableIntersectionsData.dataPoints.Add(new Float2(30, 10));

            drawableIntersectionsData.segmentStarts.Add(0);
            drawableIntersectionsData.segmentEnds.Add(1);
            drawableIntersectionsData.segmentStarts.Add(2);
            drawableIntersectionsData.segmentEnds.Add(3);

            drawableIntersectionsData.segmentStarts.Add(4);
            drawableIntersectionsData.segmentEnds.Add(5);
            drawableIntersectionsData.segmentStarts.Add(6);
            drawableIntersectionsData.segmentEnds.Add(7);

            // LineSegmentUtils.GetAllIntersectionsSimple(
            //     drawableIntersectionsData.dataPoints,
            //     drawableIntersectionsData.segmentStarts,
            //     drawableIntersectionsData.segmentEnds,
            //     drawableIntersectionsData.intersectionPoints);
            LineSegmentUtils.GetAllIntersectionsKdTreeWithRandomOffset(
                drawableIntersectionsData.dataPoints,
                drawableIntersectionsData.segmentStarts,
                drawableIntersectionsData.segmentEnds,
                drawableIntersectionsData.intersectionPoints,
                drawableIntersectionsData.firstIntersectionSegments,
                drawableIntersectionsData.secondIntersectionSegments);
        }

        void InitRandomDataStress(float multiplier)
        {
            int n = (int)(80.0f * multiplier * multiplier);
            IntersectionsData randomIntersectionsData = new IntersectionsData();

            for (int i = 0; i < n; i++)
            {   
                Float2 center = VectorUtils.RangomInsideUnitCircle(random) * 40.0f * multiplier;
                Float2 offset = VectorUtils.RangomInsideUnitCircle(random).Normalized() * 5f;

                Float2 start = center - offset;
                Float2 end = center + offset;

                int currentCount = randomIntersectionsData.dataPoints.Count;

                randomIntersectionsData.dataPoints.Add(start);
                randomIntersectionsData.dataPoints.Add(end);

                randomIntersectionsData.segmentStarts.Add(currentCount);
                randomIntersectionsData.segmentEnds.Add(currentCount + 1);
            }

            Stopwatch sw1 = new Stopwatch();
            sw1.Start();
            LineSegmentUtils.GetAllIntersectionsSimple(
                randomIntersectionsData.dataPoints,
                randomIntersectionsData.segmentStarts,
                randomIntersectionsData.segmentEnds,
                randomIntersectionsData.intersectionPoints,
                randomIntersectionsData.firstIntersectionSegments,
                randomIntersectionsData.secondIntersectionSegments);
            float t1 = (float)sw1.Elapsed.TotalMilliseconds;

            int count1 = randomIntersectionsData.intersectionPoints.Count;
            List<Float2> simpleIntersectionPoints = new List<Float2>();
            simpleIntersectionPoints.Resize(count1);
            for (int i = 0; i < count1; i++)
            {
                simpleIntersectionPoints[i] = randomIntersectionsData.intersectionPoints[i];
            }
            randomIntersectionsData.intersectionPoints.Clear();

            Stopwatch sw2 = new Stopwatch();
            sw2.Start();
            LineSegmentUtils.GetAllIntersectionsKdTreeWithRandomOffset(
                randomIntersectionsData.dataPoints,
                randomIntersectionsData.segmentStarts,
                randomIntersectionsData.segmentEnds,
                randomIntersectionsData.intersectionPoints,
                randomIntersectionsData.firstIntersectionSegments,
                randomIntersectionsData.secondIntersectionSegments);
            float t2 = (float)sw2.Elapsed.TotalMilliseconds;

            int count2 = randomIntersectionsData.intersectionPoints.Count;

            int failedMatches = 0;
            float epsilon = 0.001f;

            if (count1 == count2)
            {
                for (int i = 0; i < count1; i++)
                {
                    if ((simpleIntersectionPoints[i] - randomIntersectionsData.intersectionPoints[i]).LengthSquared() > epsilon)
                    {
                        failedMatches++;
                    }
                }
            }

            randomIntersectionsData.intersectionPoints.Clear();

            GenericCode.Debug.Log($"{n} | {count1} {count2} | {t1} {t2} | {count1 == count2} {failedMatches}");
        }

        void OnDrawGizmos()
        {
            if (drawableIntersectionsData == null)
            {
                return;
            }

            int nSegments = drawableIntersectionsData.segmentStarts.Count;

            Gizmos.color = Color.green;

            for (int i = 0; i < nSegments; i++)
            {
                int istart = drawableIntersectionsData.segmentStarts[i];
                int iend = drawableIntersectionsData.segmentEnds[i];

                Vector3 start = GizmoDrawer.ToVector3(drawableIntersectionsData.dataPoints[istart]);
                Vector3 end = GizmoDrawer.ToVector3(drawableIntersectionsData.dataPoints[iend]);

                Gizmos.DrawSphere(start, 0.3f);
                Gizmos.DrawSphere(end, 0.3f);
            }

            Gizmos.color = Color.black;

            for (int i = 0; i < nSegments; i++)
            {
                int istart = drawableIntersectionsData.segmentStarts[i];
                int iend = drawableIntersectionsData.segmentEnds[i];

                Vector3 start = GizmoDrawer.ToVector3(drawableIntersectionsData.dataPoints[istart]);
                Vector3 end = GizmoDrawer.ToVector3(drawableIntersectionsData.dataPoints[iend]);

                Gizmos.DrawLine(start, end);
            }

            Gizmos.color = Color.red;

            for (int i = 0; i < drawableIntersectionsData.intersectionPoints.Count; i++)
            {
                Vector3 intersectionPoint = GizmoDrawer.ToVector3(drawableIntersectionsData.intersectionPoints[i]);
                Gizmos.DrawSphere(intersectionPoint, 0.3f);
            }
        }
    }

    public class IntersectionsData
    {
        public List<Float2> dataPoints = new List<Float2>();
        public List<int> segmentStarts = new List<int>();
        public List<int> segmentEnds = new List<int>();
        public List<Float2> intersectionPoints = new List<Float2>();
        public List<int> firstIntersectionSegments = new List<int>();
        public List<int> secondIntersectionSegments = new List<int>();
    }
}
