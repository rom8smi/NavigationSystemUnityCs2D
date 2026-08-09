using UnityEngine;
using GenericCode;
using System.Collections.Generic;
using DelaunatorSharp;

namespace TriangulationNavigation
{
    public class LineSegmentIntersectionTwoTests : MonoBehaviour
    {
        List<ConstraintEdge> constraintEdges;
        List<Float2> newPoints;

        float t = 0f;
        int drawIndex = 0;

        void Start()
        {
            InitData();
        }

        void InitData()
        {
            List<Obstacle> obstacles1 = new List<Obstacle>()
            {
                new Obstacle{
                    obstacleCorners = new List<Float2>{
                        new Float2(0f, 0f),
                        new Float2(1f, 0f),
                        new Float2(1f, 1f),
                        new Float2(0f, 1f),
                    },
                    isCornerIntersectingWithWorldBounds = new List<bool>{false, false, false, false}
                },
                new Obstacle{
                    obstacleCorners = new List<Float2>{
                        new Float2(0f, 0f) + new Float2(0.7f, 0.7f),
                        new Float2(1f, 0f) + new Float2(0.7f, 0.7f),
                        new Float2(1f, 1f) + new Float2(0.7f, 0.7f),
                        new Float2(0f, 1f) + new Float2(0.7f, 0.7f),
                    },
                    isCornerIntersectingWithWorldBounds = new List<bool>{false, false, false, false}
                }
            };
            constraintEdges = new List<ConstraintEdge>();
            newPoints = new List<Float2>();

            NavMesh navMesh = new NavMesh();
            List<int> newObstacleWalkablityIndices = new List<int>();
            List<List<int>> obstacleIntersections = new List<List<int>>();
            List<bool> newIsObstacleCornerIntersectingWithWorldBounds = new List<bool>();
            navMesh.AddObstaclesWithConstraints(
                obstacles1,
                constraintEdges,
                newPoints,
                newObstacleWalkablityIndices,
                obstacleIntersections,
                newIsObstacleCornerIntersectingWithWorldBounds);
        }

        void Update()
        {
            t += RuntimeConstants.deltaTime;
            if (t > 0.5f)
            {
                t = 0f;
                drawIndex++;
                if (drawIndex > constraintEdges.Count)
                {
                    drawIndex = 0;
                }
            }
        }

        void OnDrawGizmos()
        {
            if (constraintEdges != null && newPoints != null)
            {
                Gizmos.color = Color.green;

                for (int i = 0; i < newPoints.Count; i++)
                {
                    Vector3 p = GizmoDrawer.ToVector3(newPoints[i]);
                    Gizmos.DrawSphere(p, 0.03f);
                }

                Gizmos.color = Color.black;

                for (int i = 0; i < constraintEdges.Count; i++)
                {
                    if (i < drawIndex)
                    {
                        int istart = constraintEdges[i].p;
                        int iend = constraintEdges[i].q;

                        Vector3 start = GizmoDrawer.ToVector3(newPoints[istart]);
                        Vector3 end = GizmoDrawer.ToVector3(newPoints[iend]);

                        Gizmos.DrawLine(start, end);
                    }
                }
            }
        }
    }
}
