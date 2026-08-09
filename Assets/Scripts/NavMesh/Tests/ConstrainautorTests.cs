using UnityEngine;
using GenericCode;
using System.Collections.Generic;
using System.Diagnostics;
using DelaunatorSharp;

namespace TriangulationNavigation
{
    public class ConstrainautorTests : MonoBehaviour
    {
        List<Float2> delaunatorPoints;
        List<DelaunatorSharp.Triangle> delaunatorTriangles;
        List<Float2> delaunatorPointsCon;
        List<DelaunatorSharp.Triangle> delaunatorTrianglesCon;
        GizmoDrawer gizmoDrawer;

        void Start()
        {
            gizmoDrawer = new GizmoDrawer();
            Delaunator();
        }

        void Delaunator()
        {
            delaunatorPoints = new List<Float2>{
                new Float2(0.0f, -15.0f),
                new Float2(-10.0f, 0.0f),
                new Float2(0.0f, 15.0f),
                new Float2(10.0f, 0.0f)
            };
            delaunatorPointsCon = new List<Float2>{
                new Float2(0.0f, -15.0f),
                new Float2(-10.0f, 0.0f),
                new Float2(0.0f, 15.0f),
                new Float2(10.0f, 0.0f)
            };

            for (int i = 0; i < delaunatorPoints.Count; i++)
            {
                delaunatorPoints[i] += new Float2(-15.0f, 0.0f);
                delaunatorPointsCon[i] += new Float2(15.0f, 0.0f);
            }

            Delaunator delaunator = new Delaunator();
            delaunator.Create(delaunatorPoints);
            delaunatorTriangles = delaunator.GetTriangles();

            Delaunator delaunatorCon = new Delaunator();
            delaunatorCon.Create(delaunatorPointsCon);

            List<ConstraintEdge> edges = new List<ConstraintEdge>
            {
                new ConstraintEdge{
                    p = 0,
                    q = 2
                }
            };
            Constrainautor constrainautor = new Constrainautor();
            constrainautor.Create(delaunatorCon, edges);

            delaunatorTrianglesCon = delaunatorCon.GetTriangles();
        }

        void OnDrawGizmos()
        {
            if (!Application.isPlaying)
            {
                return;
            }

            if (gizmoDrawer != null)
            {
                gizmoDrawer.DrawDelaunator(delaunatorPoints, delaunatorTriangles);
                gizmoDrawer.DrawDelaunator(delaunatorPointsCon, delaunatorTrianglesCon);
            }
        }
    }
}
