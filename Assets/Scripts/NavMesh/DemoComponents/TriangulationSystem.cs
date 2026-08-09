using System.Collections.Generic;
using BowyerWatsonTriangulationNamespace;
using DelaunatorSharp;
using UnityEngine;
using GenericCode;

namespace TriangulationNavigation
{
    public class TriangulationSystem : MonoBehaviour
    {
        BowyerWatsonTriangulation bowyerWatsonTriangulation;
        List<Float2> delaunatorPoints;
        List<DelaunatorSharp.Triangle> delaunatorTriangles;

        GizmoDrawer gizmoDrawer;

        void Start()
        {
            gizmoDrawer = new GizmoDrawer();

            // BowyerWatson();
            Delaunator();
        }

        void BowyerWatson()
        {
            bowyerWatsonTriangulation = new BowyerWatsonTriangulation();
            bowyerWatsonTriangulation.Triangulate(PointsGenerator.GetRandomPointsInsideCircle(300, 2, 40.0f));
        }

        void Delaunator()
        {
            delaunatorPoints = PointsGenerator.GetRandomPointsInsideCircle(new ManualRandom(2), 300, 40.0f);

            Delaunator delaunator = new Delaunator();
            delaunator.Create(delaunatorPoints);
            delaunatorTriangles = delaunator.GetTriangles();
        }

        void Update()
        {
        }

        void OnDrawGizmos()
        {
            if (!Application.isPlaying)
            {
                return;
            }

            if (gizmoDrawer != null)
            {
                gizmoDrawer.DrawBowyerWatson(bowyerWatsonTriangulation);
                gizmoDrawer.DrawDelaunator(delaunatorPoints, delaunatorTriangles);
            }
        }
    }
}
