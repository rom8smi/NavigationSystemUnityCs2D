using System.Collections.Generic;
using UnityEngine;
using GenericCode;

namespace TriangulationNavigation
{
    public class GizmoDrawer
    {
        public void DrawBowyerWatson(BowyerWatsonTriangulationNamespace.BowyerWatsonTriangulation bowyerWatsonTriangulation)
        {
            if (bowyerWatsonTriangulation == null)
            {
                return;
            }

            for (int i = 0; i < bowyerWatsonTriangulation.points.Count; i++)
            {
                Gizmos.color = Color.red;
                Vector3 point = new Vector3(bowyerWatsonTriangulation.points[i].x, 0.0f, bowyerWatsonTriangulation.points[i].y);
                Gizmos.DrawCube(point, Vector3.one * 0.5f);
            }

            Gizmos.color = Color.red;
            Gizmos.DrawLine(new Vector3(bowyerWatsonTriangulation.minX, 0.0f, bowyerWatsonTriangulation.minY), new Vector3(bowyerWatsonTriangulation.maxX, 0.0f, bowyerWatsonTriangulation.minY));
            Gizmos.DrawLine(new Vector3(bowyerWatsonTriangulation.minX, 0.0f, bowyerWatsonTriangulation.minY), new Vector3(bowyerWatsonTriangulation.minX, 0.0f, bowyerWatsonTriangulation.maxY));
            Gizmos.DrawLine(new Vector3(bowyerWatsonTriangulation.maxX, 0.0f, bowyerWatsonTriangulation.minY), new Vector3(bowyerWatsonTriangulation.maxX, 0.0f, bowyerWatsonTriangulation.maxY));
            Gizmos.DrawLine(new Vector3(bowyerWatsonTriangulation.minX, 0.0f, bowyerWatsonTriangulation.maxY), new Vector3(bowyerWatsonTriangulation.maxX, 0.0f, bowyerWatsonTriangulation.maxY));

            Gizmos.color = Color.blue;
            DrawGizmosLine(bowyerWatsonTriangulation.tri1, bowyerWatsonTriangulation.tri2);
            DrawGizmosLine(bowyerWatsonTriangulation.tri2, bowyerWatsonTriangulation.tri3);
            DrawGizmosLine(bowyerWatsonTriangulation.tri3, bowyerWatsonTriangulation.tri1);

            for (int i = 0; i < bowyerWatsonTriangulation.triangulation.Count; i++)
            {
                BowyerWatsonTriangulationNamespace.Triangle t = bowyerWatsonTriangulation.triangulation[i];
                Gizmos.color = Color.red;
                DrawGizmosLine(t.points[0], t.points[1]);
                DrawGizmosLine(t.points[1], t.points[2]);
                DrawGizmosLine(t.points[2], t.points[0]);
            }
        }

        public void DrawDelaunator(List<Float2> allPoints, List<DelaunatorSharp.Triangle> triangles)
        {
            for (int i = 0; i < allPoints.Count; i++)
            {
                Gizmos.color = Color.black;
                Vector3 point = ToVector3(allPoints[i]);
                Gizmos.DrawCube(point, Vector3.one * 0.5f);
            }

            for (int i = 0; i < triangles.Count; i++)
            {
                DelaunatorSharp.Triangle triangle = triangles[i];
                List<int> trianglePoints = triangle.points;
                DrawGizmosLine(allPoints[trianglePoints[0]], allPoints[trianglePoints[1]]);
                DrawGizmosLine(allPoints[trianglePoints[1]], allPoints[trianglePoints[2]]);
                DrawGizmosLine(allPoints[trianglePoints[2]], allPoints[trianglePoints[0]]);
            }
        }

        public void DrawAgentsGizmos(bool showPaths, List<Transform> agentTransforms, List<Agent> agents)
        {
            if (showPaths)
            {
                for (int i = 0; i < agentTransforms.Count; i++)
                {
                    DrawAgentGizmos(agents[i], agentTransforms[i].position);
                }
            }
        }

        void DrawAgentGizmos(Agent agent, Vector3 position)
        {
            if (agent.followingPath)
            {
                DrawPath(agent.currentWaypointIndex, position, agent.waypoints, agent.waypoints.Count, Color.black);
                DrawPath(agent.currentWaypointIndex, position, agent.simplifiedWaypoints, agent.simplifiedWaypoints.Count, Color.green);
            }
        }

        void DrawPath(int startIndex, Vector3 position, List<Float2> waypoints, int count, Color color)
        {
            for (int i = startIndex; i < count; i++)
            {
                Gizmos.color = color;
                Vector3 currentWaypoint = new Vector3(waypoints[i].x, 0.0f, waypoints[i].y);
                Gizmos.DrawCube(currentWaypoint, Vector3.one * 0.8f);

                if (i == startIndex)
                {
                    Gizmos.DrawLine(position, currentWaypoint);
                }
                else
                {
                    Vector3 previousWaypoint = new Vector3(waypoints[i - 1].x, 0.0f, waypoints[i - 1].y);
                    Gizmos.DrawLine(previousWaypoint, currentWaypoint);
                }
            }
        }

        public static Vector3 ToVector3(Float2 p)
        {
            return new Vector3(p.x, 0.0f, p.y);
        }

        public static void DrawGizmosLine(Float2 start, Float2 end)
        {
            Gizmos.DrawLine(ToVector3(start), ToVector3(end));
        }

        void DrawGizmosLine(BowyerWatsonTriangulationNamespace.Point start, BowyerWatsonTriangulationNamespace.Point end)
        {
            Gizmos.DrawLine(new Vector3(start.x, 0.0f, start.y), new Vector3(end.x, 0.0f, end.y));
        }
    }
}
