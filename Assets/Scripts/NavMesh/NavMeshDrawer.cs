using System.Collections.Generic;
using UnityEngine;
using GenericCode;
using DelaunatorSharp;

namespace TriangulationNavigation
{
    public class NavMeshDrawer
    {
        Mesh navMeshSurface;

        public NavMeshDrawer()
        {
        }

        public void CreateNavMeshSurfaceDrawer(NavMesh navMesh)
        {
            List<Triangle> triangles = navMesh.delaunator.GetTriangles();

            List<Vector3> meshVertices = new List<Vector3>();
            List<int> meshTriangles = new List<int>();
            List<Vector3> meshNormals = new List<Vector3>();

            for (int i = 0; i < triangles.Count; i++)
            {
                if (navMesh.trianglesWalkability[i] == -1)
                {
                    Triangle triangle = triangles[i];
                    List<int> trianglePoints = triangle.points;

                    for (int j = 0; j < 3; j++)
                    {
                        int pointIndex = trianglePoints[j];
                        meshVertices.Add(ToVector3(navMesh.allPoints[pointIndex]));
                        meshTriangles.Add(meshVertices.Count - 1);
                        meshNormals.Add(new Vector3(0.0f, 0.0f, 1.0f));
                    }
                }
            }

            navMeshSurface = new Mesh();
            navMeshSurface.vertices = meshVertices.ToArray();
            navMeshSurface.triangles = meshTriangles.ToArray();
            navMeshSurface.normals = meshNormals.ToArray();
        }

        Vector3 ToVector3(Float2 p)
        {
            return new Vector3(p.x, 0.0f, p.y);
        }

        public void DrawSurface(Material navMeshSurfaceMaterial, bool displayNavmeshGizmos)
        {
            if (displayNavmeshGizmos)
            {
                Graphics.DrawMesh(navMeshSurface, Vector3.zero, Quaternion.identity, navMeshSurfaceMaterial, 0);
            }
        }

        public void DrawNavMesh(NavMesh navMesh, bool displayNavmeshGizmos, bool displayUnwalkableEdges, float cubeSizeMultiplier)
        {
            if (displayNavmeshGizmos)
            {
                Gizmos.color = new Color(0.05f, 0.05f, 0.05f, 1.0f);

                for (int i = 0; i < navMesh.allPoints.Count; i++)
                {
                    Vector3 point = ToVector3(navMesh.allPoints[i]);
                    Gizmos.DrawCube(point, Vector3.one * 0.5f * cubeSizeMultiplier);
                }

                List<Edge> edges = navMesh.delaunator.GetEdges();
                for (int i = 0; i < edges.Count; i++)
                {
                    int edgeIndex = edges[i].index;

                    Color c = Color.blue;
                    bool displayEdge = true;

                    if (!navMesh.edgesWalkability[edgeIndex])
                    {
                        c = Color.red;
                        if (!displayUnwalkableEdges)
                        {
                            displayEdge = false;
                        }
                    }

                    if (displayEdge)
                    {
                        Gizmos.color = c;

                        int p = edges[i].p;
                        int q = edges[i].q;

                        GizmoDrawer.DrawGizmosLine(navMesh.allPoints[p], navMesh.allPoints[q]);
                    }
                }
            }
        }

        public void DrawObstaclePushDirections(NavMesh navMesh, bool displayPushDirections, List<Float2> positions, float size)
        {
            if (displayPushDirections)
            {
                Gizmos.color = new Color(0.05f, 0.05f, 0.05f, 1.0f);
                float epsilon = 0.001f;

                for (int i = 0; i < positions.Count; i++)
                {
                    GetNearestWalkablePositionResult getNearestWalkablePositionResult = navMesh.GetNearestWalkablePosition(positions[i], epsilon);

                    if (getNearestWalkablePositionResult.wasMoved)
                    {
                        Vector3 point = ToVector3(positions[i]);
                        Gizmos.DrawCube(point, Vector3.one * 0.3f);

                        Float2 direction = (getNearestWalkablePositionResult.position - positions[i]).Normalized();
                        GizmoDrawer.DrawGizmosLine(positions[i], positions[i] + direction * size);
                    }

                }
            }
        }
    }
}
