using UnityEngine;
using System.Collections.Generic;
using DelaunatorSharp;
using GenericCode;

namespace TriangulationNavigation
{
    public class LineSegmentIntersectionThreeTests : MonoBehaviour
    {
        public List<Transform> obstacleTransforms;
        public float worldSize;

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
            List<Obstacle> obstacles = GetObstacles(worldSize);
            constraintEdges = new List<ConstraintEdge>();
            newPoints = new List<Float2>();

            NavMesh navMesh = new NavMesh();
            List<int> newObstacleWalkablityIndices = new List<int>();
            List<List<int>> obstacleIntersections = new List<List<int>>();
            List<bool> newIsObstacleCornerIntersectingWithWorldBounds = new List<bool>();
            navMesh.AddObstaclesWithConstraints(
                obstacles,
                constraintEdges,
                newPoints,
                newObstacleWalkablityIndices,
                obstacleIntersections,
                newIsObstacleCornerIntersectingWithWorldBounds);
        }

        void Update()
        {
            t += RuntimeConstants.deltaTime;
            if (t > 0.4f)
            {
                t = 0f;
                drawIndex++;
                if (drawIndex > constraintEdges.Count)
                {
                    drawIndex = 0;
                }
            }
        }

        public List<Obstacle> GetObstacles(float p_worldSize)
        {
            int obstaclesLength = obstacleTransforms.Count;
            List<Obstacle> obstacles = new List<Obstacle>();
            List<List<Float2>> allCorners = new List<List<Float2>>();

            for (int i = 0; i < obstaclesLength; i++)
            {
                if (obstacleTransforms[i].gameObject.activeSelf)
                {
                    Vector3 position = obstacleTransforms[i].position;
                    float rotation = -obstacleTransforms[i].rotation.eulerAngles.y * MathUtils.Deg2Rad();
                    Vector3 scale = obstacleTransforms[i].lossyScale;

                    RectangularObstacle rectangularObstacle = new RectangularObstacle
                    {
                        center = new Float2(position.x, position.z),
                        size = new Float2(scale.x, scale.z),
                        rotation = rotation,
                        radius = 1f
                    };

                    List<Float2> corners = rectangularObstacle.GetCorners();
                    allCorners.Add(corners);
                }
            }

            AABB bounds = new AABB
            {
                minX = -0.5f * p_worldSize,
                maxX = 0.5f * p_worldSize,
                minY = -0.5f * p_worldSize,
                maxY = 0.5f * p_worldSize,
            };

            for (int i = 0; i < allCorners.Count; i++)
            {
                Obstacle obstacle = ObstacleUtils.Create(allCorners[i], bounds, 2.0f * p_worldSize, false);
                obstacles.Add(obstacle);
            }

            return obstacles;
        }

        void OnDrawGizmos()
        {
            if (constraintEdges != null && newPoints != null)
            {
                Gizmos.color = Color.green;

                for (int i = 0; i < newPoints.Count; i++)
                {
                    Vector3 p = GizmoDrawer.ToVector3(newPoints[i]);
                    Gizmos.DrawSphere(p, 0.3f);
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
