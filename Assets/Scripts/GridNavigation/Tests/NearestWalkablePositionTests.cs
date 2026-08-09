using System.Collections.Generic;
using UnityEngine;
using GenericCode;

namespace GridNavigation
{
    public class NearestWalkablePositionTests : MonoBehaviour
    {
        Grid navigationGrid;
        Pathfinding pathfinding;
        List<LineSegment> lineSegments;

        GizmosDrawer gizmosDrawer;

        void Awake()
        {
            CreateNavigationGrid();
            CreateLineSegments();
            gizmosDrawer = new GizmosDrawer();
        }

        void Update()
        {

        }

        void CreateNavigationGrid()
        {
            Float2 gridWorldSize = new Float2(100.0f, 100.0f);
            float nodeRadius = 5f;
            int obstacleProximityPenalty = 10;

            navigationGrid = new Grid
            {
                obstacles = GetObstacles(),
                penaltyObstacles = new PenaltyObstacle[0],
                gridWorldSize = gridWorldSize,
                nodeRadius = nodeRadius,
                obstacleProximityPenalty = obstacleProximityPenalty,
                gridWorldOrigin = new Float2(transform.position.x - gridWorldSize.x * 0.5f, transform.position.z - gridWorldSize.y * 0.5f)
            };
            navigationGrid.Setup();
            pathfinding = new Pathfinding(navigationGrid);
            pathfinding.smoothPath = true;
            pathfinding.numberOfSmoothIterations = 10;
        }

        Obstacle[] GetObstacles()
        {
            return new Obstacle[]
            {
                new Obstacle
                {
                    center = new Float2(0.0f, 0.0f),
                    size = new Float2(30.0f, 30.0f)
                }
            };
        }

        void CreateLineSegments()
        {
            ManualRandom random = new ManualRandom(0);
            lineSegments = new List<LineSegment>();

            for (int i = 0; i < 20; i++)
            {
                Float2 start = new Float2(random.next_float(-20.0f, 20.0f), random.next_float(-20.0f, 20.0f));
                Float2 end = navigationGrid.GetNearestWalkablePosition(start);

                lineSegments.Add(new LineSegment
                {
                    start = start,
                    end = end
                });
            }
        }

        void OnDrawGizmos()
        {
            if (!Application.isPlaying)
            {
                return;
            }

            gizmosDrawer.DrawGizmos(navigationGrid, transform.position, true, 0, 10, navigationGrid.nodeDiameter - 0.3f);
            gizmosDrawer.DrawLineSegments(lineSegments, 0.5f);
        }
    }
}
