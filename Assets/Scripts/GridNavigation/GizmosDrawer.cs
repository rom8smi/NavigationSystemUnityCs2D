using System.Collections.Generic;
using UnityEngine;
using GenericCode;

namespace GridNavigation
{
    public class GizmosDrawer
    {
        public void DrawGizmos(
            Grid navigationGrid,
            Vector3 origin,
            bool displayGridGizmos,
            int penaltyMin,
            int penaltyMax,
            float nodeDiameter)
        {
            if (!Application.isPlaying)
            {
                return;
            }

            if (displayGridGizmos && navigationGrid != null && navigationGrid.nodes != null)
            {
                Gizmos.DrawWireCube(origin, new Vector3(navigationGrid.gridWorldSize.x, 1, navigationGrid.gridWorldSize.y));

                for (int i = 0; i < navigationGrid.nodes.Length; i++)
                {
                    Node node = navigationGrid.nodes[i];
                    Gizmos.color = Color.Lerp(Color.white, Color.black, Mathf.InverseLerp(penaltyMin, penaltyMax, node.movementPenalty));

                    Gizmos.color = node.walkable ? Gizmos.color : Color.red;
                    Float2 pos = navigationGrid.GetWorldPosition(node.gridX, node.gridY);
                    Gizmos.DrawCube(new Vector3(pos.x, 0.0f, pos.y), Vector3.one * nodeDiameter);
                }
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
                DrawPath(agent.currentWaypointIndex, position, agent.waypoints, Color.black);
            }
        }

        void DrawPath(int startIndex, Vector3 position, List<Float2> waypoints, Color color)
        {
            for (int i = startIndex; i < waypoints.Count; i++)
            {
                Gizmos.color = Color.black;
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

        public void DrawLineSegments(List<LineSegment> lineSegments, float endingsSize)
        {
            for (int i = 0; i < lineSegments.Count; i++)
            {
                Gizmos.color = Color.black;

                Vector3 start = new Vector3(lineSegments[i].start.x, 0.0f, lineSegments[i].start.y);
                Vector3 end = new Vector3(lineSegments[i].end.x, 0.0f, lineSegments[i].end.y);

                Gizmos.DrawCube(start, Vector3.one * endingsSize);
                Gizmos.DrawCube(end, Vector3.one * endingsSize);
                Gizmos.DrawLine(start, end);
            }
        }
    }
}
