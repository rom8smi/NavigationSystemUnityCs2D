using System.Collections.Generic;
using UnityEngine;
using GenericCode;

namespace TriangulationNavigation
{
    public class NavigationSystem
    {
        public NavMesh navMesh;
        public Pathfinding pathfinding;
        public AgentsMover agentsMover;
        public DebugAgentsMover debugAgentsMover;
        public bool use_debug_agents_mover;

        public NavMeshDrawer navMeshDrawer;

        public List<Agent> agents;
        public List<Float2> agentPositions;
        public List<List<int>> agentNeighbours;
        public List<int> agentNeighbourCounts;
        public List<AgentType> agentTypes;

        public List<Obstacle> obstacles;
        public AABB worldBounds;
        public AABB paddedWorldBounds;

        public float largestAgentRadius;

        public NavigationSystem(List<AgentType> p_agentTypes)
        {
            agentTypes = p_agentTypes;

            navMesh = new NavMesh();
            navMeshDrawer = new NavMeshDrawer();
            debugAgentsMover = new DebugAgentsMover();

            pathfinding = new Pathfinding();
            agentsMover = new AgentsMover();

            agents = new List<Agent>();
            agentPositions = new List<Float2>();
            agentNeighbours = new List<List<int>>();
            agentNeighbourCounts = new List<int>();

            largestAgentRadius = 0.0f;
            for (int i = 0; i < agentTypes.Count; i++)
            {
                largestAgentRadius = MathUtils.Max(largestAgentRadius, agentTypes[i].radius);
            }
        }

        public void Update(List<Transform> agentTransforms, float deltaTime)
        {
            if (use_debug_agents_mover)
                {
                    debugAgentsMover.MoveAgents(
                        agentsMover,
                        agents,
                        agentPositions,
                        agentTransforms,
                        agentNeighbours,
                        agentNeighbourCounts,
                        agentTypes,
                        largestAgentRadius,
                        navMesh,
                        pathfinding,
                        deltaTime);
                }
                else
                {
                    agentsMover.MoveAgents(
                        agents,
                        agentPositions,
                        agentTransforms,
                        agentNeighbours,
                        agentNeighbourCounts,
                        agentTypes,
                        largestAgentRadius,
                        navMesh,
                        pathfinding,
                        deltaTime);
                }
        }
    }
}
