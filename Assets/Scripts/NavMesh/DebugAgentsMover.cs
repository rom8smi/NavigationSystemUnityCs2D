using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine;
using GenericCode;

namespace TriangulationNavigation
{
    public class DebugAgentsMover
    {
        bool debugIntoFile = true;
        string fileName = "DebugAgentsMover";
        int currentCount = 10;
        int maxCount = 5000;
        int iUpdate = 0;

        public void MoveAgents(
            AgentsMover agentsMover,
            List<Agent> agents,
            List<Float2> agentPositions,
            List<Transform> agentTransforms,
            List<List<int>> agentNeighbours,
            List<int> agentNeighbourCounts,
            List<AgentType> agentTypes,
            float largestAgentRadius,
            NavMesh navMesh,
            Pathfinding pathfinding,
            float deltaTime)
        {
            Stopwatch sw = new Stopwatch();
            sw.Start();

            agentsMover.Repath(agents, agentPositions, pathfinding, navMesh);

            float t1 = (float)sw.Elapsed.TotalMilliseconds;

            agentsMover.CalculatePathVelocities(agents, agentPositions, navMesh);

            float t2 = (float)sw.Elapsed.TotalMilliseconds;
            float t3 = t2;

            if (agentsMover.useKdTreeForNeighbours)
            {
                agentsMover.FindNeighboursWithKdTree(agents, agentPositions, agentNeighbours, agentNeighbourCounts, agentTypes);
            }
            else
            {
                if (agentPositions.Count < 3)
                {
                    agentsMover.FindNeighboursDirect(agents, agentPositions, agentNeighbours, agentNeighbourCounts, largestAgentRadius);
                }
                else
                {
                    agentsMover.triangulation.Create(agentPositions);

                    t3 = (float)sw.Elapsed.TotalMilliseconds;

                    agentsMover.FindNeighboursWithTriangulation(agentNeighbours, agentNeighbourCounts);
                }
            }

            float t4 = (float)sw.Elapsed.TotalMilliseconds;

            agentsMover.CalculateVelocitiesFromNeighbours(agents, agentPositions, agentTransforms, agentNeighbours, agentNeighbourCounts, agentTypes, navMesh, deltaTime);

            float t5 = (float)sw.Elapsed.TotalMilliseconds;

            if (debugIntoFile)
            {
                iUpdate++;

                if (iUpdate > 2)
                {
                    iUpdate = 0;
                    Write(t1, t2 - t1, t3 - t2, t4 - t3, t5 - t4);

                    currentCount++;
                    UnityEngine.Debug.Log("Written: " + (currentCount * 100f / maxCount).ToString() + " %");

                    if (currentCount >= maxCount)
                    {
                        UnityEditor.EditorApplication.isPlaying = false;
                    }
                }
            }
            else
            {
                UnityEngine.Debug.Log($"{t1} {t2 - t1} {t3 - t2} {t4 - t3} {t5 - t4}");
            }
        }

        void Write(float repathTime, float calculatePathVelocitiesTime, float triangulationCreationTime, float moveAgentsTime, float adjustAgentPositionsForObstaclesTime)
        {
            string path = System.IO.Path.Combine(Application.dataPath, fileName + ".txt");

            if (!System.IO.File.Exists(path))
            {
                string createText = $"# t repathTime calculatePathVelocitiesTime triangulationCreationTime moveAgentsTime adjustAgentPositionsForObstaclesTime\n";
                System.IO.File.WriteAllText(path, createText);
            }

            string appendText = $"{Time.time} {repathTime} {calculatePathVelocitiesTime} {triangulationCreationTime} {moveAgentsTime} {adjustAgentPositionsForObstaclesTime}\n";
            System.IO.File.AppendAllText(path, appendText);
        }
    }
}
