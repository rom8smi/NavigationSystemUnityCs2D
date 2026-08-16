using System.Collections.Generic;
using GenericCode;
using UnityEngine;

namespace TriangulationNavigation
{
    public class RandomDynamicAgentsSpawner : MonoBehaviour
    {
        public List<AgentsRandomSpawner> agentsRandomSpawners;
        ManualRandom random = new ManualRandom(0);
        float fUpdates;

        void Start()
        {
            random = new ManualRandom(0);
        }

        void Update()
        {
            fUpdates += RuntimeConstants.updatesPerFrame;
            if (fUpdates > 1.0f)
            {
                int iUpdates = (int)fUpdates;

                for (int i = 0; i < iUpdates; i++)
                {
                    UpdateInner();
                }

                fUpdates -= iUpdates;
            }
        }

        void UpdateInner()
        {
            Remove();
            Add();
        }

        void Remove()
        {
            for (int i = 0; i < NavigationSystemComponent.active.navigationSystem.agents.Count; i++)
            {
                if (!NavigationSystemComponent.active.navigationSystem.HasPath(i))
                {
                    float probability = 0.001f;
                    float randomNumber = random.next_float();

                    if (randomNumber < probability)
                    {
                        NavigationSystemComponent.active.navigationSystem.agents.RemoveAt(i);
                        NavigationSystemComponent.active.navigationSystem.agentPositions.RemoveAt(i);
                        NavigationSystemComponent.active.navigationSystem.agentNeighbours.RemoveAt(i);
                        NavigationSystemComponent.active.navigationSystem.agentNeighbourCounts.RemoveAt(i);

                        Destroy(NavigationSystemComponent.active.agentTransforms[i].gameObject);
                        NavigationSystemComponent.active.agentTransforms.RemoveAt(i);

                        i--;
                    }
                }
            }
        }

        void Add()
        {
            List<int> agentTypeCount = new List<int>();
            for (int i = 0; i < NavigationSystemComponent.active.navigationSystem.agents.Count; i++)
            {
                int agentType = NavigationSystemComponent.active.navigationSystem.agents[i].agentTypeIndex;
                if (agentType >= agentTypeCount.Count)
                {
                    agentTypeCount.Add(1);
                }
                else
                {
                    agentTypeCount[agentType]++;
                }
            }

            for (int i = 0; i < agentsRandomSpawners.Count; i++)
            {
                int largestNumberOfAgents = agentsRandomSpawners[i].numberToSpawn;
                int agentType = agentsRandomSpawners[i].agentPrefab.GetComponent<AgentMonobehaviour>().agentTypeIndex;

                int numberToSpawn = largestNumberOfAgents - agentTypeCount[agentType];
                if (numberToSpawn > 0)
                {
                    for (int j = 0; j < numberToSpawn; j++)
                    {
                        float probability = 0.001f;
                        float randomNumber = random.next_float();

                        if (randomNumber < probability)
                        {
                            agentsRandomSpawners[i].Spawn(random);
                        }
                    }
                }
            }
        }
    }
}
