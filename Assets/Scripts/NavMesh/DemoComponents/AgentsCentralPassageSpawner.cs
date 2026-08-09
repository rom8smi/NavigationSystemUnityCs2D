using GenericCode;
using UnityEngine;

namespace TriangulationNavigation
{
    public class AgentsCentralPassageSpawner : MonoBehaviour
    {
        public GameObject agentPrefab;
        public float radius;
        public int numberToSpawn;
        public int seed;

        void Start()
        {
            ManualRandom random = new ManualRandom((ulong)seed);

            for (int i = 0; i < numberToSpawn; i++)
            {
                Float2 pos = VectorUtils.RangomInsideUnitCircle(random) * radius;
                GameObject instance = Instantiate(agentPrefab, new Vector3(pos.x, 0.0f, pos.y), Quaternion.identity);
                instance.name = $"{agentPrefab.name} {NavigationSystemComponent.active.agentTransforms.Count}";

                AgentMonobehaviour agentMonobehaviour = instance.GetComponent<AgentMonobehaviour>();

                NavigationSystemComponent.active.AddAgentTransform(agentMonobehaviour.transform, agentMonobehaviour.agentTypeIndex, -pos);
                Destroy(agentMonobehaviour);
            }
        }
    }
}
