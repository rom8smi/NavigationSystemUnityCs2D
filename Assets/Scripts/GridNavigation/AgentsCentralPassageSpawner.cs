using GenericCode;
using UnityEngine;

namespace GridNavigation
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
                AgentMonobehaviour agentMonobehaviour = instance.GetComponent<AgentMonobehaviour>();

                NavigationSystem.active.AddAgentTransform(agentMonobehaviour.transform, agentMonobehaviour.radiusIndex, agentMonobehaviour.speed, -pos);
                Destroy(agentMonobehaviour);
            }
        }
    }
}
