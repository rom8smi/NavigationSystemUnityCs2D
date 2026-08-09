using UnityEngine;
using GenericCode;

namespace GridNavigation
{
    public class AgentsRandomSpawner : MonoBehaviour
    {
        public GameObject agentPrefab;
        public Float2 centerPosition;
        public Float2 size;
        public int numberToSpawn;
        public Transform target;
        public int seed;

        void Start()
        {
            ManualRandom random = new ManualRandom((ulong)seed);
            
            for (int i = 0; i < numberToSpawn; i++)
            {
                float x = random.next_float(centerPosition.x - 0.5f * size.x, centerPosition.x + 0.5f * size.x);
                float y = random.next_float(centerPosition.y - 0.5f * size.y, centerPosition.y + 0.5f * size.y);
                Vector3 pos = new Vector3(x, 0f, y);

                GameObject instance = Instantiate(agentPrefab, pos, Quaternion.identity);
                instance.GetComponent<AgentMonobehaviour>().target = target;
            }
        }
    }
}
