using UnityEngine;
using GenericCode;

namespace GridNavigation
{
    public class AgentMonobehaviour : MonoBehaviour
    {
        public Transform target;
        public int radiusIndex;
        public float speed;

        void Start()
        {
            Float2 targetPosition = new Float2(target.position.x, target.position.z);
            NavigationSystem.active.AddAgentTransform(transform, radiusIndex, speed, targetPosition);

            AgentColorCoder agentColorCoder = GetComponent<AgentColorCoder>();
            if (agentColorCoder != null)
            {
                agentColorCoder.Setup(NavigationSystem.active.agents[NavigationSystem.active.agents.Count - 1]);
            }

            Destroy(this);
        }
    }
}
