using UnityEngine;
using GenericCode;

namespace TriangulationNavigation
{
    public class AgentMonobehaviour : MonoBehaviour
    {
        public Transform target;
        public int agentTypeIndex;

        void Start()
        {
            Float2 targetPosition = new Float2(0.0f, 0.0f);
            if (target != null)
            {
                targetPosition = new Float2(target.position.x, target.position.z);
            }

            gameObject.name = $"{gameObject.name.Replace("(Clone)", string.Empty)} {NavigationSystemComponent.active.agentTransforms.Count}";
            NavigationSystemComponent.active.AddAgentTransform(transform, agentTypeIndex, targetPosition);

            // AgentColorCoder agentColorCoder = GetComponent<AgentColorCoder>();
            // if (agentColorCoder != null)
            // {
            //     agentColorCoder.Setup(NavigationSystem.active.agents[NavigationSystem.active.agents.Count - 1]);
            // }

            Destroy(this);
        }
    }
}
