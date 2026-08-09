using UnityEngine;

namespace GridNavigation
{
    public class AgentColorCoder : MonoBehaviour
    {
        public MeshRenderer meshRenderer;
        public Agent agent;
        public Gradient gradient;
        public float min;
        public float max;

        public void Setup(Agent p_agent)
        {
            agent = p_agent;
        }

        void Update()
        {
            float t = (agent.density - min) / (max - min);
            meshRenderer.material.color = gradient.Evaluate(t);
        }
    }
}
