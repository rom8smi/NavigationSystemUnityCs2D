using System.Diagnostics;
using TriangulationNavigation;
using UnityEngine;
using GenericCode;

public class ManualAgentController : MonoBehaviour
{
    public Transform targetTransform;
    public float speed;

    void Update()
    {
        float deltaTime = RuntimeConstants.deltaTime;

        if (Input.GetKey(KeyCode.UpArrow))
        {
            MoveAgent(new Float2(0f, deltaTime * speed));
        }
        if (Input.GetKey(KeyCode.DownArrow))
        {
            MoveAgent(new Float2(0f, -deltaTime * speed));
        }
        if (Input.GetKey(KeyCode.LeftArrow))
        {
            MoveAgent(new Float2(-deltaTime * speed, 0f));
        }
        if (Input.GetKey(KeyCode.RightArrow))
        {
            MoveAgent(new Float2(deltaTime * speed, 0f));
        }

        if (Input.GetKey(KeyCode.W))
        {
            MoveTarget(new Float2(0f, deltaTime * speed));
        }
        if (Input.GetKey(KeyCode.S))
        {
            MoveTarget(new Float2(0f, -deltaTime * speed));
        }
        if (Input.GetKey(KeyCode.A))
        {
            MoveTarget(new Float2(-deltaTime * speed, 0f));
        }
        if (Input.GetKey(KeyCode.D))
        {
            MoveTarget(new Float2(deltaTime * speed, 0f));
        }
    }

    void MoveAgent(Float2 deltaPos)
    {
        Float2 pos = NavigationSystemComponent.active.navigationSystem.agentPositions[0];
        pos += deltaPos;
        NavigationSystemComponent.active.navigationSystem.agentPositions[0] = pos;

        NavigationSystemComponent.active.agentTransforms[0].position = new Vector3(pos.x, 0f, pos.y);

        Agent agent = NavigationSystemComponent.active.navigationSystem.agents[0];
        Path path = NavigationSystemComponent.active.navigationSystem.pathfinding.FindPath(pos, agent.destination, NavigationSystemComponent.active.navigationSystem.navMesh);

        if (path.success)
        {
            agent.waypoints = path.waypoints;
            agent.followingPath = true;
            agent.currentWaypointIndex = 0;
        }
    }

    void MoveTarget(Float2 deltaPos)
    {
        Float2 pos = new Float2(targetTransform.position.x, targetTransform.position.z);
        pos += deltaPos;
        targetTransform.position = new Vector3(pos.x, 0.0f, pos.y);

        NavigationSystemComponent.active.navigationSystem.agents[0].destination = pos;
        Agent agent = NavigationSystemComponent.active.navigationSystem.agents[0];

        Path path = NavigationSystemComponent.active.navigationSystem.pathfinding.FindPath(NavigationSystemComponent.active.navigationSystem.agentPositions[0], pos, NavigationSystemComponent.active.navigationSystem.navMesh);

        if (path.success)
        {
            agent.waypoints = path.waypoints;
            agent.followingPath = true;
            agent.destination = pos;
            agent.currentWaypointIndex = 0;
        }
    }
}
