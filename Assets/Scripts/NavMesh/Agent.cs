using System.Collections.Generic;
using GenericCode;

namespace TriangulationNavigation
{
    public class Agent
    {
        public int agentTypeIndex;
        public float speed;
        public Float2 destination;
        public List<Float2> waypoints;
        public List<Float2> simplifiedWaypoints;
        public int currentWaypointIndex;
        public bool followingPath;
        public bool searchPathLater;
        public Float2 pathVelocity;
        public Float2 localAvoidanceVelocity;
        public float powerFactorSum;
        public Float2 finalVelocity;
        public float remainingPathDistance;
        public int pathMovementFailuresCount;
        public float density;
    }
}
