using System.Collections.Generic;
using GenericCode;

namespace GridNavigation
{
    public class Agent
    {
        public int radiusIndex;
        public float speed;
        public Float2 destination;
        public List<Float2> waypoints;
        public int currentWaypointIndex;
        public bool followingPath;
        public Float2 finalVelocity;
        public float remainingPathDistance;
        public int pathMovementFailuresCount;
        public float density;
    }
}
