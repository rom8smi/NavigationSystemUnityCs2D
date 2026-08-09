namespace GridNavigation
{
    public struct Node
    {
        public bool walkable;
        public int gridX;
        public int gridY;
        public int gridIndex;
        public int movementPenalty;

        public int nearestWalkableEdgeLeft;
        public int nearestWalkableEdgeRight;
        public int nearestWalkableEdgeBottom;
        public int nearestWalkableEdgeTop;
    }
}
