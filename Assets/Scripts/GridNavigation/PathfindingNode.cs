namespace GridNavigation
{
    public struct PathfindingNode
    {
        public int gCost;
        public int hCost;
        public int parent;
        public int heapIndex;
        public bool isInClosedSet;
    }
}
