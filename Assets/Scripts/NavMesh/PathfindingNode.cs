namespace TriangulationNavigation
{
    public struct PathfindingNode
    {
        public float gCost;
        public float hCost;
        public int parent;
        public int heapIndex;
        public bool isInClosedSet;
    }
}
