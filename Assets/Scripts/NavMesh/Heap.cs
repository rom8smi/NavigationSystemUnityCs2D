using System.Collections.Generic;

namespace TriangulationNavigation
{
    public class Heap
    {
        List<int> items;
        public int count;

        public Heap()
        {
            items = new List<int>();
            count = 0;
        }

        public void Add(int item, List<PathfindingNode> pathfindingNodes)
        {
            PathfindingNode node = pathfindingNodes[item];
            node.heapIndex = count;
            pathfindingNodes[item] = node;

            if (items.Count == count)
            {
                items.Add(item);
            }
            else
            {
                items[count] = item;
            }

            SortUp(item, pathfindingNodes);
            count++;
        }

        public void Clear(List<PathfindingNode> pathfindingNodes)
        {
            if (count > 0)
            {
                for (int i = 0; i < count; i++)
                {
                    int nodeIndex = items[i];
                    PathfindingNode node = pathfindingNodes[nodeIndex];
                    node.isInClosedSet = false;
                    node.parent = -1;
                    node.heapIndex = -1;
                    pathfindingNodes[nodeIndex] = node;
                }

                items.Clear();
                count = 0;
            }
        }

        public int RemoveFirst(List<PathfindingNode> pathfindingNodes)
        {
            int firstItem = items[0];
            count--;
            items[0] = items[count];

            PathfindingNode node = pathfindingNodes[items[0]];
            node.heapIndex = 0;
            pathfindingNodes[items[0]] = node;

            SortDown(items[0], pathfindingNodes);

            node = pathfindingNodes[firstItem];
            node.heapIndex = -1;
            pathfindingNodes[firstItem] = node;

            return firstItem;
        }

        public void UpdateItem(int item, List<PathfindingNode> pathfindingNodes)
        {
            SortUp(item, pathfindingNodes);
        }

        void SortDown(int item, List<PathfindingNode> pathfindingNodes)
        {
            while (true)
            {
                int childIndexLeft = pathfindingNodes[item].heapIndex * 2 + 1;
                int childIndexRight = pathfindingNodes[item].heapIndex * 2 + 2;
                int swapIndex = 0;

                if (childIndexLeft < count)
                {
                    swapIndex = childIndexLeft;

                    if (childIndexRight < count)
                    {
                        if (CompareTo(items[childIndexLeft], items[childIndexRight], pathfindingNodes) < 0)
                        {
                            swapIndex = childIndexRight;
                        }
                    }

                    if (CompareTo(item, items[swapIndex], pathfindingNodes) < 0)
                    {
                        Swap(item, items[swapIndex], pathfindingNodes);
                    }
                    else
                    {
                        return;
                    }
                }
                else
                {
                    return;
                }
            }
        }

        void SortUp(int item, List<PathfindingNode> pathfindingNodes)
        {
            int parentIndex = (pathfindingNodes[item].heapIndex - 1) / 2;

            while (true)
            {
                int parentItem = items[parentIndex];
                if (CompareTo(item, parentItem, pathfindingNodes) > 0)
                {
                    Swap(item, parentItem, pathfindingNodes);
                }
                else
                {
                    break;
                }

                parentIndex = (pathfindingNodes[item].heapIndex - 1) / 2;
            }
        }

        void Swap(int itemA, int itemB, List<PathfindingNode> pathfindingNodes)
        {
            items[pathfindingNodes[itemA].heapIndex] = itemB;
            items[pathfindingNodes[itemB].heapIndex] = itemA;
            int itemAIndex = pathfindingNodes[itemA].heapIndex;

            PathfindingNode node = pathfindingNodes[itemA];
            node.heapIndex = pathfindingNodes[itemB].heapIndex;
            pathfindingNodes[itemA] = node;

            node = pathfindingNodes[itemB];
            node.heapIndex = itemAIndex;
            pathfindingNodes[itemB] = node;
        }

        public int CompareTo(int nodeA, int nodeB, List<PathfindingNode> pathfindingNodes)
        {
            int compare = InverseCompareTo(FCost(nodeA, pathfindingNodes), FCost(nodeB, pathfindingNodes));
            if (compare == 0)
            {
                compare = InverseCompareTo(pathfindingNodes[nodeA].hCost, pathfindingNodes[nodeB].hCost);
            }
            return compare;
        }

        int InverseCompareTo(float a, float b)
        {
            if (a > b)
            {
                return -1;
            }
            if (a < b)
            {
                return 1;
            }
            return 0;
        }

        float FCost(int node, List<PathfindingNode> pathfindingNodes)
        {
            return pathfindingNodes[node].gCost + pathfindingNodes[node].hCost;
        }
    }
}
