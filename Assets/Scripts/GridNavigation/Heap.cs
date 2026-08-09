using System.Collections.Generic;

namespace GridNavigation
{
    public class Heap
    {
        List<int> items;
        public int count;

        public Heap()
        {
            items = new List<int>();
        }

        public void Add(int item, PathfindingNode[] pathfindingNodes)
        {
            pathfindingNodes[item].heapIndex = count;

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

        public void Clear(PathfindingNode[] pathfindingNodes)
        {
            if (count > 0)
            {
                for (int i = 0; i < count; i++)
                {
                    int nodeIndex = items[i];
                    pathfindingNodes[nodeIndex].isInClosedSet = false;
                    pathfindingNodes[nodeIndex].parent = -1;
                    pathfindingNodes[nodeIndex].heapIndex = -1;
                }

                items.Clear();
                count = 0;
            }
        }

        public int RemoveFirst(PathfindingNode[] pathfindingNodes)
        {
            int firstItem = items[0];
            count--;
            items[0] = items[count];
            pathfindingNodes[items[0]].heapIndex = 0;
            SortDown(items[0], pathfindingNodes);

            pathfindingNodes[firstItem].heapIndex = -1;
            return firstItem;
        }

        public void UpdateItem(int item, PathfindingNode[] pathfindingNodes)
        {
            SortUp(item, pathfindingNodes);
        }

        void SortDown(int item, PathfindingNode[] pathfindingNodes)
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

        void SortUp(int item, PathfindingNode[] pathfindingNodes)
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

        void Swap(int itemA, int itemB, PathfindingNode[] pathfindingNodes)
        {
            items[pathfindingNodes[itemA].heapIndex] = itemB;
            items[pathfindingNodes[itemB].heapIndex] = itemA;
            int itemAIndex = pathfindingNodes[itemA].heapIndex;
            pathfindingNodes[itemA].heapIndex = pathfindingNodes[itemB].heapIndex;
            pathfindingNodes[itemB].heapIndex = itemAIndex;
        }

        public int CompareTo(int nodeA, int nodeB, PathfindingNode[] pathfindingNodes)
        {
            int compare = InverseCompareTo(FCost(nodeA, pathfindingNodes), FCost(nodeB, pathfindingNodes));
            if (compare == 0)
            {
                compare = InverseCompareTo(pathfindingNodes[nodeA].hCost, pathfindingNodes[nodeB].hCost);
            }
            return compare;
        }

        int InverseCompareTo(int a, int b)
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

        int FCost(int node, PathfindingNode[] pathfindingNodes)
        {
            return pathfindingNodes[node].gCost + pathfindingNodes[node].hCost;
        }
    }
}
