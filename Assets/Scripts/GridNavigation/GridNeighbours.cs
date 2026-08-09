namespace GridNavigation
{
    public class GridNeighbours
    {
        public int[] neighbours;
        public int count;

        public GridNeighbours(int size)
        {
            neighbours = new int[size];
            count = 0;
        }

        public void Add(int neighbour)
        {
            neighbours[count] = neighbour;
            count++;
        }

        public void Clear()
        {
            count = 0;
        }
    }
}
