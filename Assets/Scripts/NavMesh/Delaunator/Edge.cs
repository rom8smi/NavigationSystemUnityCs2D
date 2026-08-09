namespace DelaunatorSharp
{
    public struct Edge
    {
        public int p;
        public int q;
        public int index;

        public Edge(int p_e, int p_p, int p_q)
        {
            index = p_e;
            p = p_p;
            q = p_q;
        }
    }
}
