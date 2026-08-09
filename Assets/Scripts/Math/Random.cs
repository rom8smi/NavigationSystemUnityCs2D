namespace GenericCode
{
    public class ManualRandom
    {
        ulong seed;
		float random_max;

        public ManualRandom(ulong p_seed)
        {
            seed = p_seed;
		    random_max = 32767.0f;
        }

        int next_int()
        {
            seed = seed * 1103515245 + 12345;
            return (int)((uint)(seed / 65536) % 32768);
        }

        // min and max - inclusive
        public int next_int(int min, int max)
        {
            return (int)next_float(min, max + 1.0f);
        }

        public float next_float()
        {
            return (float)(next_int()) / random_max;
        }

        public float next_float(float min, float max)
        {
            return (max - min) * next_float() + min;
        }

        ulong get_seed()
        {
            return seed;
        }

        void set_seed(ulong p_seed)
        {
            seed = p_seed;
        }
    }
}
