using System.Collections.Generic;

namespace GenericCode
{
    public static class RandomUtils
    {
        // Fisher-Yates algorithm based on https://stackoverflow.com/questions/273313/randomize-a-listt
        public static void Shuffle(List<Float2> data, ManualRandom random)
        {
            int n = data.Count;
            while (n > 1)
            {
                n--;
                int k = random.next_int(0, n);
                Float2 value = data[k];
                data[k] = data[n];
                data[n] = value;
            }
        }
    }
}
