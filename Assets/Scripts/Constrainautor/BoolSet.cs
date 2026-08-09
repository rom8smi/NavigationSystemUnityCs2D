using System.Collections.Generic;
using GenericCode;

namespace DelaunatorSharp
{
    public class BoolSet
    {
        public List<bool> bs;

        public BoolSet()
        {
            bs = new List<bool>();
        }

        public void Create(int len)
        {
            bs.Resize(len);

            for (int i = 0; i < len; i++)
            {
                bs[i] = false;
            }
        }

        public void Clear()
        {
            bs.Clear();
        }

        public void Add(int idx)
        {
            bs[idx] = true;
        }

        public void Remove(int idx)
        {
            bs[idx] = false;
        }

        public bool Set(int idx, bool val)
        {
            bs[idx] = val;
            return val;
        }

        public bool Has(int idx)
        {
            return bs[idx];
        }
    }
}
