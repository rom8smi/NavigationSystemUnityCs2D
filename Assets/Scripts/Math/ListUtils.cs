using System.Collections.Generic;
using System.Linq;

namespace GenericCode
{
    public static class ListUtils
    {
        public static void Resize<T>(this List<T> list, int sz)
        {
            int cur = list.Count;
            if (sz < cur)
            {
                list.RemoveRange(sz, cur - sz);
            }
            else if (sz > cur)
            {
                list.AddRange(Enumerable.Repeat(default(T), sz - cur));
            }
        }
    }
}
