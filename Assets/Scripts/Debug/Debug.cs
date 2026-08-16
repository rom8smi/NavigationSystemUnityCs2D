namespace GenericCode
{
    public static class Debug
    {
        public static void Log(string message)
        {
            UnityEngine.Debug.Log(message);
        }

        public static void IndexAssert(int index, int size, string message)
        {
            if (index < 0 || index >= size)
            {
                UnityEngine.Debug.Log("!!! Index out of range: " + index.ToString() + " | " + size.ToString() + " | " + message);
            }
        }
    }
}
