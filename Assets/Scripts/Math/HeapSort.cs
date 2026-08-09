using System.Collections.Generic;

namespace GenericCode
{
    public static class HeapSort
    {
        // Based on C# version from https://www.geeksforgeeks.org/iterative-heap-sort/
        public static void Sort(List<int> indices, List<float> data, int n)
        {
            for (int i = 1; i < n; i++)
            {
                // if child is bigger than parent
                if (data[i] > data[(i - 1) / 2])
                {
                    int j = i;

                    // swap child and parent until
                    // parent is smaller
                    while (data[j] > data[(j - 1) / 2])
                    {
                        float temp = data[j];
                        data[j] = data[(j - 1) / 2];
                        data[(j - 1) / 2] = temp;

                        int tempInd = indices[j];
                        indices[j] = indices[(j - 1) / 2];
                        indices[(j - 1) / 2] = tempInd;

                        j = (j - 1) / 2;
                    }
                }
            }

            for (int i = n - 1; i > 0; i--)
            {
                // swap value of first indexed
                // with last indexed

                float temp = data[0];
                data[0] = data[i];
                data[i] = temp;

                int tempInd = indices[0];
                indices[0] = indices[i];
                indices[i] = tempInd;

                // maintaining heap property
                // after each swapping
                int j = 0, index;

                do
                {
                    index = (2 * j + 1);

                    // if left child is smaller than
                    // right child point index variable
                    // to right child
                    if (index < (i - 1) && data[index] < data[index + 1])
                    {
                        index++;
                    }

                    // if parent is smaller than child
                    // then swapping parent with child
                    // having higher value
                    if (index < i && data[j] < data[index])
                    {
                        float temp1 = data[j];
                        data[j] = data[index];
                        data[index] = temp1;

                        int tempInd1 = indices[j];
                        indices[j] = indices[index];
                        indices[index] = tempInd1;
                    }

                    j = index;

                } while (index < i);
            }
        }

        public static void Sort(List<int> data, int n)
        {
            for (int i = 1; i < n; i++)
            {
                // if child is bigger than parent
                if (data[i] > data[(i - 1) / 2])
                {
                    int j = i;

                    // swap child and parent until
                    // parent is smaller
                    while (data[j] > data[(j - 1) / 2])
                    {
                        int temp = data[j];
                        data[j] = data[(j - 1) / 2];
                        data[(j - 1) / 2] = temp;

                        j = (j - 1) / 2;
                    }
                }
            }

            for (int i = n - 1; i > 0; i--)
            {
                // swap value of first indexed
                // with last indexed

                int temp = data[0];
                data[0] = data[i];
                data[i] = temp;

                // maintaining heap property
                // after each swapping
                int j = 0, index;

                do
                {
                    index = (2 * j + 1);

                    // if left child is smaller than
                    // right child point index variable
                    // to right child
                    if (index < (i - 1) && data[index] < data[index + 1])
                    {
                        index++;
                    }

                    // if parent is smaller than child
                    // then swapping parent with child
                    // having higher value
                    if (index < i && data[j] < data[index])
                    {
                        int temp1 = data[j];
                        data[j] = data[index];
                        data[index] = temp1;
                    }

                    j = index;

                } while (index < i);
            }
        }
    }
}
