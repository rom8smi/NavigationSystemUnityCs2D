namespace GenericCode
{
    [System.Serializable]
    public struct Float2
    {
        public float x;
        public float y;

        public Float2(float x, float y)
        {
            this.x = x;
            this.y = y;
        }

        public float GetAxis(int axis)
        {
            if (axis == 0)
            {
                return x;
            }
            return y;
        }

        public static Float2 operator +(Float2 a, Float2 b)
        {
            return new Float2(a.x + b.x, a.y + b.y);
        }

        public static Float2 operator -(Float2 a, Float2 b)
        {
            return new Float2(a.x - b.x, a.y - b.y);
        }

        public static Float2 operator -(Float2 a)
        {
            return new Float2(-a.x, -a.y);
        }

        public static Float2 operator *(Float2 a, float b)
        {
            return new Float2(a.x * b, a.y * b);
        }

        public static Float2 operator *(float a, Float2 b)
        {
            return new Float2(a * b.x, a * b.y);
        }

        public static Float2 operator /(Float2 a, float b)
        {
            return new Float2(a.x / b, a.y / b);
        }

        public static float Dot(Float2 a, Float2 b)
        {
            return a.x * b.x + a.y * b.y;
        }

        public float Length()
        {
            return MathUtils.Sqrt(x * x + y * y);
        }

        public void Normalize()
        {
            float l = x * x + y * y;
            if (l != 0)
            {
                l = MathUtils.Sqrt(l);
                x /= l;
                y /= l;
            }
        }

        public Float2 Normalized()
        {
            float p_x = x;
            float p_y = y;
            float l = x * x + y * y;
            if (l != 0)
            {
                l = MathUtils.Sqrt(l);
                p_x /= l;
                p_y /= l;
            }

            return new Float2(p_x, p_y);
        }

        public float LengthSquared()
        {
            return x * x + y * y;
        }

        public static Float2 Zero()
        {
            return new Float2(0.0f, 0.0f);
        }
    }
}
