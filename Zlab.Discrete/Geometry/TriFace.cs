using System.Numerics;
using System.Runtime.CompilerServices;

namespace ZLab.Discrete.Geometry
{
    /// <summary>
    /// Triangle face defined by three vertex indices (int). (12 bytes)
    /// </summary>
    public readonly struct TriFace
    {
        /// <summary>
        /// Index of the first vertex of the triangle.
        /// </summary>
        public readonly int A; // <- 4 bytes
        /// <summary>
        /// Index of the second vertex of the triangle.
        /// </summary>
        public readonly int B; // <- 4 bytes
        /// <summary>
        /// Index of the third vertex of the triangle.
        /// </summary>
        public readonly int C; // <- 4 bytes

        public bool IsValid => A >= 0 && B >= 0 && C >= 0;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public TriFace(int a, int b, int c)
        {
            A = a;
            B = b;
            C = c;
        }

        /// <summary>
        /// Fetches the vertex positions from the provided vertex array.
        /// </summary>
        /// <param name="vertices">Input vertices array from <see cref="v0"/></param>
        /// <param name="v1">Output of first vertex</param>
        /// <param name="v2">Output of second vertex</param>
        /// <param name="v2">Output of third vertex</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void QueryVertices(Vector3[] vertices, out Vector3 v0, out Vector3 v1, out Vector3 v2)
        {
            v0 = vertices[A]; v1 = vertices[B]; v2 = vertices[C];
        }


        // Equality & hashing
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(TriFace other) => A == other.A && B == other.B && C == other.C;
        public override bool Equals(object? obj) => obj is TriFace t && Equals(t);
        public override int GetHashCode()
        {
            // simple fast mixing
            unchecked
            {
                int h = A;
                h = h * 397 ^ B;
                h = h * 397 ^ C;
                return h;
            }
        }
        public static bool operator ==(TriFace left, TriFace right) => left.Equals(right);
        public static bool operator !=(TriFace left, TriFace right) => !left.Equals(right);

        public override string ToString() => $"TriFace32 [A:{A},B:{B},C:{C}]";
    }
}
