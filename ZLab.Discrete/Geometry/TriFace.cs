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

        /// <summary>
        /// Checks if the triangle face is valid (all indices are non-negative).
        /// </summary>
        public bool IsValid => A >= 0 && B >= 0 && C >= 0;

        /// <summary>
        /// Creates a triangle face from three vertex indices.
        /// </summary>
        /// <param name="a">first vertex index</param>
        /// <param name="b">second vertex index</param>
        /// <param name="c">third vertex index</param>
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
        /// <param name="vertices">Input vertices array</param>
        /// <param name="v0">Output of first vertex</param>
        /// <param name="v1">Output of second vertex</param>
        /// <param name="v2">Output of third vertex</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void QueryVertices(Vector3[] vertices, out Vector3 v0, out Vector3 v1, out Vector3 v2)
        {
            v0 = vertices[A]; v1 = vertices[B]; v2 = vertices[C];
        }


        // Equality & hashing
        /// <summary>
        /// Checks if two triangle faces are equal (same vertex indices in the same order).
        /// </summary>
        /// <param name="other">other triangle face to compare</param>
        /// <returns>true if equal, false otherwise</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(TriFace other) => A == other.A && B == other.B && C == other.C;

        /// <summary>
        /// Checks if two triangle faces are equal (same vertex indices in the same order).
        /// </summary>
        /// <param name="obj">other object to compare</param>
        /// <returns>true if equal, false otherwise</returns>
        public override bool Equals(object? obj) => obj is TriFace t && Equals(t);

        /// <summary>
        /// Computes a hash code for the triangle face.
        /// </summary>
        /// <returns>hash code</returns>
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

        /// <summary>
        /// Equality operator for triangle faces.
        /// </summary>
        /// <param name="left">left triangle face</param>
        /// <param name="right">right triangle face</param>
        /// <returns>true if the triangle faces are equal, false otherwise</returns>
        public static bool operator ==(TriFace left, TriFace right) => left.Equals(right);

        /// <summary>
        /// Inequality operator for triangle faces.
        /// </summary>
        /// <param name="left">left triangle face</param>
        /// <param name="right">right triangle face</param>
        /// <returns>true if the triangle faces are not equal, false otherwise</returns>
        public static bool operator !=(TriFace left, TriFace right) => !left.Equals(right);

        /// <summary>
        /// String representation of the triangle face.
        /// </summary>
        /// <returns>string</returns>
        public override string ToString() => $"TriFace32 [A:{A},B:{B},C:{C}]";
    }
}
