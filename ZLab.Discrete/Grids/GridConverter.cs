using System.Numerics;
using System.Runtime.CompilerServices;
using ZLab.Discrete.Core;

namespace ZLab.Discrete.Grids
{
    internal static class GridConverter
    {
        private const float Eps = 1e-6f;

        // ---------------------------
        // Global lattice (origin = (0,0,0))
        // ---------------------------

        // --------------------------- World to Min conversions ---------------------------
        /// <summary>
        /// World -> grid (min-corner cell index) for global grids
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static (int x, int y, int z) WorldToGridMin(Vector3 p, Vector3 size)
            => ((int)MathFx.Floor((p.X + Eps) / size.X),
                (int)MathFx.Floor((p.Y + Eps) / size.Y),
                (int)MathFx.Floor((p.Z + Eps) / size.Z));

        /// <summary>
        /// World -> grid (min-corner cell index) given a world-space grid origin (min corner) for local grids
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static (int x, int y, int z) WorldToGridMin(Vector3 p, Vector3 size, Vector3 origin)
            => ((int)MathFx.Floor((p.X - origin.X + Eps) / size.X),
                (int)MathFx.Floor((p.Y - origin.Y + Eps) / size.Y),
                (int)MathFx.Floor((p.Z - origin.Z + Eps) / size.Z));

        // --------------------------- World to Max conversions ---------------------------
        /// <summary>
        /// World -> grid (max-corner cell index, inclusive) for global grids
        /// </summary>
        /// <remarks>Use this for the MAX CORNER when you want an inclusive index</remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static (int x, int y, int z) WorldToGridMaxInclusive(Vector3 p, Vector3 size)
            => ((int)MathFx.Floor((p.X - Eps) / size.X),
                (int)MathFx.Floor((p.Y - Eps) / size.Y),
                (int)MathFx.Floor((p.Z - Eps) / size.Z));

        /// <summary>
        /// World -> grid (max-corner cell index, inclusive) given a world-space grid origin (min corner) for local grids
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static (int x, int y, int z) WorldToGridMaxInclusive(Vector3 p, Vector3 size, Vector3 origin)
            => ((int)MathFx.Floor((p.X - origin.X - Eps) / size.X),
                (int)MathFx.Floor((p.Y - origin.Y - Eps) / size.Y),
                (int)MathFx.Floor((p.Z - origin.Z - Eps) / size.Z));
        // --------------------------- Index to Min conversions ---------------------------
        /// <summary>
        /// Grid index -> world min corner with origin
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 IndexToMinCorner(int x, int y, int z, Vector3 size)
            => new(x * size.X, y * size.Y, z * size.Z);

        /// <summary>
        /// Grid index -> world min corner with origin (local grid)
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 IndexToMinCorner(int x, int y, int z, Vector3 size, Vector3 origin)
            => new(origin.X + x * size.X, origin.Y + y * size.Y, origin.Z + z * size.Z);

        // --------------------------- Index to Max conversions ---------------------------
        /// <summary>
        /// Grid index -> world max corner
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 IndexToMaxCorner(int x, int y, int z, Vector3 size)
            => new((x + 1) * size.X, (y + 1) * size.Y, (z + 1) * size.Z);

        /// <summary>
        /// Grid index -> world max corner with origin (local grid)
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 IndexToMaxCorner(int x, int y, int z, Vector3 size, Vector3 origin)
            => new(origin.X + (x + 1) * size.X, origin.Y + (y + 1) * size.Y, origin.Z + (z + 1) * size.Z);
    }
}

