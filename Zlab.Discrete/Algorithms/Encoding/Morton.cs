using System.Runtime.CompilerServices;

namespace ZLab.Discrete.Algorithms.Encoding
{
    internal static class Morton
    {
        // Interleave 21-bit x so its bits occupy every 3rd bit of a 64-bit word.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static ulong Part1By2(ulong x)
        {
            x &= 0x1FFFFF;                            // keep 21 bits
            x = (x | (x << 32)) & 0x1F00000000FFFFUL;
            x = (x | (x << 16)) & 0x1F0000FF0000FFUL;
            x = (x | (x << 8)) & 0x100F00F00F00F00FUL;
            x = (x | (x << 4)) & 0x10C30C30C30C30C3UL;
            x = (x | (x << 2)) & 0x1249249249249249UL;
            return x;
        }

        // Compact the interleaved bits: inverse of Part1By2
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static uint Compact1By2(ulong x)
        {
            x &= 0x1249249249249249UL;
            x = (x ^ (x >> 2)) & 0x10C30C30C30C30C3UL;
            x = (x ^ (x >> 4)) & 0x100F00F00F00F00FUL;
            x = (x ^ (x >> 8)) & 0x1F0000FF0000FFUL;
            x = (x ^ (x >> 16)) & 0x1F00000000FFFFUL;
            x = (x ^ (x >> 32)) & 0x1FFFFFUL;
            return (uint)x;
        }

        /// Encode (ix,iy,iz) -> morton (Z-order). Each must be smaller than 2^21.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ulong Encode(uint ix, uint iy, uint iz)
        {
            return Part1By2(ix) | (Part1By2(iy) << 1) | (Part1By2(iz) << 2);
        }

        /// Decode morton -> (ix,iy,iz).
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Decode(ulong m, out uint ix, out uint iy, out uint iz)
        {
            ix = Compact1By2(m);
            iy = Compact1By2(m >> 1);
            iz = Compact1By2(m >> 2);
        }
    }
}
