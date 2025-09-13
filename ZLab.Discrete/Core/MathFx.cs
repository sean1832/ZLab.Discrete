using System;
using System.Runtime.CompilerServices;

namespace ZLab.Discrete.Core
{
    /// <summary>
    /// Math float extensions for compatibility between .NET Framework and .NET Core/5+
    /// </summary>
    internal static class MathFx
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Lerp(float a, float b, float t) => a + t * (b - a);
    }
}
