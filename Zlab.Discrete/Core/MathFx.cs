using System;
using System.Runtime.CompilerServices;

namespace ZLab.Discrete.Core
{
    /// <summary>
    /// Math float extensions for compatibility between .NET Framework and .NET Core/5+
    /// </summary>
    internal static class MathFx
    {
#if NETFRAMEWORK
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Sqrt(float x) => (float)Math.Sqrt(x);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Abs(float x) => Math.Abs(x);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Min(float a, float b) => Math.Min(a, b);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Max(float a, float b) => Math.Max(a, b);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Floor(float x) => (float)Math.Floor(x);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Clamp(float value, float min, float max) => value < min ? min : (value > max ? max : value);
        public static int Clamp (int value, int min, int max) => value < min ? min : (value > max ? max : value);

#else
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Sqrt(float x) => MathF.Sqrt(x);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Abs(float x)  => MathF.Abs(x);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Min(float a, float b) => MathF.Min(a, b);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Max(float a, float b) => MathF.Max(a, b);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Floor(float x) => MathF.Floor(x);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Clamp(float value, float min, float max) => Math.Clamp(value, min, max);
        public static int Clamp (int value, int min, int max) => Math.Clamp(value, min, max);
#endif
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Lerp(float a, float b, float t) => a + t * (b - a);
    }
}
