using System;
using System.Buffers;

namespace ZLab.Discrete.Algorithms.DistanceTransforms
{
    /// <summary>
    /// 3D Signed Distance Field (SDF) from binary or ternary masks using exact Euclidean distance transform (EDT). 
    /// Build on top of <see cref="Edt3D"/>.
    /// </summary>
    internal static class Sdf3D
    {
        /// <summary>
        /// Compute a signed distance field (SDF) from a 3D binary mask (0 = background/outside, 1 = foreground/inside).
        /// Isotropic voxels (unit spacing).
        /// </summary>
        /// <param name="binaryMask">Flat row-major volume of size nx*ny*nz (x changes fastest, then y, then z).</param>
        /// <param name="outSdf"><b>OUTPUT</b>: Preallocated output span for SDF (float). Must be same length as binaryMask.</param>
        /// <param name="nx">Number of voxels along X.</param>
        /// <param name="ny">Number of voxels along Y.</param>
        /// <param name="nz">Number of voxels along Z.</param>
        /// <param name="parallel">If true, uses <code>Parallel.For</code> for line-wise passes.</param>
        /// <remarks><paramref name="outSdf"/> will be mutated to contain the signed distance field (SDF).</remarks>
        public static void FromBinaryMask(ReadOnlySpan<byte> binaryMask, Span<float> outSdf,  int nx, int ny, int nz, bool parallel)
        {
            int total = nx * ny * nz;
            if (binaryMask.Length != total)
                throw new ArgumentException("Mask length does not match nx*ny*nz.");
            if (outSdf.Length != total)
                throw new ArgumentException("sdf length must match binaryMask length.");
            if (nx <= 0 || ny <= 0 || nz <= 0)
                throw new ArgumentException("Dimensions must be positive.");
            // build seed grids:
            // distance to foreground (inside): costs are 0 at foreground, INF elsewhere
            // distance to background (outside): costs are 0 at background, INF elsewhere
            const int infinity = 1 << 28; // large but not too large to avoid overflow

            // temp buffer for EDT
            int[] bufferA = ArrayPool<int>.Shared.Rent(total);
            int[] bufferB = ArrayPool<int>.Shared.Rent(total);
            int[] bufferC = ArrayPool<int>.Shared.Rent(total); 
            int[] bufferD = ArrayPool<int>.Shared.Rent(total);
            try
            {
                Span<int> seedForeground = bufferA.AsSpan(0, total);
                Span<int> seedBackground = bufferB.AsSpan(0, total);
                
                for (int i = 0; i < binaryMask.Length; i++)
                {
                    bool isForeground = binaryMask[i] != 0;
                    seedForeground[i] = isForeground ? 0 : infinity;
                    seedBackground[i] = isForeground ? infinity : 0;
                }

                Span<int> sqForeground = bufferC.AsSpan(0, total);
                Span<int> sqBackground = bufferD.AsSpan(0, total);
                // compute exact squared distance to nearest foreground and background
                Edt3D.ExactSquaredIsotropic(seedForeground, sqForeground, nx, ny, nz, parallel);
                Edt3D.ExactSquaredIsotropic(seedBackground, sqBackground, nx, ny, nz, parallel);
                // combine into signed distance field
                // SDF = sqrt(distToForeground) - sqrt(distToBackground)
                for (int i = 0; i < outSdf.Length; i++)
                {
                    float outside = (float)Math.Sqrt(sqBackground[i]);
                    float inside = (float)Math.Sqrt(sqForeground[i]);
                    outSdf[i] = inside - outside; // > 0 outside, < 0 inside
                }
            }
            finally
            {
                ArrayPool<int>.Shared.Return(bufferB);
                ArrayPool<int>.Shared.Return(bufferA);
                ArrayPool<int>.Shared.Return(bufferC);
                ArrayPool<int>.Shared.Return(bufferD);
            }
        }


        /// <summary>
        /// Computes a 3D signed distance field (SDF) from a binary mask with anisotropic voxel spacing using exact Euclidean distance transform (EDT).
        /// </summary>
        /// <param name="binaryMask">Flattened row-major mask (z*nx*ny + y*nx + x). Value 1 = foreground (inside), 0 = background (outside).</param>
        /// <param name="outSdf">Preallocated output span for SDF (float). Must be same length as binaryMask.</param>
        /// <param name="nx">Number of voxels along X.</param>
        /// <param name="ny">Number of voxels along Y.</param>
        /// <param name="nz">Number of voxels along Z.</param>
        /// <param name="spacingX">Voxel spacing along X axis (must be positive).</param>
        /// <param name="spacingY">Voxel spacing along Y axis (must be positive).</param>
        /// <param name="spacingZ">Voxel spacing along Z axis (must be positive).</param>
        /// <param name="parallel">If true, lines along X, Y, and Z are processed in parallel.</param>
        /// <remarks><paramref name="outSdf"/> will be mutated to contain the signed distance field (SDF).</remarks>
        public static void FromBinaryMaskAnisotropic(
            ReadOnlySpan<byte> binaryMask, Span<float> outSdf,
            int nx, int ny, int nz,
            double spacingX, double spacingY, double spacingZ,
            bool parallel)
        {
            int total = nx * ny * nz;
            if (outSdf.Length != binaryMask.Length)
                throw new ArgumentException("sdf length must match binaryMask length.");
            if (binaryMask.Length != total)
                throw new ArgumentException("binaryMask length must be nx*ny*nz.");
            if (spacingX <= 0 || spacingY <= 0 || spacingZ <= 0)
                throw new ArgumentOutOfRangeException(nameof(spacingX), "Voxel spacings must be positive.");

            const double infinity = 1e30;

            double[] bufferA = ArrayPool<double>.Shared.Rent(total);
            double[] bufferB = ArrayPool<double>.Shared.Rent(total);
            double[] bufferC = ArrayPool<double>.Shared.Rent(total);
            double[] bufferD = ArrayPool<double>.Shared.Rent(total);
            try
            {
                Span<double> seedForeground = bufferA.AsSpan(0, total);
                Span<double> seedBackground = bufferB.AsSpan(0, total);

                for (int i = 0; i < binaryMask.Length; i++)
                {
                    bool isForeground = binaryMask[i] != 0;
                    seedForeground[i] = isForeground ? 0.0 : infinity;
                    seedBackground[i] = isForeground ? infinity : 0.0;
                }
                Span<double> sqForeground = bufferC.AsSpan(0, total);
                Span<double> sqBackground = bufferD.AsSpan(0, total);

                Edt3D.ExactSquaredAnisotropic(seedForeground, sqForeground, nx, ny, nz, spacingX,
                    spacingY, spacingZ, parallel);
                Edt3D.ExactSquaredAnisotropic(seedBackground, sqBackground, nx, ny, nz, spacingX,
                    spacingY, spacingZ, parallel);

                for (int i = 0; i < outSdf.Length; i++)
                {
                    float outside = (float)Math.Sqrt(sqBackground[i]);
                    float inside = (float)Math.Sqrt(sqForeground[i]);
                    outSdf[i] = inside - outside; // <- assign to output span
                }
            }
            finally
            {
                ArrayPool<double>.Shared.Return(bufferA);
                ArrayPool<double>.Shared.Return(bufferB);
                ArrayPool<double>.Shared.Return(bufferC);
                ArrayPool<double>.Shared.Return(bufferD);
            }
        }

        /// <summary>
        /// Compute SDF from a 3D *ternary* mask:
        /// <code>0 = Outside, 1 = Inside, 2 = Intersecting (boundary/zero set).</code>
        /// Isotropic voxels (unit spacing). Row-major (x fastest).
        /// </summary>
        /// <param name="ternaryMask">Flattened row-major mask (z*nx*ny + y*nx + x). Value 0 = Outside, 1 = Inside, 2 = Intersecting (boundary/zero set).</param>
        /// <param name="outSdf">Preallocated output span for SDF (float). Must be same length as binaryMask.</param>
        /// <param name="nx">Number of voxels along X.</param>
        /// <param name="ny">Number of voxels along Y.</param>
        /// <param name="nz">Number of voxels along Z.</param>
        /// <param name="parallel">If true, lines along X, Y, and Z are processed in parallel.</param>
        /// <remarks><paramref name="outSdf"/> will be mutated to contain the signed distance field (SDF).</remarks>
        public static void FromTernaryMask(ReadOnlySpan<byte> ternaryMask, Span<float> outSdf, int nx, int ny, int nz, bool parallel)
        {
            int total = nx * ny * nz;
            if (outSdf.Length != ternaryMask.Length)
                throw new ArgumentException("sdf length must match ternaryMask length.");
            if (ternaryMask.Length != total)
                throw new ArgumentException("Mask length must be nx*ny*nz.");
            if (nx <= 0 || ny <= 0 || nz <= 0)
                throw new ArgumentException("Dimensions must be positive.");

            // Build two seed volumes with the boundary in BOTH:
            //  - insideFG: Inside U Intersecting
            //  - outsideFG: Outside U Intersecting
            const int infinity = 1 << 28;

            int[] bufferA = ArrayPool<int>.Shared.Rent(total);
            int[] bufferB = ArrayPool<int>.Shared.Rent(total);
            int[] bufferC = ArrayPool<int>.Shared.Rent(total);
            int[] bufferD = ArrayPool<int>.Shared.Rent(total);
            try
            {
                Span<int> seedForeground = bufferA.AsSpan(0, total);
                Span<int> seedBackground = bufferB.AsSpan(0, total);
                for (int i = 0; i < total; i++)
                {
                    byte s = ternaryMask[i]; // 0=Outside,1=Inside,2=Intersecting
                    bool inInsideFg = (s == 1) || (s == 2);
                    bool inOutsideFg = (s == 0) || (s == 2);

                    seedForeground[i] = inInsideFg ? 0 : infinity;
                    seedBackground[i] = inOutsideFg ? 0 : infinity;
                }
                Span<int> sqForeground = bufferC.AsSpan(0, total);
                Span<int> sqBackground = bufferD.AsSpan(0, total);

                // Exact squared distances to each foreground
                Edt3D.ExactSquaredIsotropic(seedForeground, sqForeground, nx, ny, nz, parallel);
                Edt3D.ExactSquaredIsotropic(seedBackground, sqBackground, nx, ny, nz, parallel);

                // SDF = sqrt(distToForeground) - sqrt(distToBackground)
                for (int i = 0; i < total; i++)
                    outSdf[i] = (float)Math.Sqrt(sqForeground[i]) - (float)Math.Sqrt(sqBackground[i]);

                // Hard snap exact boundary to zero (removes tiny FP noise)
                for (int i = 0; i < total; i++)
                    if (ternaryMask[i] == 2)
                        outSdf[i] = 0f;
            }
            finally
            {
                ArrayPool<int>.Shared.Return(bufferA);
                ArrayPool<int>.Shared.Return(bufferB);
                ArrayPool<int>.Shared.Return(bufferC);
                ArrayPool<int>.Shared.Return(bufferD);
            }
        }

        /// <summary>
        /// Compute SDF from a 3D *ternary* mask:
        /// <code>0 = Outside, 1 = Inside, 2 = Intersecting (boundary/zero set).</code>
        /// Anisotropic voxels (spacingX/Y/Z). Row-major (x fastest).
        /// </summary>
        /// <param name="ternaryMask">Flattened row-major mask (z*nx*ny + y*nx + x). Value 0 = Outside, 1 = Inside, 2 = Intersecting (boundary/zero set).</param>
        /// <param name="outSdf">Preallocated output span for SDF (float). Must be same length as binaryMask.</param>
        /// <param name="nx">Number of voxels along X.</param>
        /// <param name="ny">Number of voxels along Y.</param>
        /// <param name="nz">Number of voxels along Z.</param>
        /// <param name="spacingX">Voxel spacing along X axis (must be positive).</param>
        /// <param name="spacingY">Voxel spacing along Y axis (must be positive).</param>
        /// <param name="spacingZ">Voxel spacing along Z axis (must be positive).</param>
        /// <param name="parallel">If true, lines along X, Y, and Z are processed in parallel.</param>
        /// <remarks><paramref name="outSdf"/> will be mutated to contain the signed distance field (SDF).</remarks>
        public static void FromTernaryMaskAnisotropic(
            ReadOnlySpan<byte> ternaryMask, Span<float> outSdf,
            int nx, int ny, int nz,
            double spacingX, double spacingY, double spacingZ,
            bool parallel)
        {
            int total = nx * ny * nz;
            if (outSdf.Length != ternaryMask.Length)
                throw new ArgumentException("sdf length must match ternaryMask length.");
            if (ternaryMask.Length != total)
                throw new ArgumentException("Mask length must be nx*ny*nz.");
            if (spacingX <= 0 || spacingY <= 0 || spacingZ <= 0)
                throw new ArgumentOutOfRangeException(nameof(spacingX), "Spacings must be positive.");

            const double INF = 1e30;

            double[] bufferA = ArrayPool<double>.Shared.Rent(total);
            double[] bufferB = ArrayPool<double>.Shared.Rent(total);
            double[] bufferC = ArrayPool<double>.Shared.Rent(total);
            double[] bufferD = ArrayPool<double>.Shared.Rent(total);
            try
            {
                Span<double> seedForeground = bufferA.AsSpan(0, total);
                Span<double> seedBackground = bufferB.AsSpan(0, total);

                for (int i = 0; i < total; i++)
                {
                    byte s = ternaryMask[i]; // 0=Outside,1=Inside,2=Intersecting
                    bool inInsideFg = (s == 1) || (s == 2);
                    bool inOutsideFg = (s == 0) || (s == 2);

                    seedForeground[i] = inInsideFg ? 0.0 : INF;
                    seedBackground[i] = inOutsideFg ? 0.0 : INF;
                }

                Span<double> sqForeground = bufferC.AsSpan(0, total);
                Span<double> sqBackground = bufferD.AsSpan(0, total);

                Edt3D.ExactSquaredAnisotropic(seedForeground, sqForeground, nx, ny, nz, spacingX, spacingY, spacingZ, parallel);
                Edt3D.ExactSquaredAnisotropic(seedBackground, sqBackground, nx, ny, nz, spacingX, spacingY, spacingZ, parallel);

                for (int i = 0; i < total; i++)
                    outSdf[i] = (float)Math.Sqrt(sqForeground[i]) - (float)Math.Sqrt(sqBackground[i]);

                // Exact zero on boundary
                for (int i = 0; i < total; i++)
                    if (ternaryMask[i] == 2)
                        outSdf[i] = 0f;
            }
            finally
            {
                ArrayPool<double>.Shared.Return(bufferA);
                ArrayPool<double>.Shared.Return(bufferB);
                ArrayPool<double>.Shared.Return(bufferC);
                ArrayPool<double>.Shared.Return(bufferD);
            }
        }
    }
}
