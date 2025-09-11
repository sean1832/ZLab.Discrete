using System;
using ZLab.Discrete.Core;

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
        /// <param name="nx">Number of voxels along X.</param>
        /// <param name="ny">Number of voxels along Y.</param>
        /// <param name="nz">Number of voxels along Z.</param>
        /// <param name="parallel">If true, uses <code>Parallel.For</code> for line-wise passes.</param>
        /// <returns>Flat row-major SDF (float), positive outside, negative inside.</returns>
        public static float[] FromBinaryMask(ReadOnlySpan<byte> binaryMask, int nx, int ny, int nz, bool parallel)
        {
            if (binaryMask.Length != nx * ny * nz)
                throw new ArgumentException("Mask length does not match nx*ny*nz.");
            if (nx <= 0 || ny <= 0 || nz <= 0)
                throw new ArgumentException("Dimensions must be positive.");
            // build seed grids:
            // distance to foreground (inside): costs are 0 at foreground, INF elsewhere
            // distance to background (outside): costs are 0 at background, INF elsewhere
            const int infinity = 1 << 28; // large but not too large to avoid overflow
            int[] seedsToForeground = new int[nx * ny * nz];
            int[] seedsToBackground = new int[nx * ny * nz];
            for (int i = 0; i < binaryMask.Length; i++)
            {
                bool isForeground = binaryMask[i] != 0;
                seedsToForeground[i] = isForeground ? 0 : infinity;
                seedsToBackground[i] = isForeground ? infinity : 0;
            }
            // compute exact squared distance to nearest foreground and background
            int[] squaredToForeground = Edt3D.ExactSquaredIsotropic(seedsToForeground, nx, ny, nz, parallel);
            int[] squaredToBackground = Edt3D.ExactSquaredIsotropic(seedsToBackground, nx, ny, nz, parallel);
            // combine into signed distance field
            // SDF = +sqrt(distToBackground) - sqrt(distToForeground)
            float[] sdf = new float[nx * ny * nz];
            for (int i = 0; i < sdf.Length; i++)
            {
                float outside = (float)Math.Sqrt(squaredToBackground[i]);
                float inside = (float)Math.Sqrt(squaredToForeground[i]);
                sdf[i] = inside - outside; // > 0 outside, < 0 inside
            }
            return sdf;
        }


        /// <summary>
        /// Computes a 3D signed distance field (SDF) from a binary mask with anisotropic voxel spacing using exact Euclidean distance transform (EDT).
        /// </summary>
        /// <param name="binaryMask">Flattened row-major mask (z*nx*ny + y*nx + x). Value 1 = foreground (inside), 0 = background (outside).</param>
        /// <param name="sdf">Preallocated output span for SDF (float). Must be same length as binaryMask.</param>
        /// <param name="nx">Number of voxels along X.</param>
        /// <param name="ny">Number of voxels along Y.</param>
        /// <param name="nz">Number of voxels along Z.</param>
        /// <param name="spacingX">Voxel spacing along X axis (must be positive).</param>
        /// <param name="spacingY">Voxel spacing along Y axis (must be positive).</param>
        /// <param name="spacingZ">Voxel spacing along Z axis (must be positive).</param>
        /// <param name="parallel">If true, lines along X, Y, and Z are processed in parallel.</param>
        /// <returns>Flattened row-major SDF (float). Positive outside, negative inside, zero on boundary, measured in physical units.</returns>
        public static void FromBinaryMaskAnisotropic(
            ReadOnlySpan<byte> binaryMask, Span<float> sdf,
            int nx, int ny, int nz,
            double spacingX, double spacingY, double spacingZ,
            bool parallel)
        {
            if (sdf.Length != binaryMask.Length)
                throw new ArgumentException("sdf length must match binaryMask length.");
            if (binaryMask.Length != nx * ny * nz)
                throw new ArgumentException("binaryMask length must be nx*ny*nz.");
            if (spacingX <= 0 || spacingY <= 0 || spacingZ <= 0)
                throw new ArgumentOutOfRangeException(nameof(spacingX), "Voxel spacings must be positive.");

            const double infinity = 1e30;

            double[] seedsToForeground = new double[binaryMask.Length];
            double[] seedsToBackground = new double[binaryMask.Length];

            for (int i = 0; i < binaryMask.Length; i++)
            {
                bool isForeground = binaryMask[i] != 0;
                seedsToForeground[i] = isForeground ? 0.0 : infinity;
                seedsToBackground[i] = isForeground ? infinity : 0.0;
            }

            double[] squaredToForeground = Edt3D.ExactSquaredAnisotropic(seedsToForeground, nx, ny, nz, spacingX, spacingY, spacingZ, parallel);
            double[] squaredToBackground = Edt3D.ExactSquaredAnisotropic(seedsToBackground, nx, ny, nz, spacingX, spacingY, spacingZ, parallel);

            for (int i = 0; i < sdf.Length; i++)
            {
                float outside = (float)Math.Sqrt(squaredToBackground[i]);
                float inside = (float)Math.Sqrt(squaredToForeground[i]);
                sdf[i] = inside - outside; // <- assign to output span
            }
        }

        /// <summary>
        /// Compute SDF from a 3D *ternary* mask:
        /// <code>0 = Outside, 1 = Inside, 2 = Intersecting (boundary/zero set).</code>
        /// Isotropic voxels (unit spacing). Row-major (x fastest).
        /// </summary>
        /// <param name="ternaryMask">Flattened row-major mask (z*nx*ny + y*nx + x). Value 0 = Outside, 1 = Inside, 2 = Intersecting (boundary/zero set).</param>
        /// <param name="sdf">Preallocated output span for SDF (float). Must be same length as binaryMask.</param>
        /// <param name="nx">Number of voxels along X.</param>
        /// <param name="ny">Number of voxels along Y.</param>
        /// <param name="nz">Number of voxels along Z.</param>
        /// <param name="parallel">If true, lines along X, Y, and Z are processed in parallel.</param>
        /// <returns>Returns SDF with outside LARGER THAN 0, inside SMALLER THAN 0, boundary == 0 (in physical units).</returns>
        public static void FromTernaryMask(ReadOnlySpan<byte> ternaryMask, Span<float> sdf, int nx, int ny, int nz, bool parallel)
        {
            if (sdf.Length != ternaryMask.Length)
                throw new ArgumentException("sdf length must match ternaryMask length.");
            if (ternaryMask.Length != nx * ny * nz)
                throw new ArgumentException("Mask length must be nx*ny*nz.");
            if (nx <= 0 || ny <= 0 || nz <= 0)
                throw new ArgumentException("Dimensions must be positive.");

            // Build two seed volumes with the boundary in BOTH:
            //  - insideFG: Inside U Intersecting
            //  - outsideFG: Outside U Intersecting
            const int infinity = 1 << 28;
            int n = ternaryMask.Length;
            int[] seedsToInsideFg = new int[n];
            int[] seedsToOutsideFg = new int[n];

            for (int i = 0; i < n; i++)
            {
                byte s = ternaryMask[i]; // 0=Outside,1=Inside,2=Intersecting
                bool inInsideFg = (s == 1) || (s == 2);
                bool inOutsideFg = (s == 0) || (s == 2);

                seedsToInsideFg[i] = inInsideFg ? 0 : infinity;
                seedsToOutsideFg[i] = inOutsideFg ? 0 : infinity;
            }

            // Exact squared distances to each foreground
            int[] sqToInside = Edt3D.ExactSquaredIsotropic(seedsToInsideFg, nx, ny, nz, parallel);
            int[] sqToOutside = Edt3D.ExactSquaredIsotropic(seedsToOutsideFg, nx, ny, nz, parallel);

            // SDF = dist(outsideFG) - dist(insideFG)
            for (int i = 0; i < n; i++)
                sdf[i] = (float)Math.Sqrt(sqToInside[i]) - (float)Math.Sqrt(sqToOutside[i]);

            // Hard snap exact boundary to zero (removes tiny FP noise)
            for (int i = 0; i < n; i++)
                if (ternaryMask[i] == 2) sdf[i] = 0f;
        }

        /// <summary>
        /// Compute SDF from a 3D *ternary* mask:
        /// <code>0 = Outside, 1 = Inside, 2 = Intersecting (boundary/zero set).</code>
        /// Anisotropic voxels (spacingX/Y/Z). Row-major (x fastest).
        /// </summary>
        /// <param name="ternaryMask">Flattened row-major mask (z*nx*ny + y*nx + x). Value 0 = Outside, 1 = Inside, 2 = Intersecting (boundary/zero set).</param>
        /// <param name="sdf">Preallocated output span for SDF (float). Must be same length as binaryMask.</param>
        /// <param name="nx">Number of voxels along X.</param>
        /// <param name="ny">Number of voxels along Y.</param>
        /// <param name="nz">Number of voxels along Z.</param>
        /// <param name="spacingX">Voxel spacing along X axis (must be positive).</param>
        /// <param name="spacingY">Voxel spacing along Y axis (must be positive).</param>
        /// <param name="spacingZ">Voxel spacing along Z axis (must be positive).</param>
        /// <param name="parallel">If true, lines along X, Y, and Z are processed in parallel.</param>
        /// <returns>Returns SDF with outside LARGER THAN 0, inside SMALLER THAN 0, boundary == 0 (in physical units).</returns>
        public static void FromTernaryMaskAnisotropic(
            ReadOnlySpan<byte> ternaryMask, Span<float> sdf,
            int nx, int ny, int nz,
            double spacingX, double spacingY, double spacingZ,
            bool parallel)
        {
            if (sdf.Length != ternaryMask.Length)
                throw new ArgumentException("sdf length must match ternaryMask length.");
            if (ternaryMask.Length != nx * ny * nz)
                throw new ArgumentException("Mask length must be nx*ny*nz.");
            if (spacingX <= 0 || spacingY <= 0 || spacingZ <= 0)
                throw new ArgumentOutOfRangeException(nameof(spacingX), "Spacings must be positive.");

            int n = ternaryMask.Length;
            const double INF = 1e30;

            double[] seedsToInsideFg = new double[n];
            double[] seedsToOutsideFg = new double[n];

            for (int i = 0; i < n; i++)
            {
                byte s = ternaryMask[i]; // 0=Outside,1=Inside,2=Intersecting
                bool inInsideFg = (s == 1) || (s == 2);
                bool inOutsideFg = (s == 0) || (s == 2);

                seedsToInsideFg[i] = inInsideFg ? 0.0 : INF;
                seedsToOutsideFg[i] = inOutsideFg ? 0.0 : INF;
            }

            double[] sqToInside = Edt3D.ExactSquaredAnisotropic(
                seedsToInsideFg, nx, ny, nz, spacingX, spacingY, spacingZ, parallel);

            double[] sqToOutside = Edt3D.ExactSquaredAnisotropic(
                seedsToOutsideFg, nx, ny, nz, spacingX, spacingY, spacingZ, parallel);

            for (int i = 0; i < n; i++)
                sdf[i] = (float)Math.Sqrt(sqToInside[i]) - (float)Math.Sqrt(sqToOutside[i]);

            // Exact zero on boundary
            for (int i = 0; i < n; i++)
                if (ternaryMask[i] == 2) sdf[i] = 0f;
        }
    }
}
