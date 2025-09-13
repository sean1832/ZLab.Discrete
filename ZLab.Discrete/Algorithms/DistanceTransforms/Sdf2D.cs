using System;

namespace ZLab.Discrete.Algorithms.DistanceTransforms
{
    /// <summary>
    /// 2D Signed Distance Field (SDF) from binary mask using Exact Euclidean Distance Transform (EDT).
    /// Builds on top of <see cref="Edt2D"/>.
    /// </summary>
    internal static class Sdf2D
    {
        /// <summary>
        /// Build a signed distance field (SDF) from a binary mask using exact Euclidean distance.
        /// Foreground (mask=1) is "inside" (negative SDF), background (mask=0) is "outside" (positive SDF).
        /// </summary>
        /// <param name="mask">Flat row-major binary grid (0 = background, 1 = foreground).</param>
        /// <param name="width">Image width.</param>
        /// <param name="height">Image height.</param>
        /// <param name="parallel">If true, rows/columns are processed in parallel.</param>
        /// <returns>Flat row-major SDF in float: positive outside, negative inside, ~0 on the boundary.</returns>
        public static float[] FromBinaryMask(ReadOnlySpan<byte> mask, int width, int height, bool parallel)
        {
            if (mask.Length != width * height)
                throw new ArgumentException("Mask length does not match width*height.");
            if (width <= 0 || height <= 0)
                throw new ArgumentException("Width and height must be positive.");

            // build seed grids:
            // distance to foreground (inside): costs are 0 at foreground, INF elsewhere
            // distance to background (outside): costs are 0 at background, INF elsewhere
            const int infinity = 1 << 28; // large but not too large to avoid overflow
            int[] seedsToForeground = new int[width * height];
            int[] seedsToBackground = new int[width * height];

            for (int i = 0; i < mask.Length; i++)
            {
                bool isForeground = mask[i] != 0;
                seedsToForeground[i] = isForeground ? 0 : infinity;
                seedsToBackground[i] = isForeground ? infinity : 0;
            }

            // compute exact squared distance to nearest foreground and background
            int[] squaredToForeground = Edt2D.ExactSquaredIsotropic(seedsToForeground, width, height, parallel);
            int[] squaredToBackground = Edt2D.ExactSquaredIsotropic(seedsToBackground, width, height, parallel);

            // combine into signed distance field
            // SDF = +sqrt(distToBackground) - sqrt(distToForeground)
            float[] sdf = new float[width * height];
            for (int i = 0; i < sdf.Length; i++)
            {
                float distToForeground = MathF.Sqrt(squaredToForeground[i]);
                float distToBackground = MathF.Sqrt(squaredToBackground[i]);
                sdf[i] = distToBackground - distToForeground; // > 0 outside, < 0 inside
            }
            return sdf;
        }


        /// <summary>
        /// Build a signed distance field (SDF) from a binary mask when pixel spacing differs along X and Y.
        /// This version is "exact" for Euclidean distance when using the given spacings.
        /// </summary>
        /// <param name="binaryMask">Flat row-major binary grid (0 = background, 1 = foreground).</param>
        /// <param name="width">Image width.</param>
        /// <param name="height">Image height.</param>
        /// <param name="pixelSpacingX">Physical spacing along X (e.g., mm per pixel).</param>
        /// <param name="pixelSpacingY">Physical spacing along Y.</param>
        /// <param name="parallel">If true, rows/columns are processed in parallel.</param>
        /// <returns>Flat row-major SDF in float.</returns>
        public static float[] FromBinaryMaskAnisotropic(
            ReadOnlySpan<byte> binaryMask,
            int width,
            int height,
            double pixelSpacingX,
            double pixelSpacingY,
            bool parallel = true)
        {
            if (binaryMask.Length != width * height)
                throw new ArgumentException("binaryMask length must be width*height.", nameof(binaryMask));
            if (pixelSpacingX <= 0 || pixelSpacingY <= 0)
                throw new ArgumentOutOfRangeException(nameof(pixelSpacingX), "Pixel spacings must be positive.");

            const double infinity = 1e30; // large double
            double[] seedsToForeground = new double[width * height];
            double[] seedsToBackground = new double[width * height];

            for (int i = 0; i < binaryMask.Length; i++)
            {
                bool isForeground = binaryMask[i] != 0;
                seedsToForeground[i] = isForeground ? 0.0 : infinity;
                seedsToBackground[i] = isForeground ? infinity : 0.0;
            }

            double[] squaredToForeground = Edt2D.ExactSquaredAnisotropic(seedsToForeground, width, height, pixelSpacingX, pixelSpacingY, parallel);
            double[] squaredToBackground = Edt2D.ExactSquaredAnisotropic(seedsToBackground, width, height, pixelSpacingX, pixelSpacingY, parallel);

            float[] sdf = new float[width * height];
            for (int i = 0; i < sdf.Length; i++)
            {
                float outsideDistance = (float)Math.Sqrt(squaredToBackground[i]);
                float insideDistance = (float)Math.Sqrt(squaredToForeground[i]);
                sdf[i] = outsideDistance - insideDistance;
            }
            return sdf;
        }
    }
}
