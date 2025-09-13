using System;
using System.Threading.Tasks;

namespace ZLab.Discrete.Algorithms.DistanceTransforms
{
    /// <summary>
    /// 2D Exact Euclidean Distance Transform (EDT). Builds on top of <see cref="Edt1D"/>.
    /// </summary>
    /// <remarks>
    /// Reference:
    /// <a href="https://cs.brown.edu/people/pfelzens/papers/dt-final.pdf">
    /// P.F.Felzenszwalb, D.P.Huttenlocher (2012). Distance Transforms of Sampled Functions.
    /// </a>
    /// </remarks>
    internal static class Edt2D
    {
        /// <summary>
        /// Computes the exact 2D squared Euclidean distance transform (EDT) for isotropic pixels (unit spacing).
        /// </summary>
        /// <param name="seedCosts">Flattened row-major cost grid (y*width + x). 0 at seeds, INF at non-seeds.</param>
        /// <param name="width">Number of pixels along X.</param>
        /// <param name="height">Number of pixels along Y.</param>
        /// <param name="parallel">If true, rows and columns are processed in parallel.</param>
        /// <returns>Flattened row-major array of squared distances to the nearest seed.</returns>
        public static int[] ExactSquaredIsotropic(int[] seedCosts, int width, int height, bool parallel)
        {
            int[] rowPass = new int[seedCosts.Length];
            int[] result = new int[seedCosts.Length];

            // Pass 1: rows (X direction)
            if (parallel)
                Parallel.For(0, height, (Action<int>)ProcessRow);
            else
                for (int y = 0; y < height; y++) ProcessRow(y);

            // Pass 2: columns (Y direction)
            if (parallel)
                Parallel.For(0, width, (Action<int>)ProcessColumn);
            else
                for (int x = 0; x < width; x++) ProcessColumn(x);
            return result;


            // =============================
            // Local functions
            // =============================
            // Pass 1: rows (X direction)
            void ProcessRow(int y)
            {
                int rowOffset = y * width;
                Span<int> inputRow = seedCosts.AsSpan(rowOffset, width);
                Span<int> outputRow = rowPass.AsSpan(rowOffset, width);
                Edt1D.Transform_Isotropic(inputRow, outputRow);
            }

            // Pass 2: columns (Y direction)
            void ProcessColumn(int x)
            {
                // Extract column into a temporary buffer (reused on stack if width/height are moderate)
                int h = height;
                Span<int> inputColumn = h <= 4096 ? stackalloc int[h] : new int[h];
                Span<int> outputColumn = h <= 4096 ? stackalloc int[h] : new int[h];

                for (int y = 0; y < h; y++)
                    inputColumn[y] = rowPass[y * width + x];

                Edt1D.Transform_Isotropic(inputColumn, outputColumn);

                for (int y = 0; y < h; y++)
                    result[y * width + x] = outputColumn[y];
            }
            ;
        }


        /// <summary>
        /// Computes the exact 2D squared Euclidean distance transform (EDT) for anisotropic pixels (non-unit spacing).
        /// </summary>
        /// <param name="seedCosts">Flattened row-major cost grid (y*width + x). 0 at seeds, INF at non-seeds.</param>
        /// <param name="width">Number of pixels along X.</param>
        /// <param name="height">Number of pixels along Y.</param>
        /// <param name="spacingX">Physical spacing along the X axis.</param>
        /// <param name="spacingY">Physical spacing along the Y axis.</param>
        /// <param name="parallel">If true, rows and columns are processed in parallel.</param>
        /// <returns>Flattened row-major array of squared distances to the nearest seed, scaled by anisotropic spacings.</returns>
        public static double[] ExactSquaredAnisotropic(
            double[] seedCosts,
            int width, int height,
            double spacingX, double spacingY,
            bool parallel)
        {
            double[] rowPass = new double[seedCosts.Length];
            double[] result = new double[seedCosts.Length];

            // First pass: transform along rows (x direction)
            // weight = spacingX^2
            double weightX = spacingX * spacingX;
            double weightY = spacingY * spacingY;

            if (parallel)
                Parallel.For(0, height, (Action<int>)ProcessRow);
            else
            {
                for (int y = 0; y < height; y++)
                    ProcessRow(y);
            }
            ;


            // Second pass: transform along columns (y direction)
            if (parallel)
                Parallel.For(0, width, (Action<int>)ProcessColumn);
            else
            {
                for (int x = 0; x < width; x++)
                    ProcessColumn(x);
            }
            return result;

            // =============================
            // Local functions
            // =============================
            void ProcessRow(int y)
            {
                int rowOffset = y * width;
                Span<double> inputRow = seedCosts.AsSpan(rowOffset, width);
                Span<double> outputRow = rowPass.AsSpan(rowOffset, width);
                Edt1D.Transform_Weighted(inputRow, outputRow, weightX);
            }

            void ProcessColumn(int x)
            {
                int h = height;
                Span<double> inputColumn = h <= 4096 ? stackalloc double[h] : new double[h];
                Span<double> outputColumn = h <= 4096 ? stackalloc double[h] : new double[h];

                for (int y = 0; y < height; y++)
                    inputColumn[y] = rowPass[y * width + x];

                Edt1D.Transform_Weighted(inputColumn, outputColumn, weightY);
                for (int y = 0; y < height; y++)
                    result[y * width + x] = outputColumn[y];
            }
        }
    }
}
