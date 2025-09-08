using System;

namespace ZLab.Discrete.Algorithms.DistanceTransforms
{

    /// <summary>
    /// 1D Exact Euclidean Distance Transform (EDT).
    /// </summary>
    /// <remarks>
    /// Reference:
    /// <a href="https://cs.brown.edu/people/pfelzens/papers/dt-final.pdf">
    /// P.F.Felzenszwalb, D.P.Huttenlocher (2012). Distance Transforms of Sampled Functions.
    /// </a>
    /// </remarks>
    internal static class Edt1D
    {
        /// <summary>
        /// Computes the 1D exact squared Euclidean distance transform (isotropic case, unit spacing).
        /// </summary>
        /// <remarks>
        /// Formula: <code>output[i] = min_j ((i - j)^2 + input[j])</code> 
        /// Runs in O(n) time using the lower envelope of parabolas method (Felzenszwalb–Huttenlocher).
        /// </remarks>
        /// <param name="input">
        /// Input array (length n). 0 at seed positions, large value (e.g. INF) elsewhere.
        /// </param>
        /// <param name="output">
        /// Output array (length n). Each entry is the squared distance to the nearest seed.
        /// </param>
        public static void Transform_Isotropic(ReadOnlySpan<int> input, Span<int> output)
        {
            int length = input.Length;
            if (length == 0) return;
            // use stackalloc for small arrays, heap for larger ones
            Span<int> candidatePositions = length <= 4096 ? stackalloc int[length] : new int[length];
            Span<double> regionBounds = (length + 1) <= 4096 ? stackalloc double[length + 1] : new double[length + 1];

            int k = 0; // number of candidate positions
            candidatePositions[0] = 0;
            regionBounds[0] = double.NegativeInfinity;
            regionBounds[1] = double.PositiveInfinity;

            // Build lower envelope of parabolas
            for (int position = 1; position < length; position++)
            {
                double intersection;
                while (true)
                {
                    int p = candidatePositions[k];

                    // Intersection where new parabola at 'position' beats the previous at 'previous'
                    // s = ((f[q] + q^2) - (f[p] + p^2)) / (2*(q-p))
                    long fQ = input[position], fP = input[p];
                    long qq = position, pp = p;
                    long num = (fQ + qq * qq) - (fP + pp * pp);
                    long den = 2L * (qq - pp);
                    intersection = num / (double)den;

                    if (intersection <= regionBounds[k])
                    {
                        k--;
                        if (k < 0) { k = 0; break; }
                    }
                    else break;
                }
                k++;
                candidatePositions[k] = position;
                regionBounds[k] = intersection;
                regionBounds[k + 1] = double.PositiveInfinity;
            }

            int activeCandidateIdx = 0;
            for (int x = 0; x < length; x++)
            {
                while (regionBounds[activeCandidateIdx + 1] < x) activeCandidateIdx++;
                int s = candidatePositions[activeCandidateIdx];
                int dx = x - s;
                output[x] = dx * dx + input[s];
            }
        }

        /// <summary>
        /// Computes the 1D exact squared Euclidean distance transform with axis weight (anisotropic spacing).
        /// </summary>
        /// <remarks>
        /// Formula: <code>output[i] = min_j (weight * (i - j)^2 + input[j])</code> 
        /// Runs in O(n) time using the lower envelope of parabolas method (Felzenszwalb–Huttenlocher).
        /// </remarks>
        /// <param name="input">Input array (length n). 0 at seed positions, large value (e.g. INF) elsewhere.</param>
        /// <param name="output">Output array (length n). Each entry is the squared distance to the nearest seed.</param>
        /// <param name="weight">Squared voxel spacing on this axis (weight = spacing²).</param>
        public static void Transform_Weighted(ReadOnlySpan<double> input, Span<double> output, double weight)
        {
            int length = input.Length;
            if (length == 0) return;
            if (weight <= 0) throw new ArgumentOutOfRangeException(nameof(weight), "weight must be > 0");

            Span<int> candidatePositions = length <= 4096 ? stackalloc int[length] : new int[length];
            Span<double> regionBounds = (length + 1) <= 4096 ? stackalloc double[length + 1] : new double[length + 1];

            int k = 0;
            candidatePositions[0] = 0;
            regionBounds[0] = double.NegativeInfinity;
            regionBounds[1] = double.PositiveInfinity;

            for (int position = 1; position < length; position++)
            {
                double intersection;
                while (true)
                {
                    int p = candidatePositions[k];

                    // s = ((f[q] + w*q^2) - (f[p] + w*p^2)) / (2*w*(q-p))
                    double f_q = input[position], f_p = input[p];
                    double qq = position, pp = p;
                    double num = (f_q + weight * qq * qq) - (f_p + weight * pp * pp);
                    double den = 2.0 * weight * (qq - pp);
                    intersection = num / den;

                    if (intersection <= regionBounds[k])
                    {
                        k--;
                        if (k >= 0) continue;
                        k = 0;
                    }

                    break;
                }
                k++;
                candidatePositions[k] = position;
                regionBounds[k] = intersection;
                regionBounds[k + 1] = double.PositiveInfinity;
            }

            int activeCandidateIdx = 0;
            for (int x = 0; x < length; x++)
            {
                while (regionBounds[activeCandidateIdx + 1] < x) activeCandidateIdx++;
                int s = candidatePositions[activeCandidateIdx];
                double dx = x - s;
                output[x] = weight * dx * dx + input[s];
            }
        }
    }
}
