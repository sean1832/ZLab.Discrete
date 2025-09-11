using System;
using System.Buffers;
using System.Threading.Tasks;

namespace ZLab.Discrete.Algorithms.DistanceTransforms
{
    /// <summary>
    /// 3D Exact Euclidean Distance Transform (EDT). Builds on top of <see cref="Edt1D"/>.
    /// </summary>
    /// <remarks>
    /// Reference:
    /// <a href="https://cs.brown.edu/people/pfelzens/papers/dt-final.pdf">
    /// P.F.Felzenszwalb, D.P.Huttenlocher (2012). Distance Transforms of Sampled Functions.
    /// </a>
    /// </remarks>
    internal static class Edt3D
    {
        /// <summary>
        /// Computes the exact 3D squared Euclidean distance transform (EDT) for isotropic voxels (unit spacing).
        /// </summary>
        /// <param name="seedCosts">Flattened row-major cost volume (z*nx*ny + y*nx + x). 0 at seed voxels, INF at non-seeds.</param>
        /// <param name="sqDistances">FINAL OUTPUT: Preallocated output array (length nx*ny*nz). Each entry is the squared distance to the nearest seed.</param>
        /// <param name="nx">Number of voxels along X.</param>
        /// <param name="ny">Number of voxels along Y.</param>
        /// <param name="nz">Number of voxels along Z.</param>
        /// <param name="parallel">If true, lines along X, Y, and Z are processed in parallel.</param>
        /// <returns>Flattened row-major array of squared distances to the nearest seed.</returns>
        public static void ExactSquaredIsotropic(
            ReadOnlySpan<int> seedCosts, 
            Span<int> sqDistances,
            int nx, int ny, int nz, bool parallel)
        {
            int total = nx * ny * nz;
            if (seedCosts.Length != total)
                throw new ArgumentException("seedCosts length must equal nx*ny*nz.");
            if (sqDistances.Length != total)
                throw new ArgumentException("squaredDistances length must equal nx*ny*nz.");

            // Rent two temporary buffers of total size and ping-pong them
            int[] bufferA = ArrayPool<int>.Shared.Rent(total);
            int[] bufferB = ArrayPool<int>.Shared.Rent(total);

            try
            {
                int[] currentArray = bufferA;
                int[] nextArray = bufferB;

                // ----------------------------
                // X pass
                // ----------------------------
                if (parallel)
                {
                    Parallel.For(0, ny * nz, yz =>
                    {
                        int y = yz % ny;
                        int z = yz / ny;
                        int offset = z * nx * ny + y * nx;

                        ReadOnlySpan<int> lineIn = currentArray.AsSpan(offset, nx);
                        Span<int> lineOut = nextArray.AsSpan(offset, nx);

                        Edt1D.Transform_Isotropic(lineIn, lineOut);
                    });
                }
                else
                {
                    for (int yz = 0; yz < ny * nz; yz++)
                    {
                        int y = yz % ny;
                        int z = yz / ny;
                        int offset = z * nx * ny + y * nx;

                        Edt1D.Transform_Isotropic(
                            currentArray.AsSpan(offset, nx),
                            nextArray.AsSpan(offset, nx));
                    }
                }

                // swap
                (int[] tempA, int[] tempB) = (currentArray, nextArray);
                currentArray = tempB;
                nextArray = tempA;

                // ----------------------------
                // Y pass
                // ----------------------------
                if (parallel)
                {
                    Parallel.For(0, nx * nz,
                        () => (
                            ArrayPool<int>.Shared.Rent(ny),
                            ArrayPool<int>.Shared.Rent(ny)
                        ),
                        (xz, _, state) =>
                        {
                            (int[]? lineIn, int[]? lineOut) = state;
                            int x = xz % nx;
                            int z = xz / nx;

                            for (int y = 0; y < ny; y++)
                                lineIn[y] = currentArray[z * nx * ny + y * nx + x];

                            Edt1D.Transform_Isotropic(
                                lineIn.AsSpan(0, ny),
                                lineOut.AsSpan(0, ny));

                            for (int y = 0; y < ny; y++)
                                nextArray[z * nx * ny + y * nx + x] = lineOut[y];

                            return state;
                        },
                        state =>
                        {
                            ArrayPool<int>.Shared.Return(state.Item1);
                            ArrayPool<int>.Shared.Return(state.Item2);
                        });
                }
                else
                {
                    int[] lineIn = ArrayPool<int>.Shared.Rent(ny);
                    int[] lineOut = ArrayPool<int>.Shared.Rent(ny);
                    try
                    {
                        for (int xz = 0; xz < nx * nz; xz++)
                        {
                            int x = xz % nx;
                            int z = xz / nx;

                            for (int y = 0; y < ny; y++)
                                lineIn[y] = currentArray[z * nx * ny + y * nx + x];

                            Edt1D.Transform_Isotropic(
                                lineIn.AsSpan(0, ny),
                                lineOut.AsSpan(0, ny));

                            for (int y = 0; y < ny; y++)
                                nextArray[z * nx * ny + y * nx + x] = lineOut[y];
                        }
                    }
                    finally
                    {
                        ArrayPool<int>.Shared.Return(lineIn);
                        ArrayPool<int>.Shared.Return(lineOut);
                    }
                }

                // swap
                (int[] tempC, int[] tempD) = (currentArray, nextArray);
                currentArray = tempD;
                nextArray = tempC;

                // ----------------------------
                // Z pass (final writes into squaredDistances)
                // ----------------------------
                if (parallel)
                {
                    Parallel.For(0, nx * ny,
                        () => (
                            ArrayPool<int>.Shared.Rent(nz),
                            ArrayPool<int>.Shared.Rent(nz)
                        ),
                        (xy, _, state) =>
                        {
                            (int[]? lineIn, int[]? lineOut) = state;
                            int x = xy % nx;
                            int y = xy / nx;

                            for (int z = 0; z < nz; z++)
                                lineIn[z] = currentArray[z * nx * ny + y * nx + x];

                            Edt1D.Transform_Isotropic(
                                lineIn.AsSpan(0, nz),
                                lineOut.AsSpan(0, nz));

                            for (int z = 0; z < nz; z++)
                                nextArray[z * nx * ny + y * nx + x] = lineOut[z];

                            return state;
                        },
                        state =>
                        {
                            ArrayPool<int>.Shared.Return(state.Item1);
                            ArrayPool<int>.Shared.Return(state.Item2);
                        });

                    // copy final result
                    nextArray.AsSpan(0, total).CopyTo(sqDistances);
                }
                else
                {
                    int[] lineIn = ArrayPool<int>.Shared.Rent(nz);
                    int[] lineOut = ArrayPool<int>.Shared.Rent(nz);
                    try
                    {
                        for (int xy = 0; xy < nx * ny; xy++)
                        {
                            int x = xy % nx;
                            int y = xy / nx;

                            for (int z = 0; z < nz; z++)
                                lineIn[z] = currentArray[z * nx * ny + y * nx + x];

                            Edt1D.Transform_Isotropic(
                                lineIn.AsSpan(0, nz),
                                lineOut.AsSpan(0, nz));

                            for (int z = 0; z < nz; z++)
                                sqDistances[z * nx * ny + y * nx + x] = lineOut[z];
                        }
                    }
                    finally
                    {
                        ArrayPool<int>.Shared.Return(lineIn);
                        ArrayPool<int>.Shared.Return(lineOut);
                    }
                }
            }
            finally
            {
                ArrayPool<int>.Shared.Return(bufferA);
                ArrayPool<int>.Shared.Return(bufferB);
            }
        }

        /// <summary>
        /// Computes the exact 3D squared Euclidean distance transform (EDT) for anisotropic voxels (non-unit spacing).
        /// </summary>
        /// <param name="seedCosts">Flattened row-major cost volume (z*nx*ny + y*nx + x). 0 at seed voxels, INF at non-seeds.</param>
        /// <param name="sqDistances">FINAL OUTPUT: Preallocated output array (length nx*ny*nz). Each entry is the squared distance to the nearest seed, scaled by anisotropic spacings.</param>
        /// <param name="nx">Number of voxels along X.</param>
        /// <param name="ny">Number of voxels along Y.</param>
        /// <param name="nz">Number of voxels along Z.</param>
        /// <param name="spacingX">Physical voxel spacing along the X axis.</param>
        /// <param name="spacingY">Physical voxel spacing along the Y axis.</param>
        /// <param name="spacingZ">Physical voxel spacing along the Z axis.</param>
        /// <param name="parallel">If true, lines along X, Y, and Z are processed in parallel.</param>
        public static void ExactSquaredAnisotropic(
            ReadOnlySpan<double> seedCosts, Span<double> sqDistances,
            int nx, int ny, int nz,
            double spacingX, double spacingY, double spacingZ,
            bool parallel)
        {
            int total = nx * ny * nz;
            if (seedCosts.Length != total)
                throw new ArgumentException("seedCosts length must equal nx*ny*nz.");
            if (sqDistances.Length != total)
                throw new ArgumentException("squaredDistances length must equal nx*ny*nz.");

            // Rent two temporary buffers of total size and ping-pong them
            double[] bufferA = ArrayPool<double>.Shared.Rent(total);
            double[] bufferB = ArrayPool<double>.Shared.Rent(total);

            try
            {
                double[] currentArray = bufferA;
                double[] nextArray = bufferB;

                seedCosts.CopyTo(currentArray);

                double weightX = spacingX * spacingX;
                double weightY = spacingY * spacingY;
                double weightZ = spacingZ * spacingZ;

                // ----------------------------
                // X pass
                // ----------------------------
                if (parallel)
                {
                    Parallel.For(0, ny * nz, yz =>
                    {
                        int y = yz % ny;
                        int z = yz / ny;
                        int offset = z * nx * ny + y * nx;

                        ReadOnlySpan<double> lineIn = currentArray.AsSpan(offset, nx);
                        Span<double> lineOut = nextArray.AsSpan(offset, nx);

                        Edt1D.Transform_Weighted(lineIn, lineOut, weightX);
                    });
                }
                else
                {
                    for (int yz = 0; yz < ny * nz; yz++)
                    {
                        int y = yz % ny;
                        int z = yz / ny;
                        int offset = z * nx * ny + y * nx;

                        Edt1D.Transform_Weighted(
                            currentArray.AsSpan(offset, nx),
                            nextArray.AsSpan(offset, nx),
                            weightX);
                    }
                }

                // swap
                (double[] tempA, double[] tempB) = (currentArray, nextArray);
                currentArray = tempB;
                nextArray = tempA;

                // ----------------------------
                // Y pass
                // ----------------------------
                if (parallel)
                {
                    Parallel.For(0, nx * nz,
                        () => (
                            ArrayPool<double>.Shared.Rent(ny),
                            ArrayPool<double>.Shared.Rent(ny)
                        ),
                        (xz, _, state) =>
                        {
                            (double[]? lineIn, double[]? lineOut) = state;
                            int x = xz % nx;
                            int z = xz / nx;

                            for (int y = 0; y < ny; y++)
                                lineIn[y] = currentArray[z * nx * ny + y * nx + x];

                            Edt1D.Transform_Weighted(
                                lineIn.AsSpan(0, ny),
                                lineOut.AsSpan(0, ny),
                                weightY);

                            for (int y = 0; y < ny; y++)
                                nextArray[z * nx * ny + y * nx + x] = lineOut[y];

                            return state;
                        },
                        state =>
                        {
                            ArrayPool<double>.Shared.Return(state.Item1);
                            ArrayPool<double>.Shared.Return(state.Item2);
                        });
                }
                else
                {
                    double[] lineIn = ArrayPool<double>.Shared.Rent(ny);
                    double[] lineOut = ArrayPool<double>.Shared.Rent(ny);
                    try
                    {
                        for (int xz = 0; xz < nx * nz; xz++)
                        {
                            int x = xz % nx;
                            int z = xz / nx;

                            for (int y = 0; y < ny; y++)
                                lineIn[y] = currentArray[z * nx * ny + y * nx + x];

                            Edt1D.Transform_Weighted(
                                lineIn.AsSpan(0, ny),
                                lineOut.AsSpan(0, ny),
                                weightY);

                            for (int y = 0; y < ny; y++)
                                nextArray[z * nx * ny + y * nx + x] = lineOut[y];
                        }
                    }
                    finally
                    {
                        ArrayPool<double>.Shared.Return(lineIn);
                        ArrayPool<double>.Shared.Return(lineOut);
                    }
                }

                // swap
                (double[] tempC, double[] tempD) = (currentArray, nextArray);
                currentArray = tempD;
                nextArray = tempC;

                // ----------------------------
                // Z pass (final writes into squaredDistances)
                // ----------------------------
                if (parallel)
                {
                    Parallel.For(0, nx * ny,
                        () => (
                            ArrayPool<double>.Shared.Rent(nz),
                            ArrayPool<double>.Shared.Rent(nz)
                        ),
                        (xy, _, state) =>
                        {
                            (double[]? lineIn, double[]? lineOut) = state;
                            int x = xy % nx;
                            int y = xy / nx;

                            for (int z = 0; z < nz; z++)
                                lineIn[z] = currentArray[z * nx * ny + y * nx + x];

                            Edt1D.Transform_Weighted(
                                lineIn.AsSpan(0, nz),
                                lineOut.AsSpan(0, nz),
                                weightZ);

                            for (int z = 0; z < nz; z++)
                                nextArray[z * nx * ny + y * nx + x] = lineOut[z];

                            return state;
                        },
                        state =>
                        {
                            ArrayPool<double>.Shared.Return(state.Item1);
                            ArrayPool<double>.Shared.Return(state.Item2);
                        });

                    // copy final result
                    nextArray.AsSpan(0, total).CopyTo(sqDistances);
                }
                else
                {
                    double[] lineIn = ArrayPool<double>.Shared.Rent(nz);
                    double[] lineOut = ArrayPool<double>.Shared.Rent(nz);
                    try
                    {
                        for (int xy = 0; xy < nx * ny; xy++)
                        {
                            int x = xy % nx;
                            int y = xy / nx;

                            for (int z = 0; z < nz; z++)
                                lineIn[z] = currentArray[z * nx * ny + y * nx + x];

                            Edt1D.Transform_Weighted(
                                lineIn.AsSpan(0, nz),
                                lineOut.AsSpan(0, nz),
                                weightZ);

                            for (int z = 0; z < nz; z++)
                                sqDistances[z * nx * ny + y * nx + x] = lineOut[z];
                        }
                    }
                    finally
                    {
                        ArrayPool<double>.Shared.Return(lineIn);
                        ArrayPool<double>.Shared.Return(lineOut);
                    }
                }
            }
            finally
            {
                ArrayPool<double>.Shared.Return(bufferA);
                ArrayPool<double>.Shared.Return(bufferB);
            }
        }

        /// <summary>
        /// Computes the exact 3D squared Euclidean distance transform (EDT) for isotropic voxels (unit spacing).
        /// </summary>
        /// <param name="seedCosts">Flattened row-major cost volume (z*nx*ny + y*nx + x). 0 at seed voxels, INF at non-seeds.</param>
        /// <param name="nx">Number of voxels along X.</param>
        /// <param name="ny">Number of voxels along Y.</param>
        /// <param name="nz">Number of voxels along Z.</param>
        /// <param name="parallel">If true, lines along X, Y, and Z are processed in parallel.</param>
        /// <remarks><b>Obsolete</b>: use the overload that takes <see cref="ReadOnlySpan{T}"/> and <see cref="Span{T}"/> to avoid allocations.</remarks>
        /// <returns>Flattened row-major array of squared distances to the nearest seed.</returns>
        [Obsolete("Use the overload that takes ReadOnlySpan<int> and Span<int> to avoid allocations.")]
        public static int[] ExactSquaredIsotropic(int[] seedCosts, int nx, int ny, int nz, bool parallel)
        {
            // Three-pass 3D EDT (X, Y, Z) using 1D EDT in each pass
            int[] afterX = new int[seedCosts.Length];
            int[] afterY = new int[seedCosts.Length];
            int[] afterZ = new int[seedCosts.Length];

            // Pass 1: X direction
            int linesYz = ny * nz;
            if (parallel)
                Parallel.For(0, linesYz, (Action<int>)ProcessXLine);
            else
                for (int yz = 0; yz < linesYz; yz++) ProcessXLine(yz);

            // Pass 2: Y direction
            int linesXz = nx * nz;
            if (parallel)
                Parallel.For(0, linesXz, (Action<int>)ProcessYLine);
            else
                for (int xz = 0; xz < linesXz; xz++) ProcessYLine(xz);

            // Pass 3: Z direction
            int linesXy = nx * ny;
            if (parallel)
                Parallel.For(0, linesXy, (Action<int>)ProcessZLine);
            else
                for (int xy = 0; xy < linesXy; xy++) ProcessZLine(xy);

            return afterZ;

            // =============================
            // Local functions
            // =============================
            void ProcessXLine(int yz)
            {
                int y = yz % ny;
                int z = yz / ny;

                int offset = z * nx * ny + y * nx;

                ReadOnlySpan<int> lineIn = seedCosts.AsSpan(offset, nx);
                Span<int> lineOut = afterX.AsSpan(offset, nx);

                Edt1D.Transform_Isotropic(lineIn, lineOut);
            }

            void ProcessYLine(int xz)
            {
                int x = xz % nx;
                int z = xz / nx;
                int lineLength = ny;
                Span<int> lineIn = lineLength <= 4096 ? stackalloc int[lineLength] : new int[lineLength];
                Span<int> lineOut = lineLength <= 4096 ? stackalloc int[lineLength] : new int[lineLength];
                for (int y = 0; y < ny; y++)
                    lineIn[y] = afterX[z * nx * ny + y * nx + x];
                Edt1D.Transform_Isotropic(lineIn, lineOut);
                for (int y = 0; y < ny; y++)
                    afterY[z * nx * ny + y * nx + x] = lineOut[y];
            }

            void ProcessZLine(int xy)
            {
                int x = xy % nx;
                int y = xy / nx;

                int lineLength = nz;
                Span<int> lineIn = lineLength <= 4096 ? stackalloc int[lineLength] : new int[lineLength];
                Span<int> lineOut = lineLength <= 4096 ? stackalloc int[lineLength] : new int[lineLength];

                for (int z = 0; z < nz; z++)
                    lineIn[z] = afterY[z * nx * ny + y * nx + x];
                Edt1D.Transform_Isotropic(lineIn, lineOut);
                for (int z = 0; z < nz; z++)
                    afterZ[z * nx * ny + y * nx + x] = lineOut[z];
            }
        }

        /// <summary>
        /// Computes the exact 3D squared Euclidean distance transform (EDT) for anisotropic voxels (non-unit spacing).
        /// </summary>
        /// <param name="seedCosts">Flattened row-major cost volume (z*nx*ny + y*nx + x). 0 at seed voxels, INF at non-seeds.</param>
        /// <param name="nx">Number of voxels along X.</param>
        /// <param name="ny">Number of voxels along Y.</param>
        /// <param name="nz">Number of voxels along Z.</param>
        /// <param name="spacingX">Physical voxel spacing along the X axis.</param>
        /// <param name="spacingY">Physical voxel spacing along the Y axis.</param>
        /// <param name="spacingZ">Physical voxel spacing along the Z axis.</param>
        /// <param name="parallel">If true, lines along X, Y, and Z are processed in parallel.</param>
        /// <remarks><b>Obsolete</b>: use the overload that takes <see cref="ReadOnlySpan{T}"/> and <see cref="Span{T}"/> to avoid allocations.</remarks>
        /// <returns>Flattened row-major array of squared distances to the nearest seed, scaled by anisotropic spacings.</returns>
        [Obsolete("Use the overload that takes ReadOnlySpan<double> and Span<double> to avoid allocations.")]
        public static double[] ExactSquaredAnisotropic(
            double[] seedCosts, int nx, int ny, int nz,
            double spacingX, double spacingY, double spacingZ,
            bool parallel)
        {
            double[] afterX = new double[seedCosts.Length];
            double[] afterY = new double[seedCosts.Length];
            double[] afterZ = new double[seedCosts.Length];

            double weightX = spacingX * spacingX;
            double weightY = spacingY * spacingY;
            double weightZ = spacingZ * spacingZ;

            if (parallel)
                Parallel.For(0, ny * nz, (Action<int>)ProcessXLine);
            else
                for (int yz = 0; yz < ny * nz; yz++) ProcessXLine(yz);
            if (parallel)
                Parallel.For(0, nx * nz, (Action<int>)ProcessYLine);
            else
                for (int xz = 0; xz < nx * nz; xz++) ProcessYLine(xz);
            if (parallel)
                Parallel.For(0, nx * ny, (Action<int>)ProcessZLine);
            else
                for (int xy = 0; xy < nx * ny; xy++) ProcessZLine(xy);
            return afterZ;

            // ==========================
            // Local functions
            // ==========================
            void ProcessXLine(int yz)
            {
                int y = yz % ny;
                int z = yz / ny;
                int offset = z * nx * ny + y * nx;
                ReadOnlySpan<double> lineIn = seedCosts.AsSpan(offset, nx);
                Span<double> lineOut = afterX.AsSpan(offset, nx);
                Edt1D.Transform_Weighted(lineIn, lineOut, weightX);
            }

            void ProcessYLine(int xz)
            {
                int x = xz % nx;
                int z = xz / nx;
                int lineLength = ny;
                Span<double> lineIn = lineLength <= 4096 ? stackalloc double[lineLength] : new double[lineLength];
                Span<double> lineOut = lineLength <= 4096 ? stackalloc double[lineLength] : new double[lineLength];
                for (int y = 0; y < ny; y++)
                    lineIn[y] = afterX[z * nx * ny + y * nx + x];
                Edt1D.Transform_Weighted(lineIn, lineOut, weightY);
                for (int y = 0; y < ny; y++)
                    afterY[z * nx * ny + y * nx + x] = lineOut[y];
            }

            void ProcessZLine(int xy)
            {
                int x = xy % nx;
                int y = xy / nx;
                int lineLength = nz;
                Span<double> lineIn = lineLength <= 4096 ? stackalloc double[lineLength] : new double[lineLength];
                Span<double> lineOut = lineLength <= 4096 ? stackalloc double[lineLength] : new double[lineLength];
                for (int z = 0; z < nz; z++)
                    lineIn[z] = afterY[z * nx * ny + y * nx + x];
                Edt1D.Transform_Weighted(lineIn, lineOut, weightZ);
                for (int z = 0; z < nz; z++)
                    afterZ[z * nx * ny + y * nx + x] = lineOut[z];
            }
        }
    }
}
