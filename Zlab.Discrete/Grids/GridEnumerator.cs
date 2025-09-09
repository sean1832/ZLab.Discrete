using System;
using System.Collections.Generic;
using System.Numerics;
using System.Threading.Tasks;
using ZLab.Discrete.Grids.Interfaces;

namespace ZLab.Discrete.Grids
{
    internal static class GridEnumerator
    {
        [Obsolete("Use ForEachVoxel or ForEachVoxelParallel instead for better performance.")]
        public static IEnumerable<(Vector3 position, T value)> EnumerateVoxels<T>(this GridBase g, IGrid<T> data)
        {
            GridMeta meta = g.Meta;
            for (int z = meta.MinZ; z < meta.MinZ + meta.Nz; z++)
            for (int y = meta.MinY; y < meta.MinY + meta.Ny; y++)
            for (int x = meta.MinX; x < meta.MinX + meta.Nx; x++)
            {
                yield return (GridConverter.IndexToMinCorner(x, y, z, meta.VoxelSize), data.GetValue((x, y, z)));
            }
        }

        /// <summary>
        /// Visit each voxel once (x-fastest) using the grid's flat buffer.
        /// </summary>
        public static void ForEachVoxel<T>(this GridBase g, IGrid<T> data, Action<Vector3, T> action)
        {
            // No allocations, no per-voxel interface calls.
            GridMeta meta = g.Meta;
            ReadOnlySpan<T> buffer = data.GetReadOnlyBuffer();

            int nx = meta.Nx;
            int ny = meta.Ny;
            int nz = meta.Nz;
            if (buffer.Length != g.Meta.Count)
                throw new ArgumentException("Data buffer size does not match grid dimensions.");

            Vector3 size = meta.VoxelSize;
            int lin = 0;

            // min corner of (MinX, MinY, MinZ)
            Vector3 sliceStart = GridConverter.IndexToMinCorner(meta.MinX, meta.MinY, meta.MinZ, size);

            for (int z = 0; z < nz; z++)
            {
                Vector3 rowStart = sliceStart;
                for (int y = 0; y < ny; y++)
                {
                    Vector3 pos = rowStart;
                    int rowLin = lin;

                    // x-fastest: contiguous in buf
                    for (int x = 0; x < nx; x++)
                    {
                        action(pos, buffer[rowLin + x]);
                        pos.X += size.X;
                    }

                    lin += nx;
                    rowStart.Y += size.Y;
                }

                sliceStart.Z += size.Z;
            }
        }

        /// <summary>
        /// Parallel z-slice enumeration. Use only if the work done in <paramref name="action"/>
        /// is heavy enough to justify the parallel overhead.
        /// </summary>
        public static void ForEachVoxelParallel<T>(this GridBase g, IGrid<T> data, Action<Vector3, T> action, int? maxDegree = null)
        {
            GridMeta meta = g.Meta;
            ReadOnlyMemory<T> memory = data.GetReadOnlyMemory();
            int nx = meta.Nx;
            int ny = meta.Ny;
            int nz = meta.Nz;
            
            if (memory.Length != g.Meta.Count)
                throw new ArgumentException("Data memory size does not match grid dimensions.");

            int layer = nx * ny;
            Vector3 size = meta.VoxelSize;
            Vector3 basePos = GridConverter.IndexToMinCorner(meta.MinX, meta.MinY, meta.MinZ, size);

            ParallelOptions option = new();
            if (maxDegree.HasValue)
                option.MaxDegreeOfParallelism = maxDegree.Value;
            Parallel.For(0, nz, option, z =>
            {
                // create the Span *inside* the worker
                ReadOnlySpan<T> buf = memory.Span;

                int lin = z * layer;
                Vector3 rowStart = new(basePos.X, basePos.Y, basePos.Z + z * size.Z);

                for (int y = 0; y < ny; y++)
                {
                    Vector3 pos = rowStart;
                    int rowLin = lin + y * nx;

                    for (int x = 0; x < nx; x++)
                    {
                        action(pos, buf[(rowLin + x)]);
                        pos.X += size.X;
                    }
                    rowStart.Y += size.Y;
                }
            });
        }
    }
}
