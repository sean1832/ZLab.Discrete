using System.Collections.Generic;
using System.Numerics;
using ZLab.Discrete.Grids.Interfaces;

namespace ZLab.Discrete.Grids
{
    internal static class GridEnumerator
    {
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
    }
}
