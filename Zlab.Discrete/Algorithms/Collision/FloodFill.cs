using System;
using System.Collections.Generic;
using ZLab.Discrete.Grids;
using ZLab.Discrete.Voxels;

namespace ZLab.Discrete.Algorithms.Collision
{
    internal static class FloodFill
    {
        public static void Fill3D(OccupancyGrid grid)
        {
            int minX = grid.Meta.MinX;
            int minY = grid.Meta.MinY;
            int minZ = grid.Meta.MinZ;

            int nx = grid.Meta.Nx;
            int ny = grid.Meta.Ny;
            int nz = grid.Meta.Nz;

            int maxX = minX + nx - 1;
            int maxY = minY + ny - 1;
            int maxZ = minZ + nz - 1;

            bool[] visited = new bool[nx * ny * nz];

            ReadOnlySpan<(int dx, int dy, int dz)> neighbours = stackalloc (int, int, int)[]
            {
                (1, 0, 0), (-1, 0, 0),
                (0, 1, 0), (0, -1, 0),
                (0, 0, 1), (0, 0, -1),
            };

            Queue<(int x, int y, int z)> queue = new(1 << 12);

            // Seed all 6 faces
            for (int y = minY; y <= maxY; y++)
                for (int z = minZ; z <= maxZ; z++)
                {
                    TrySeed(minX, y, z);
                    TrySeed(maxX, y, z);
                }

            for (int x = minX; x <= maxX; x++)
                for (int z = minZ; z <= maxZ; z++)
                {
                    TrySeed(x, minY, z);
                    TrySeed(x, maxY, z);
                }

            for (int x = minX; x <= maxX; x++)
                for (int y = minY; y <= maxY; y++)
                {
                    TrySeed(x, y, minZ);
                    TrySeed(x, y, maxZ);
                }

            // BFS through non-Intersecting voxels
            while (queue.Count > 0)
            {
                (int x, int y, int z) = queue.Dequeue();

                if (grid.GetValue((x, y, z)) != Occupancy.Outside)
                    grid.SetValue((x, y, z), Occupancy.Outside);

                foreach ((int dx, int dy, int dz) in neighbours)
                {
                    int xn = x + dx, yn = y + dy, zn = z + dz;
                    (int xn, int yn, int zn) nIdx = (xn, yn, zn);
                    if (!grid.Contains(nIdx)) continue;
                    if (grid.GetValue(nIdx) == Occupancy.Boundary) continue;

                    int lin = ToIndex(xn, yn, zn); // guaranteed in-range now
                    if (visited[lin]) continue;

                    visited[lin] = true;
                    queue.Enqueue(nIdx);
                }
            }

            // Label interior vs exterior
            for (int z = minZ; z <= maxZ; z++)
                for (int y = minY; y <= maxY; y++)
                    for (int x = minX; x <= maxX; x++)
                    {
                        (int x, int y, int z) idx = (x, y, z);
                        if (grid.GetValue(idx) == Occupancy.Boundary) continue;

                        int lin = ToIndex(x, y, z);
                        grid.SetValue(idx, visited[lin] ? Occupancy.Outside : Occupancy.Inside);
                    }

            return;

            // local helpers
            int ToIndex(int x, int y, int z) => (z - minZ) * nx * ny + (y - minY) * nx + (x - minX);

            void TrySeed(int x, int y, int z)
            {
                (int x, int y, int z) idx = (x, y, z);
                if (!grid.Contains(idx)) return;
                if (grid.GetValue(idx) == Occupancy.Boundary) return;

                int lin = ToIndex(x, y, z);
                if (visited[lin]) return;

                visited[lin] = true;
                queue.Enqueue(idx);
            }
        }
    }
}
