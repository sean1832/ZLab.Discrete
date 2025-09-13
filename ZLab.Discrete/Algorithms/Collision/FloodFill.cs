using System;
using System.Buffers;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using ZLab.Discrete.Grids;
using ZLab.Discrete.Voxels;

namespace ZLab.Discrete.Algorithms.Collision
{
    internal static class FloodFill
    {
        /// <summary>
        /// Flood-fill an occupancy grid in 3D. (classifies all non-boundary voxels as Inside or Outside)
        /// </summary>
        public static void Fill3D(OccupancyGrid grid)
        {
            GridMeta m = grid.Meta;
            int nx = m.Nx, ny = m.Ny, nz = m.Nz;
            int layer = nx * ny;
            int total = layer * nz;

            // Direct access to occupancy buffer (X-fastest layout: (z*ny + y)*nx + x)
            Span<Occupancy> buf = grid.GetBuffer();

            // visited bitset over linear indices [0..total)
            int wordCount = (total + 31) >> 5;
            uint[] visited = ArrayPool<uint>.Shared.Rent(wordCount);
            Array.Clear(visited, 0, wordCount);

            // Queue of linear indices
            int initialCapacity = Math.Clamp(nx * ny, 1 << 16, total);
            int[] qbuf = ArrayPool<int>.Shared.Rent(initialCapacity);
            RingQueue q = new(qbuf, initialCapacity, total);

            // -------- Seeding (LOCAL coords; no Contains/GetValue) --------
            // x = 0 and x = nx-1 faces
            for (int z = 0; z < nz; z++)
            {
                int zBase = z * layer;
                for (int y = 0; y < ny; y++)
                {
                    int rowBase = zBase + y * nx;

                    TrySeedLocal(rowBase + 0, buf, visited, ref q);           // x = 0
                    TrySeedLocal(rowBase + (nx - 1), buf, visited, ref q);    // x = nx-1
                }
            }
            // y = 0 and y = ny-1 faces
            for (int z = 0; z < nz; z++)
            {
                int zBase = z * layer;
                int y0Base = zBase + 0 * nx;
                int y1Base = zBase + (ny - 1) * nx;

                for (int x = 0; x < nx; x++)
                {
                    TrySeedLocal(y0Base + x, buf, visited, ref q); // y = 0
                    TrySeedLocal(y1Base + x, buf, visited, ref q); // y = ny-1
                }
            }
            // z = 0 and z = nz-1 faces
            int z0Base = 0 * layer;
            int z1Base = (nz - 1) * layer;
            for (int y = 0; y < ny; y++)
            {
                int yBase0 = z0Base + y * nx;
                int yBase1 = z1Base + y * nx;
                for (int x = 0; x < nx; x++)
                {
                    TrySeedLocal(yBase0 + x, buf, visited, ref q); // z = 0
                    TrySeedLocal(yBase1 + x, buf, visited, ref q); // z = nz-1
                }
            }

            // -------- BFS in local/linear space --------
            while (!q.IsEmpty)
            {
                int lin = q.Dequeue();

                // Decode lin -> (lz, ly, lx) with 1 div & 1 DivRem
                int lz = lin / layer;
                int rem = lin - lz * layer;
                int ly = Math.DivRem(rem, nx, out int lx);

                // 6-neighbors with simple bounds checks, then linear math
                // +X
                if (lx + 1 < nx)
                {
                    int nlin = lin + 1;
                    if (buf[nlin] != Occupancy.Boundary && TestAndSetVisited(visited, nlin))
                        q.Enqueue(nlin);
                }
                // -X
                if (lx - 1 >= 0)
                {
                    int nlin = lin - 1;
                    if (buf[nlin] != Occupancy.Boundary && TestAndSetVisited(visited, nlin))
                        q.Enqueue(nlin);
                }
                // +Y
                if (ly + 1 < ny)
                {
                    int nlin = lin + nx;
                    if (buf[nlin] != Occupancy.Boundary && TestAndSetVisited(visited, nlin))
                        q.Enqueue(nlin);
                }
                // -Y
                if (ly - 1 >= 0)
                {
                    int nlin = lin - nx;
                    if (buf[nlin] != Occupancy.Boundary && TestAndSetVisited(visited, nlin))
                        q.Enqueue(nlin);
                }
                // +Z
                if (lz + 1 < nz)
                {
                    int nlin = lin + layer;
                    if (buf[nlin] != Occupancy.Boundary && TestAndSetVisited(visited, nlin))
                        q.Enqueue(nlin);
                }
                // -Z
                if (lz - 1 >= 0)
                {
                    int nlin = lin - layer;
                    if (buf[nlin] != Occupancy.Boundary && TestAndSetVisited(visited, nlin))
                        q.Enqueue(nlin);
                }
            }

            // -------- Final labeling: one linear pass --------
            for (int i = 0; i < total; i++)
            {
                if (buf[i] == Occupancy.Boundary) continue;
                buf[i] = TestVisited(visited, i) ? Occupancy.Outside : Occupancy.Inside;
            }

            // Return pools
            q.ReturnBufferToPool();
            ArrayPool<uint>.Shared.Return(visited, clearArray: false);
        }



        // =============== Helpers ===============

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void TrySeedLocal(int lin, Span<Occupancy> buf, uint[] visited, ref RingQueue q)
        {
            if (buf[lin] == Occupancy.Boundary) return;
            if (TestAndSetVisited(visited, lin))
                q.Enqueue(lin);
        }

        // Helper to test visited without modifying
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool TestVisited(uint[] words, int index)
        {
            int w = index >> 5;
            uint mask = 1u << (index & 31);
            return (words[w] & mask) != 0;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool TestAndSetVisited(uint[] words, int index)
        {
            int w = index >> 5;
            uint mask = 1u << (index & 31);
            uint old = words[w];
            if ((old & mask) != 0) return false;
            words[w] = old | mask;
            return true;
        }

        // =============== Resizable ring queue ===============
        private sealed class RingQueue
        {
            private readonly int _hardLimit;
            private int[] _buf;
            private int _cap;
            private int _head;
            private int _tail;
            private int _count;

            public RingQueue(int[] initial, int initialCap, int hardLimit)
            {
                _buf = initial ?? throw new ArgumentNullException(nameof(initial));
                _cap = initialCap;
                _hardLimit = Math.Max(1, hardLimit);
                _head = 0;
                _tail = 0;
                _count = 0;
            }

            public bool IsEmpty => _count == 0;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Enqueue(int v)
            {
                if (_count == _cap) Grow();
                _buf[_tail] = v;
                _tail++;
                if (_tail == _cap) _tail = 0;
                _count++;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public int Dequeue()
            {
                if (_count == 0) throw new InvalidOperationException("Queue empty.");
                int v = _buf[_head];
                _head++;
                if (_head == _cap) _head = 0;
                _count--;
                return v;
            }

            private void Grow()
            {
                if (_cap == _hardLimit)
                    throw new InvalidOperationException($"Queue reached hard limit ({_hardLimit}).");

                int newCap = _cap <= (_hardLimit >> 1) ? _cap << 1 : _hardLimit;
                int[] nb = ArrayPool<int>.Shared.Rent(newCap);

                if (_count > 0)
                {
                    if (_head < _tail)
                    {
                        Array.Copy(_buf, _head, nb, 0, _count);
                    }
                    else
                    {
                        int right = _cap - _head;
                        Array.Copy(_buf, _head, nb, 0, right);
                        Array.Copy(_buf, 0, nb, right, _tail);
                    }
                }

                ArrayPool<int>.Shared.Return(_buf, false);
                _buf = nb;
                _cap = newCap;
                _head = 0;
                _tail = _count;
            }

            public void ReturnBufferToPool()
            {
                ArrayPool<int>.Shared.Return(_buf, false);
                _buf = Array.Empty<int>();
                _cap = _head = _tail = _count = 0;
            }
        }

        // ====================================
        // OLD SLOW METHOD (for reference)
        // ===================================

        [Obsolete("Use Fill3D instead. This method is very slow and unoptimized.")]
        public static void Fill3D_Old(OccupancyGrid grid)
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
