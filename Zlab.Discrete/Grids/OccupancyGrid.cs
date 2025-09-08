using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using ZLab.Discrete.Geometry;
using ZLab.Discrete.Grids.Interfaces;
using ZLab.Discrete.Voxels;

namespace ZLab.Discrete.Grids
{
    /// <summary>
    /// Occupancy grid structure.
    /// </summary>
    public sealed class OccupancyGrid: GridBase, IGrid<Occupancy>
    {
        private Occupancy[] _occupancies; // length = Nx * Ny * Nz

        // ------------ Constructors ------------
        #region Constructors
        public OccupancyGrid(Vector3 voxelSize, BBox bounds)
        {
            // Derive integer index range from world bounds
            (int minX, int minY, int minZ) = GridConverter.WorldToGridMin(bounds.Min, voxelSize);
            (int gx1, int gy1, int gz1) = GridConverter.WorldToGridMaxInclusive(bounds.Max, voxelSize);
            int nx = gx1 - minX + 1;
            int ny = gy1 - minY + 1;
            int nz = gz1 - minZ + 1;
            if (nx <= 0 || ny <= 0 || nz <= 0)
                throw new ArgumentException("Invalid grid dimensions derived from bounds and voxel size.");
            Meta = new GridMeta(minX, minY, minZ, nx, ny, nz, voxelSize);

            _occupancies = new Occupancy[Meta.VoxelCount];
            RecomputeBounds();
        }

        public OccupancyGrid(GridMeta meta)
        {
            if (meta.Nx <= 0 || meta.Ny <= 0 || meta.Nz <= 0)
                throw new ArgumentException("Grid dimensions must be positive.");
            Meta = meta;
            _occupancies = new Occupancy[meta.VoxelCount];
            RecomputeBounds();
        }

        public OccupancyGrid(Occupancy[] occupancies, Vector3[] positions, GridMeta meta)
        {
            if (occupancies is null) throw new ArgumentNullException(nameof(occupancies));
            if (positions is null) throw new ArgumentNullException(nameof(positions));
            if (occupancies.Length != positions.Length)
                throw new ArgumentException("Occupancy and position arrays must have the same length.");

            if (meta.Nx <= 0 || meta.Ny <= 0 || meta.Nz <= 0)
                throw new ArgumentException("Grid dimensions must be positive.");
            if (!(meta.VoxelSize.X > 0 && meta.VoxelSize.Y > 0 && meta.VoxelSize.Z > 0))
                throw new ArgumentException("Voxel size must be positive.");

            long voxelCount = (long)meta.Nx * meta.Ny * meta.Nz;
            if (voxelCount > int.MaxValue)
                throw new OutOfMemoryException("Grid too large to store in a flat array.");

            Meta = meta; // snapshot of geometry & bounds
            _occupancies = new Occupancy[voxelCount];

            // Fill array: quantize each position to voxel index, compute linear index, assign
            for (int i = 0; i < occupancies.Length; i++)
            {
                Vector3 p = positions[i];
                (int gx, int gy, int gz) = GridConverter.WorldToGridMin(p, meta.VoxelSize);

                int lx = gx - meta.MinX;
                int ly = gy - meta.MinY;
                int lz = gz - meta.MinZ;

                if ((uint)lx >= (uint)meta.Nx ||
                    (uint)ly >= (uint)meta.Ny ||
                    (uint)lz >= (uint)meta.Nz)
                {
                    throw new ArgumentOutOfRangeException(nameof(positions),
                        $"Position {p} is outside provided grid meta bounds.");
                }

                int idx = (lz * meta.Ny + ly) * meta.Nx + lx; // z,y,x order, X-fastest
                _occupancies[idx] = occupancies[i];
            }

            RecomputeBounds();
        }

        #endregion

        // ------------ Copy Constructors ------------
        #region Copy Constructors

        private OccupancyGrid(OccupancyGrid other)
        {
            Meta = other.Meta; // struct copy
            _occupancies = new Occupancy[other._occupancies.Length];
            Array.Copy(other._occupancies, _occupancies, _occupancies.Length);
            RecomputeBounds();
        }
        public OccupancyGrid Clone() => new(this);

        #endregion

        // ------------ PUBLIC INTERFACE: storage operations ------------
        #region Storage Operations
        public Occupancy GetValue((int x, int y, int z) index)
        {
            if (!Contains(index))
                throw new IndexOutOfRangeException($"Index {index} is out of grid bounds.");
            int lx = index.x - Meta.MinX;
            int ly = index.y - Meta.MinY;
            int lz = index.z - Meta.MinZ;
            int linear = (lz * Meta.Ny + ly) * Meta.Nx + lx; // same as lz*Nx*Ny + ly*Nx + lx
            return _occupancies[linear];
        }

        public void SetValue((int x, int y, int z) index, Occupancy value)
        {
            if (!Contains(index))
                throw new IndexOutOfRangeException($"Index {index} is out of grid bounds.");
            int lx = index.x - Meta.MinX;
            int ly = index.y - Meta.MinY;
            int lz = index.z - Meta.MinZ;
            int linear = (lz * Meta.Ny + ly) * Meta.Nx + lx; // same as lz*Nx*Ny + ly*Nx + lx
            _occupancies[linear] = value;
        }

        public ReadOnlySpan<Occupancy> GetValues() => _occupancies;

#if NETFRAMEWORK
        public void Fill(Occupancy value)
        {
            for (int i = 0; i < _occupancies.Length; i++)
                _occupancies[i] = value;
        }
#else
        public void Fill(Occupancy value) => Array.Fill(_occupancies, value);
#endif


        public void LoadFromArray(ReadOnlySpan<Occupancy> values)
        {
            if (values.Length != _occupancies.Length)
                throw new ArgumentException("Input array length does not match grid size.");
            values.CopyTo(_occupancies);
        }

        /// <summary>
        /// Export distances as a copy (flat row-major array).
        /// </summary>
        public float[] ToArrayCopy()
        {
            float[] copy = new float[_occupancies.Length];
            Array.Copy(_occupancies, copy, copy.Length);
            return copy;
        }

        #endregion

        // ------------ PUBLIC: export operations ------------
        /// <summary>
        /// Enumerate all voxels with their world-space positions and occupancy values.
        /// </summary>
        /// <remarks>
        /// Examples:
        /// <code>
        /// foreach (var (pos, occ) in grid.EnumerateVoxels())
        /// {
        ///     // process pos and occ
        /// }
        /// </code>
        /// </remarks>
        /// <returns>sequence of (position, occupancy) tuples</returns>
        public IEnumerable<(Vector3 position, Occupancy value)> EnumerateVoxels()
            => this.EnumerateVoxels(this);

        // ------------ PUBLIC: counting operations ------------

        /// <summary>
        /// Count voxels in a given <see cref="Occupancy"/> state.
        /// </summary>
        /// <param name="state">state of occupancy to count</param>
        public long CountState(Occupancy state)
        {
            return _occupancies.Count(v => v == state);
        }

        /// <summary>
        /// Count total number of voxels in the grid.
        /// </summary>
        public long CountState()
        {
            return _occupancies.Length;
        }


        // ------------ Transformations ------------
        /// <summary>
        /// Translates the grid by a world-space vector.
        /// Internally shifts content by the nearest integer voxel offset. Cells shifting out of range are dropped.
        /// </summary>
        /// <param name="translation">World-space translation.</param>
        public void TransformWorld(Vector3 translation)
        {
            int nx = Meta.Nx; int ny = Meta.Ny; int nz = Meta.Nz;
            if (Meta.Nx == 0 || Meta.Ny == 0 || Meta.Nz == 0) return;

            // Convert world translation to integer voxel offset
            (int dx, int dy, int dz) = GridConverter.WorldToGridMin(translation, Meta.VoxelSize);
            if (dx == 0 && dy == 0 && dz == 0)
                return;

            Occupancy[] next = new Occupancy[_occupancies.Length];
            // prefill with outside
#if NETFRAMEWORK
            for (int i = 0; i < next.Length; i++)
                next[i] = Occupancy.Outside;
#else
            Array.Fill(next, Occupancy.Outside);
#endif

            // compute source range that remain in bounds after shift by (dx,dy,dz)
            // copy from src -> dst where dst = src + (dx,dy,dz)
            int srcStartX = Math.Max(0, -dx);
            int srcEndX = Math.Min(nx - 1, nx - 1 - dx);
            int srcStartY = Math.Max(0, -dy);
            int srcEndY = Math.Min(ny - 1, ny - 1 - dy);
            int srcStartZ = Math.Max(0, -dz);
            int srcEndZ = Math.Min(nz - 1, nz - 1 - dz);

            if (srcStartX <= srcEndX && srcStartY <= srcEndY && srcStartZ <= srcEndZ)
            {
                int runLenX = srcEndX - srcStartX + 1;

                // Copy whole X-runs per (y,z) with Array.Copy (X-fastest layout)
                for (int z = srcStartZ; z <= srcEndZ; z++)
                {
                    int dstZ = z + dz;
                    int srcZBase = z * ny * nx;
                    int dstZBase = dstZ * ny * nx;

                    for (int y = srcStartY; y <= srcEndY; y++)
                    {
                        int dstY = y + dy;

                        int srcBase = srcZBase + y * nx + srcStartX;
                        int dstBase = dstZBase + dstY * nx + (srcStartX + dx);

                        Array.Copy(_occupancies, srcBase, next, dstBase, runLenX);
                    }
                }
            }

            Meta = new GridMeta(Meta.MinX + dx, Meta.MinY + dy, Meta.MinZ + dz, Meta.Nx, Meta.Ny, Meta.Nz, Meta.VoxelSize);

            _occupancies = next;
            RecomputeBounds();
        }

        /// <summary>
        /// Translates the grid so that <paramref name="origin"/> moves to <paramref name="target"/>.
        /// Equivalent to <c>TransformWorld(target - origin)</c>.
        /// </summary>
        /// <param name="origin">World-space origin point.</param>
        /// <param name="target">World-space target point.</param>
        public void TransformWorld(Vector3 origin, Vector3 target)
            => TransformWorld(target - origin);

        // ------------ PUBLIC: Masking operations ------------

        /// <summary>
        /// Export occupancy as a ternary mask.
        /// </summary>
        /// <param name="flipSign">flip outside and inside</param>
        /// <returns>(0=outside, 1=inside, 2=boundary)</returns>
        public byte[] GetMaskTernary(bool flipSign = false)
        {
            byte[] mask = new byte[_occupancies.Length];
            for (int i = 0; i < _occupancies.Length; i++)
            {
                switch (_occupancies[i])
                {
                    case Occupancy.Outside: // default: 0
                        mask[i] = flipSign ? (byte)1 : (byte)0;
                        break;
                    case Occupancy.Inside: // default: 1
                        mask[i] = flipSign ? (byte)0 : (byte)1;
                        break;
                    case Occupancy.Boundary:
                        mask[i] = 2;
                        break;
                }
            }
            return mask;
        }

        /// <summary>
        /// Export occupancy as a binary mask.
        /// </summary>
        /// <param name="flipSign">flip outside and inside</param>
        /// <param name="boundaryAsInside">treat boundary as inside?</param>
        /// <returns>(0=outside, 1=inside)</returns>
        public byte[] GetMaskBinary(bool flipSign = false, bool boundaryAsInside = false)
        {
            byte[] mask = new byte[_occupancies.Length];
            for (int i = 0; i < _occupancies.Length; i++)
            {
                switch (_occupancies[i])
                {
                    case Occupancy.Outside: // default: 0
                        mask[i] = flipSign ? (byte)1 : (byte)0;
                        break;
                    case Occupancy.Inside: // default: 1
                        mask[i] = flipSign ? (byte)0 : (byte)1;
                        break;
                    case Occupancy.Boundary:
                        mask[i] = boundaryAsInside ? (flipSign ? (byte)0 : (byte)1) : (flipSign ? (byte)1 : (byte)0);
                        break;
                }
            }
            return mask;
        }
    }
}
