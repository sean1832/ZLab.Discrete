using System;
using System.Collections.Generic;
using System.Numerics;
using ZLab.Discrete.Algorithms.DistanceTransforms;
using ZLab.Discrete.Algorithms.Sampling;
using ZLab.Discrete.Geometry;
using ZLab.Discrete.Grids.Interfaces;

namespace ZLab.Discrete.Grids
{
    /// <summary>
    /// Distance field grid structure.
    /// </summary>
    public sealed class DistanceGrid: GridBase, IGrid<float>
    {
        /// <summary>
        /// flat array in row-major order (x fastest, then y, then z)
        /// </summary>
        private readonly float[] _distances; // length = Nx * Ny * Nz

        // ------------ Constructors ------------
        #region Constructors
        public DistanceGrid(Vector3 voxelSize, BBox bounds)
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

            _distances = new float[Meta.VoxelCount];
            RecomputeBounds();
        }

        public DistanceGrid(GridMeta meta)
        {
            if (meta.Nx <= 0 || meta.Ny <= 0 || meta.Nz <= 0)
                throw new ArgumentException("Grid dimensions must be positive.");
            Meta = meta;
            _distances = new float[meta.VoxelCount];
            RecomputeBounds();
        }

        public DistanceGrid(OccupancyGrid og)
        {
            Meta = og.Meta;
            _distances = new float[Meta.VoxelCount];
            RecomputeBounds();
        }

        #endregion

        // ------------ Copy Constructors ------------
        #region Copy Constructors
        private DistanceGrid(DistanceGrid other)
        {
            Meta = other.Meta; // struct copy
            _distances = new float[other._distances.Length];
            Array.Copy(other._distances, _distances, _distances.Length);
            RecomputeBounds();
        }
        public DistanceGrid Clone() => new(this);

        #endregion

        // ------------ PUBLIC INTERFACE: storage operations ------------
        #region Storage Operations

        /// <summary>
        /// Get distance value at given grid index.
        /// </summary>
        public float GetValue((int x, int y, int z) index)
        {
            if (!TryToLinearIndex(index, out int linear))
                throw new IndexOutOfRangeException($"Index {index} is out of grid bounds.");

            return _distances[linear];
        }

        /// <summary>
        /// Get distance value at given world position.
        /// </summary>
        /// <param name="position">World position</param>
        /// <param name="continues">If true, trilinear interpolation is used; otherwise, nearest voxel value is returned.</param>
        public float GetValue(Vector3 position)
        {
            return GetValue(GridConverter.WorldToGridMin(position, Meta.VoxelSize));
        }

        /// <summary>
        /// Get all distance values as a read-only span (flat row-major array).
        /// </summary>
        public ReadOnlySpan<float> GetReadOnlyBuffer() => _distances;

        /// <summary>
        /// Get all distance values as a mutable span (flat row-major array).
        /// </summary>
        public Span<float> GetBuffer() => _distances;

        /// <summary>
        /// Set distance value at given grid index.
        /// </summary>
        public void SetValue((int x, int y, int z) index, float value)
        {
            if (!TryToLinearIndex(index, out int linear))
                throw new IndexOutOfRangeException($"Index {index} is out of grid bounds.");
            _distances[linear] = value;
        }
#if NETFRAMEWORK
        /// <summary>
        /// Fill all distances with a constant value.
        /// </summary>
        public void Fill(float value)
        {
            for (int i = 0; i < _distances.Length; i++)
                _distances[i] = value;
        }
#else
        /// <summary>
        /// Fill all distances with a constant value.
        /// </summary>
        public void Fill(float value) => Array.Fill(_distances, value);
#endif

        /// <summary>
        /// Get minimum and maximum distance values in the grid.
        /// </summary>
        public (float minVal, float maxVal) GetMinMax()
        {
            if (_distances.Length == 0) return (0, 0);
            float min = _distances[0], max = _distances[0];
            for (int i = 1; i < _distances.Length; i++)
            {
                float v = _distances[i];
                if (v < min) min = v;
                if (v > max) max = v;
            }
            return (min, max);
        }

        /// <summary>
        /// Load a precomputed SDF. The array length must match Nx*Ny*Nz.
        /// </summary>
        public void LoadFromArray(ReadOnlySpan<float> source)
        {
            if (source.Length != _distances.Length)
                throw new ArgumentException("Source length must match grid voxel count.");
            source.CopyTo(_distances);
        }

        /// <summary>
        /// Export distances as a copy (flat row-major array).
        /// </summary>
        public float[] ToArrayCopy()
        {
            float[] copy = new float[_distances.Length];
            Array.Copy(_distances, copy, copy.Length);
            return copy;
        }

        /// <summary>
        /// Add a constant offset to all distances (e.g., morph iso-surface).
        /// </summary>
        public void AddOffset(float delta)
        {
            for (int i = 0; i < _distances.Length; i++) _distances[i] += delta;
        }

        #endregion

        // ------------ PUBLIC: enumeration ------------
        public IEnumerable<(Vector3 position, float value)> EnumerateVoxels()
            => this.EnumerateVoxels(this);


        // ------------ PUBLIC: SDF build (exact EDT, anisotropic) ------------
        /// <summary>
        /// Compute exact Euclidean SDF (positive outside, negative inside) from a binary mask.
        /// The binary mask is row-major with x-fastest layout; length must be Nx*Ny*Nz.
        /// </summary>
        /// <param name="binaryMask">Binary mask (0=outside, 1=inside)</param>
        /// <param name="parallel">Enable parallelism</param>
        public void BuildFromBinaryMask(ReadOnlySpan<byte> binaryMask, bool parallel = true)
        {
            if (binaryMask.Length != Meta.Nx * Meta.Ny * Meta.Nz)
                throw new ArgumentException("Binary mask length must match grid voxel count.");

            // EDT (anisotropic).
            float[] sdf = Sdf3D.FromBinaryMaskAnisotropic(
                binaryMask, Meta.Nx, Meta.Ny, Meta.Nz, Meta.VoxelSize.X, Meta.VoxelSize.Y, Meta.VoxelSize.Z, parallel);

            Array.Copy(sdf, _distances, _distances.Length);
        }

        /// <summary>
        /// Compute exact Euclidean SDF (positive outside, negative inside) from a ternary mask.
        /// </summary>
        /// <param name="ternaryMask">Ternary mask (0=outside, 1=inside, 2=boundary)</param>
        /// <param name="parallel">Enable parallelism</param>
        public void BuildFromTernaryMask(ReadOnlySpan<byte> ternaryMask, bool parallel = true)
        {
            if (ternaryMask.Length != Meta.Nx * Meta.Ny * Meta.Nz)
                throw new ArgumentException("Ternary mask length must match grid voxel count.");

            float[] sdf = Sdf3D.FromTernaryMaskAnisotropic(
                ternaryMask, Meta.Nx, Meta.Ny, Meta.Nz,
                Meta.VoxelSize.X, Meta.VoxelSize.Y, Meta.VoxelSize.Z,
                parallel);

            Array.Copy(sdf, _distances, _distances.Length);
        }

        // ------------ PRIVATE HELPERS ------------
        #region Private Helpers
        private int ToLinearIndex((int x, int y, int z) index)
            => (index.z - Meta.MinZ) * Meta.Nx * Meta.Ny + (index.y - Meta.MinY) * Meta.Nx + (index.x - Meta.MinX);

        private bool TryToLinearIndex((int x, int y, int z) index, out int linear)
        {
            if (!Contains(index)) { linear = -1; return false; }
            linear = ToLinearIndex(index);
            return true;
        }
        #endregion
    }
}
