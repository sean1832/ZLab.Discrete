using System.Numerics;
using System.Runtime.CompilerServices;
using ZLab.Discrete.Geometry;

namespace ZLab.Discrete.Grids
{
    /// <summary>
    /// Base class for 3D grid structures, providing common properties and methods.
    /// </summary>
    public abstract class GridBase
    {
        /// <summary>
        /// Metadata about the grid, including dimensions and voxel size.
        /// </summary>
        public GridMeta Meta { get; protected set; }

        /// <summary>
        /// Axis-aligned bounding box of the grid in world coordinates.
        /// </summary>
        public BBox Bounds { get; protected set; }

        private protected const float Epsilon = 1e-6f;

        // ------------ private protected utilities ------------

        private protected void RecomputeBounds()
        {
            if (Meta.Nx <= 0 || Meta.Ny <= 0 || Meta.Nz <= 0)
            {
                Bounds = new BBox(Vector3.Zero, Vector3.Zero);
                return;
            }
            // min corner of the first voxel
            Vector3 min = GridConverter.IndexToMinCorner(Meta.MinX, Meta.MinY, Meta.MinZ, Meta.VoxelSize);
            // max corner of the last voxel
            Vector3 max = GridConverter.IndexToMaxCorner(
                Meta.MinX + Meta.Nx - 1,
                Meta.MinY + Meta.Ny - 1,
                Meta.MinZ + Meta.Nz - 1,
                Meta.VoxelSize);
            Bounds = new BBox(min, max);
        }

        /// <summary>
        /// Checks if the given 3D index is within the bounds of the grid.
        /// </summary>
        /// <param name="index">The 3D index (x, y, z) to check.</param>
        /// <returns>True if the index is within the grid bounds; otherwise, false.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Contains((int x, int y, int z) index)
        {
            return index.x >= Meta.MinX && index.x < Meta.MinX + Meta.Nx
                                       && index.y >= Meta.MinY && index.y < Meta.MinY + Meta.Ny
                                       && index.z >= Meta.MinZ && index.z < Meta.MinZ + Meta.Nz;
        }

    }
}
