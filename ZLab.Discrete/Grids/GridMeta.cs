using System.Numerics;

namespace ZLab.Discrete.Grids
{
    /// <summary>
    /// Metadata for a 3D grid, including its dimensions, minimum coordinates, and voxel size.
    /// </summary>
    public readonly struct GridMeta
    {
        /// <summary>
        /// Minimum X coordinate of the grid.
        /// </summary>
        public int MinX { get; }
        /// <summary>
        /// Minimum Y coordinate of the grid.
        /// </summary>
        public int MinY { get; }
        /// <summary>
        /// Minimum Z coordinate of the grid.
        /// </summary>
        public int MinZ { get; }
        /// <summary>
        /// Number of voxels along the X axis.
        /// </summary>
        public int Nx { get; }
        /// <summary>
        /// Number of voxels along the Y axis.
        /// </summary>
        public int Ny { get; }
        /// <summary>
        /// Number of voxels along the Z axis.
        /// </summary>
        public int Nz { get; }
        /// <summary>
        /// Size of each voxel in world units.
        /// </summary>
        public Vector3 VoxelSize { get; }

        /// <summary>
        /// Creates a new GridMeta instance with the specified parameters.
        /// </summary>
        /// <param name="minX">minimum X coordinate</param>
        /// <param name="minY">minimum Y coordinate</param>
        /// <param name="minZ">minimum Z coordinate</param>
        /// <param name="nx">number of voxels along X</param>
        /// <param name="ny">number of voxels along Y</param>
        /// <param name="nz">number of voxels along Z</param>
        /// <param name="voxelSize">size of each voxel in world units</param>
        public GridMeta(int minX, int minY, int minZ, int nx, int ny, int nz, Vector3 voxelSize)
        {
            MinX = minX; MinY = minY; MinZ = minZ;
            Nx = nx; Ny = ny; Nz = nz;
            VoxelSize = voxelSize;
        }

        /// <summary>
        /// Total number of voxels in the grid.
        /// </summary>
        public long Count => (long)Nx * Ny * Nz;

        /// <summary>
        /// Size dimensions of the entire grid in world units.
        /// </summary>
        public Vector3 SizeWorld => new(Nx * VoxelSize.X, Ny * VoxelSize.Y, Nz * VoxelSize.Z);

        /// <summary>
        /// Dimensions of the grid as a tuple (Nx, Ny, Nz) in voxel counts.
        /// </summary>
        public (int X, int Y, int Z) Dimensions => (Nx, Ny, Nz);
    }
}
