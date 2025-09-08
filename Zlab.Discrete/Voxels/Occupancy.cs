namespace ZLab.Discrete.Voxels
{
    /// <summary>
    /// Occupancy state of a voxel.
    /// </summary>
    public enum Occupancy:byte
    {
        /// <summary>
        /// The voxel is outside the geometry.
        /// </summary>
        Outside = 0,
        /// <summary>
        /// The voxel is inside the geometry.
        /// </summary>
        Inside = 1,
        /// <summary>
        /// The voxel intersects the boundary of the geometry.
        /// </summary>
        Boundary = 2
    }
}
