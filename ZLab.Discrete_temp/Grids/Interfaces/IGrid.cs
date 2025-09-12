using System;

namespace ZLab.Discrete.Grids.Interfaces
{
    /// <summary>
    /// Interface for a 3D grid structure that allows getting and setting values at specific indices,
    /// </summary>
    /// <typeparam name="T">The type of values stored in the grid.</typeparam>
    public interface IGrid<T>
    {
        /// <summary>
        /// Metadata about the grid, including dimensions and voxel size.
        /// </summary>
        /// <param name="index">The 3D index (x, y, z) of the voxel.</param>
        /// <returns>The value stored at the specified index.</returns>
        T GetValue((int x, int y, int z) index);

        /// <summary>
        /// Sets the value at the specified 3D index in the grid.
        /// </summary>
        /// <param name="index">The 3D index (x, y, z) of the voxel.</param>
        /// <param name="value">The value to set at the specified index.</param>
        void SetValue((int x, int y, int z) index, T value);

        /// <summary>
        /// Gets a read-only span of the underlying buffer storing the grid values.
        /// </summary>
        /// <returns>A read-only span of the grid's values.</returns>
        ReadOnlySpan<T> GetReadOnlyBuffer();

        /// <summary>
        /// Gets a span of the underlying buffer storing the grid values for direct manipulation.
        /// </summary>
        /// <returns>A span of the grid's values.</returns>
        Span<T> GetBuffer();

        /// <summary>
        /// Gets a read-only memory representation of the underlying buffer storing the grid values.
        /// </summary>
        /// <returns>A read-only memory of the grid's values.</returns>
        ReadOnlyMemory<T> GetReadOnlyMemory();
    }
}
