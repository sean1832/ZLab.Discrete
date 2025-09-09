using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using ZLab.Discrete.Core;
using ZLab.Discrete.Grids;

namespace ZLab.Discrete.Algorithms.Sampling
{
    /// <summary>
    /// Extension methods for sampling a <see cref="DistanceGrid"/>.
    /// </summary>
    public static class DistanceGridSamplingExtension
    {
        /// <summary>
        /// Trilinear sample of a <see cref="DistanceGrid"/> at an arbitrary world position.
        /// </summary>
        /// <param name="grid">The distance grid to sample.</param>
        /// <param name="worldPos">The arbitrary world position to sample.</param>
        /// <param name="clampToBounds">
        /// If true, samples outside the grid are clamped to the outermost valid cell; 
        /// if false, an <see cref="ArgumentOutOfRangeException"/> is thrown when sampling out of bounds.
        /// </param>
        /// <returns>Distance to the nearest surface at the specified world position.</returns>
        public static float SampleTrilinear(this DistanceGrid grid, Vector3 worldPos, bool clampToBounds = true)
        {
            GridMeta meta = grid.Meta;
            Vector3 vSize = meta.VoxelSize;
            Vector3 originWorld = grid.Bounds.Min;

            // Find the index of the voxel cell containing the query position (lower corner)
            (int cellX, int cellY, int cellZ) = GridConverter.WorldToGridMin(worldPos, vSize, originWorld);

            // Handle degenerate dimensions (only 1 voxel in that dimension)
            bool singleX = meta.Nx <= 1;
            bool singleY = meta.Ny <= 1;
            bool singleZ = meta.Nz <= 1;

            // clamp / validate lower cell indices
            int cellX0 = singleX ? 0 : ClampOrThrow(cellX, 0, meta.Nx - 2, clampToBounds);
            int cellY0 = singleY ? 0 : ClampOrThrow(cellY, 0, meta.Ny - 2, clampToBounds);
            int cellZ0 = singleZ ? 0 : ClampOrThrow(cellZ, 0, meta.Nz - 2, clampToBounds);

            // World position of the lower corner of the cell
            Vector3 cellMinWorld = GridConverter.IndexToMinCorner(cellX0, cellY0, cellZ0, vSize, originWorld);

            // clamp offset within the cell to [0,1]
            float fracX = singleX ? 0f : MathFx.Clamp((worldPos.X - cellMinWorld.X) / vSize.X, 0f, 1f);
            float fracY = singleY ? 0f : MathFx.Clamp((worldPos.Y - cellMinWorld.Y) / vSize.Y, 0f, 1f);
            float fracZ = singleZ ? 0f : MathFx.Clamp((worldPos.Z - cellMinWorld.Z) / vSize.Z, 0f, 1f);

            // Convert local indices to global grid indices
            int gx0 = meta.MinX + cellX0;
            int gx1 = singleX ? gx0 : gx0 + 1;

            int gy0 = meta.MinY + cellY0;
            int gy1 = singleY ? gy0 : gy0 + 1;

            int gz0 = meta.MinZ + cellZ0;
            int gz1 = singleZ ? gz0 : gz0 + 1;

            // Fetch the 8 corner samples
            float v000 = grid.GetValue((gx0, gy0, gz0));
            float v100 = grid.GetValue((gx1, gy0, gz0));
            float v010 = grid.GetValue((gx0, gy1, gz0));
            float v110 = grid.GetValue((gx1, gy1, gz0));
            float v001 = grid.GetValue((gx0, gy0, gz1));
            float v101 = grid.GetValue((gx1, gy0, gz1));
            float v011 = grid.GetValue((gx0, gy1, gz1));
            float v111 = grid.GetValue((gx1, gy1, gz1));

            // interpolate along x
            float c00 = MathFx.Lerp(v000, v100, fracX);
            float c10 = MathFx.Lerp(v010, v110, fracX);
            float c01 = MathFx.Lerp(v001, v101, fracX);
            float c11 = MathFx.Lerp(v011, v111, fracX);

            // interpolate along y
            float c0 = MathFx.Lerp(c00, c10, fracY);
            float c1 = MathFx.Lerp(c01, c11, fracY);

            // interpolate along z
            float c = MathFx.Lerp(c0, c1, fracZ);
            return c;
        }

        /// <summary>
        /// Estimates the gradient of the signed distance field at a world-space position
        /// using central differences over one voxel step per axis.
        /// </summary>
        /// <param name="grid">Distance grid to sample</param>
        /// <param name="worldPos">Query position in world space.</param>
        /// <param name="clampToBounds">
        /// If true, samples outside the grid are clamped to the outermost valid cell; 
        /// if false, an <see cref="ArgumentOutOfRangeException"/> is thrown when sampling out of bounds.
        /// </param>
        /// <returns>
        /// Gradient vector in world units (X,Y,Z). For a true SDF, normalizing this vector
        /// yields the surface normal at <paramref name="worldPos"/>.
        /// </returns>
        /// <remarks>
        /// Uses trilinear samples per axis.
        /// Degenerate axes (dimension = 1) produce zero derivative on that axis.
        /// Result quality depends on grid resolution and SDF accuracy (it is an approximation of the mesh normal).
        /// </remarks>
        public static Vector3 SampleGradient(this DistanceGrid grid, Vector3 worldPos, bool clampToBounds = true)
        {
            GridMeta meta = grid.Meta;
            Vector3 vSize = meta.VoxelSize;

            // handle degenerate dimensions (only 1 voxel in that dimension)
            bool singleX = meta.Nx <= 1;
            bool singleY = meta.Ny <= 1;
            bool singleZ = meta.Nz <= 1;

            float dx = 0f, dy = 0f, dz = 0f;
            if (!singleX)
            {
                float stepX = vSize.X;
                float fPlus = grid.SampleTrilinear(new Vector3(worldPos.X + stepX, worldPos.Y, worldPos.Z),
                    clampToBounds);
                float fMinus = grid.SampleTrilinear(new Vector3(worldPos.X - stepX, worldPos.Y, worldPos.Z),
                    clampToBounds);
                dx = (fPlus - fMinus) / (2f * stepX);
            }

            if (!singleY)
            {
                float stepY = vSize.Y;
                float fPlus = grid.SampleTrilinear(new Vector3(worldPos.X, worldPos.Y + stepY, worldPos.Z),
                    clampToBounds);
                float fMinus = grid.SampleTrilinear(new Vector3(worldPos.X, worldPos.Y - stepY, worldPos.Z),
                    clampToBounds);
                dy = (fPlus - fMinus) / (2f * stepY);
            }

            if (!singleZ)
            {
                float stepZ = vSize.Z;
                float fPlus = grid.SampleTrilinear(new Vector3(worldPos.X, worldPos.Y, worldPos.Z + stepZ),
                    clampToBounds);
                float fMinus = grid.SampleTrilinear(new Vector3(worldPos.X, worldPos.Y, worldPos.Z - stepZ),
                    clampToBounds);
                dz = (fPlus - fMinus) / (2f * stepZ);
            }

            return new Vector3(dx, dy, dz);
        }

        /// <summary>
        /// Returns a unit-length normal estimated from the SDF gradient at a world-space position.
        /// </summary>
        /// <param name="grid">Distance grid to sample</param>
        /// <param name="worldPos">Query position in world space.</param>
        /// <param name="clampToBounds">
        /// If true, samples outside the grid are clamped to the outermost valid cell; 
        /// if false, an <see cref="ArgumentOutOfRangeException"/> is thrown when sampling out of bounds.
        /// </param>
        /// <returns>
        /// Unit normal vector. Returns <see cref="Vector3.Zero"/> if the gradient magnitude is near zero.
        /// </returns>
        /// <remarks>
        /// Normal is computed as <c>Normalize(SampleGradient(worldPos))</c>. Suitable for shading, collision response,
        /// and analysis; accuracy improves with grid resolution.
        /// </remarks>
        public static Vector3 SampleNormal(this DistanceGrid grid, Vector3 worldPos, bool clampToBounds = true)
        {
            Vector3 g = grid.SampleGradient(worldPos, clampToBounds);
            float len = g.Length();
            return len > 1e-8f ? g / len : Vector3.Zero;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static int ClampOrThrow(int value, int min, int max, bool clamp)
        {
            if (clamp) return MathFx.Clamp(value, min, max);
            if (value < min || value > max)
                throw new ArgumentOutOfRangeException(nameof(value), "Position is outside the grid bounds.");
            return value;
        }
    }
}
