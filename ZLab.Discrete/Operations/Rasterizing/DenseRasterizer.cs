using System;
using System.Numerics;
using System.Threading.Tasks;
using ZLab.Discrete.Algorithms.Collision;
using ZLab.Discrete.Geometry;
using ZLab.Discrete.Grids;

namespace ZLab.Discrete.Operations.Rasterizing
{
    /// <summary>
    /// Rasterizes a mesh into a dense occupancy grid.
    /// </summary>
    public static class DenseRasterizer
    {
        /// <summary>
        /// Rasterizes a mesh into the given <see cref="OccupancyGrid"/>.
        /// Optionally applies flood fill to classify interior voxels. (<paramref name="grid"/> is mutated in-place.)
        /// </summary>
        /// <param name="grid">
        /// Target grid to populate (mutated in-place).
        /// Callers must allocate the grid before calling.
        /// </param>
        /// <param name="mesh">Mesh to discretize into voxels.</param>
        /// <param name="floodFill">If true, runs flood fill to label inside/outside voxels.</param>
        /// <param name="parallelThreshold">Face count threshold for enabling parallel processing.</param>
        public static void RasterizeMesh(OccupancyGrid grid, MeshF mesh, bool floodFill=false, int parallelThreshold=2048)
        {
            if (grid == null) throw new ArgumentNullException(nameof(grid));

            // Check if the mesh intersects the grid bounds
            BBox meshBounds = mesh.GetBounds();
            BBox gridBounds = grid.Bounds;
            if (!gridBounds.Intersects(meshBounds)) return;


            if (mesh.Faces.Length == 0 || !mesh.IsValid) return;
            if (mesh.Faces.Length > parallelThreshold)
            {
                Parallel.ForEach(mesh.Faces, face =>
                {
                    Rasterizer.RasterizeFaceInGrid(grid, mesh, face);
                });
            }
            else
            {
                foreach (TriFace face in mesh.Faces)
                {
                    Rasterizer.RasterizeFaceInGrid(grid, mesh, face);
                }
            }

            if (floodFill && mesh.IsClosed)
                FloodFill.Fill3D(grid);
        }

        /// <summary>
        /// Rasterizes a 3D polyline into the given <see cref="OccupancyGrid"/>.
        /// </summary>
        /// <param name="grid">Target grid to populate (mutated in-place).</param>
        /// <param name="polyline">Polyline to rasterize.</param>
        /// <exception cref="ArgumentNullException">Thrown if <paramref name="grid"/> or <paramref name="polyline"/> is null.</exception>
        public static void RasterizePolyline(OccupancyGrid grid, PolylineF polyline)
        {
            if (grid == null) throw new ArgumentNullException(nameof(grid));
            if (polyline == null) throw new ArgumentNullException(nameof(polyline));
            if (polyline.Count < 2) return;
            // Check if the polyline intersects the grid bounds
            BBox polyBounds = polyline.GetBounds();
            if (polyline.Count == 0) return;

            BBox gridBounds = grid.Bounds;
            if (!gridBounds.Intersects(polyBounds)) return;

            Rasterizer.RasterizePolylineInGrid(grid, polyline);
        }

    }
}
