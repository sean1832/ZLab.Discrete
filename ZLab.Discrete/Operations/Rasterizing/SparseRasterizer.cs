using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Threading.Tasks;
using ZLab.Discrete.Geometry;
using ZLab.Discrete.Voxels;

namespace ZLab.Discrete.Operations.Rasterizing
{
    internal sealed class VoxelOriginComparer : IEqualityComparer<Vector3>
    {
        private readonly Vector3 _inv; // 1/voxelSize
        public VoxelOriginComparer(Vector3 voxelSize)
        {
            _inv = new Vector3(1f / voxelSize.X, 1f / voxelSize.Y, 1f / voxelSize.Z);
        }

        private (int ix, int iy, int iz) Q(Vector3 v)
            => ((int)MathF.Round(v.X * _inv.X),
                (int)MathF.Round(v.Y * _inv.Y),
                (int)MathF.Round(v.Z * _inv.Z));

        public bool Equals(Vector3 a, Vector3 b) => Q(a) == Q(b);

        public int GetHashCode(Vector3 v)
        {
            var (ix, iy, iz) = Q(v);
            unchecked
            {
                int h = ix;
                h = (h * 397) ^ iy;
                h = (h * 397) ^ iz;
                return h;
            }
        }
    }

    /// <summary>
    /// Sparse rasterization of a mesh into voxels. Only voxels that intersect the mesh are collected.
    /// </summary>
    public static class SparseRasterizer
    {
        /// <summary>
        /// Discretize a mesh. Only voxels that intersect the mesh are returned.
        /// </summary>
        /// <param name="mesh">Mesh to discretize</param>
        /// <param name="voxelSize">Size of each voxel</param>
        /// <param name="parallelThreshold">Mesh vertices threshold for parallelism</param>
        /// <returns>voxels intersects with the mesh boundary</returns>
        public static Vector3[] RasterizeMesh(
            MeshF mesh, Vector3 voxelSize, 
            int parallelThreshold = 2048)
        {
            TriFace[] faces = mesh.Faces;
            if (faces.Length == 0) return Array.Empty<Vector3>();

            VoxelOriginComparer comparer = new(voxelSize);

            // Small meshes: single-thread, use HashSet to dedup
            if (faces.Length <= parallelThreshold)
            {
                HashSet<Vector3> set = new(comparer);
                foreach (TriFace face in faces)
                {
                    List<Vector3> voxels = Rasterizer.RasterizeFaceSparse(mesh, face, voxelSize);
                    foreach (Vector3 v in voxels) set.Add(v);
                }
                return set.ToArray();
            }

            // Large meshes: parallel. Use a ConcurrentDictionary as a concurrent set.
            ConcurrentDictionary<Vector3, byte> setConcurrent = new(comparer);

            OrderablePartitioner<Tuple<int, int>> partitioner = Partitioner.Create(
                0, faces.Length,
                Math.Max(1, faces.Length / (Environment.ProcessorCount * 8))
            );

            Parallel.ForEach(partitioner, range =>
            {
                for (int i = range.Item1; i < range.Item2; i++)
                {
                    List<Vector3> voxels = Rasterizer.RasterizeFaceSparse(mesh, faces[i], voxelSize);
                    foreach (Vector3 v in voxels) setConcurrent.TryAdd(v, 0);
                }
            });

            return setConcurrent.Keys.ToArray();
        }

        /// <summary>
        /// Discretize a 3D polyline. Only voxels that intersect the polyline are returned.
        /// </summary>
        /// <param name="polyline">Polyline to discretize</param>
        /// <param name="voxelSize">Size of each voxel</param>
        /// <param name="includeClosingEdge">If true, includes the edge connecting the last and first vertex if the polyline is closed.</param>
        /// <returns>voxels intersects with the polyline</returns>
        public static Vector3[] RasterizePolyline(PolylineF polyline, Vector3 voxelSize, bool includeClosingEdge = true)
        {
            if (polyline.Count < 2) return Array.Empty<Vector3>();
            return Rasterizer.RasterizePolylineSparse(polyline, voxelSize, includeClosingEdge);
        }
    }
}
