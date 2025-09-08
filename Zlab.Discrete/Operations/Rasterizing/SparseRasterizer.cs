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
    public static class SparseRasterizer
    {
        /// <summary>
        /// Discretize a mesh. Only voxels that intersect the mesh are returned.
        /// </summary>
        /// <param name="mesh">Mesh to discretize</param>
        /// <param name="voxelSize">Size of each voxel</param>
        /// <param name="parallelThreshold">Mesh vertices threshold for parallelism</param>
        /// <returns><see cref="OccupancyVoxel"/> intersects with the mesh boundary</returns>
        public static List<Vector3> Rasterize(
            MeshF mesh, Vector3 voxelSize, 
            int parallelThreshold = 2048)
        {
            TriFace[] faces = mesh.Faces;
            if (faces.Length == 0) return new List<Vector3>();

            // small meshes: single-thread
            if (faces.Length <= parallelThreshold)
            {
                List<Vector3> outList = new(faces.Length * 8); // heuristic
                foreach (TriFace face in faces)
                    Rasterizer.RasterizeFace(mesh, face, voxelSize).ForEach(
                        v => outList.Add(v)
                    );
                return outList;
            }
            // large meshes: parallel with thread-local lists
            OrderablePartitioner<Tuple<int, int>> partitioner = Partitioner.Create(
                0, faces.Length, Math.Max(1, faces.Length / (Environment.ProcessorCount * 8))
            );
            ConcurrentBag<List<Vector3>> buckets = new();

            Parallel.ForEach(
                partitioner,
                () => new List<Vector3>(256),
                (range, state, local) =>
                {
                    for (int i = range.Item1; i < range.Item2; i++)
                    {
                        List<Vector3> faceList = Rasterizer.RasterizeFace(mesh, faces[i], voxelSize);
                        // append into thread-local buffer
                        local.AddRange(faceList);
                    }
                    return local;
                },
                local => buckets.Add(local)
            );

            // concat
            int total = buckets.Sum(b => b.Count);
            List<Vector3> result = new(total);
            foreach (List<Vector3> b in buckets) 
                result.AddRange(b);
            return result;
        }
    }
}
