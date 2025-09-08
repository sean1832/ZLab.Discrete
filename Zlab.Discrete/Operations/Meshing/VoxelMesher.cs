using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using ZLab.Discrete.Algorithms.Encoding;
using ZLab.Discrete.Core;
using ZLab.Discrete.Geometry;
using ZLab.Discrete.Voxels;

namespace ZLab.Discrete.Operations.Meshing
{
    /// <summary>
    /// Naive voxel meshing utilities: emits per-voxel meshes or a single merged mesh.
    /// Uses face culling with Morton-coded occupancy when voxel size is uniform.
    /// </summary>
    /// <remarks>
    /// Assumes <see cref="OccupancyVoxel"/> instances lie on a rectilinear grid.
    /// For the culled path, all voxels must share an identical <c>voxelSize</c>.
    /// If voxel sizes vary, all six faces per voxel are emitted (no merging).
    /// </remarks>
    public static class VoxelMesher
    {
        /// <summary>
        /// Generates an individual quad-mesh per voxel (no face merging across voxels).
        /// </summary>
        /// <param name="origins">Voxel origins</param>
        /// <param name="voxelSizes">Per-voxel sizes; must match <paramref name="origins"/> length.</param>
        /// <param name="cordSystem">Coordinate system for winding (right- or left-handed).</param>
        /// <returns>Array of meshes, one per input voxel.</returns>
        /// <exception cref="ArgumentException">Lengths of <paramref name="origins"/> and <paramref name="voxelSizes"/> differ.</exception>
        /// <remarks>
        /// <b>Memory-heavy:</b> each voxel becomes a standalone box mesh (24 verts, 12 tris) unless you change the box builder.
        /// Useful for debugging or per-voxel downstream ops.
        /// </remarks>
        public static MeshF[] GenerateMeshes(
            ReadOnlySpan<Vector3> origins, ReadOnlySpan<Vector3> voxelSizes, CordSystem cordSystem = CordSystem.RightHanded)
        {
            if (voxelSizes.Length != origins.Length)
                throw new ArgumentException("voxelSizes length must match voxels length.");
            if (origins.Length == 0) return Array.Empty<MeshF>();
            MeshF[] meshes = new MeshF[origins.Length];
            for (int i = 0; i < origins.Length; i++)
            {
                Vector3 origin = origins[i];
                Vector3 size = voxelSizes[i];
                meshes[i] = new BBox(origin, origin+size).ToMesh(cordSystem);
            }
            return meshes;
        }

        /// <summary>
        /// Generates an individual quad-mesh per voxel (no face merging across voxels).
        /// </summary>
        /// <param name="origins">Voxel origins</param>
        /// <param name="voxelSizes">Per-voxel sizes; must match <paramref name="origins"/> length.</param>
        /// <param name="cordSystem">Coordinate system for winding (right- or left-handed).</param>
        /// <returns>Array of meshes, one per input voxel.</returns>
        /// <exception cref="ArgumentException">Lengths of <paramref name="origins"/> and <paramref name="voxelSizes"/> differ.</exception>
        /// <remarks>
        /// <b>Memory-heavy:</b> each voxel becomes a standalone box mesh (24 verts, 12 tris)
        /// </remarks>
        public static MeshF[] GenerateMeshes(
            List<Vector3> origins, List<Vector3> voxelSizes, CordSystem cordSystem = CordSystem.RightHanded)
        {
#if NET6_0_OR_GREATER
            return GenerateMeshes(CollectionsMarshal.AsSpan(origins), CollectionsMarshal.AsSpan(voxelSizes), cordSystem);
#else
            return GenerateMeshes(origins.ToArray(), voxelSizes.ToArray(), cordSystem);
#endif
        }

        /// <summary>
        /// Generates an individual quad-mesh per voxel (all voxels share the same <paramref name="voxelSize"/>).
        /// </summary>
        /// <param name="origins">Voxel origins</param>
        /// <param name="voxelSize">Uniform voxel size for all voxels.</param>
        /// <param name="cordSystem">Coordinate system for winding (right- or left-handed).</param>
        /// <returns>Array of meshes, one per input voxel.</returns>
        public static MeshF[] GenerateMeshes(
            ReadOnlySpan<Vector3> origins,
            Vector3 voxelSize,
            CordSystem cordSystem = CordSystem.RightHanded)
        {
            if (origins.Length == 0) return Array.Empty<MeshF>();
            MeshF[] meshes = new MeshF[origins.Length];
            for (int i = 0; i < origins.Length; i++)
                meshes[i] = new BBox(origins[i], origins[i] + voxelSize).ToMesh(cordSystem);
            return meshes;
        }


        /// <summary>
        /// Generates an individual quad-mesh per voxel (all voxels share the same <paramref name="voxelSize"/>).
        /// </summary>
        /// <param name="origins">Voxel origins</param>
        /// <param name="voxelSize">Uniform voxel size for all voxels.</param>
        /// <param name="cordSystem">Coordinate system for winding (right- or left-handed).</param>
        /// <returns>Array of meshes, one per input voxel.</returns>
        public static MeshF[] GenerateMeshes(
            List<Vector3> origins,
            Vector3 voxelSize,
            CordSystem cordSystem = CordSystem.RightHanded)
        {
#if NET6_0_OR_GREATER
            return GenerateMeshes(CollectionsMarshal.AsSpan(origins), voxelSize, cordSystem);
#else
            return GenerateMeshes(origins.ToArray(), voxelSize, cordSystem);
#endif
        }

        /// <summary>
        /// Generates a single mesh from a set of voxels. Internal faces are culled using <see cref="Morton"/> occupancy set (O(1) neighbor checks).
        /// </summary>
        /// <param name="origins">Voxel origins</param>
        /// <param name="voxelSizes">Per-voxel sizes; must match <paramref name="origins"/> length.</param>
        /// <param name="cordSystem">Coordinate system for winding (right- or left-handed).</param>
        /// <returns>A single combined mesh.</returns>
        /// <exception cref="ArgumentException">Lengths of <paramref name="origins"/> and <paramref name="voxelSizes"/> differ.</exception>
        /// <exception cref="InvalidOperationException">No vertices or faces were generated.</exception>
        /// <remarks>
        /// If <paramref name="voxelSizes"/> differ, culling is disabled and all six faces per voxel are emitted.
        /// <para>
        /// <b>Origin convention:</b> the culled path treats <see cref="OccupancyVoxel.Origin"/> as the <i>minimum corner</i> of the cell.
        /// </para>
        /// </remarks>
        public static MeshF GenerateMesh(
            ReadOnlySpan<Vector3> origins, ReadOnlySpan<Vector3> voxelSizes, CordSystem cordSystem = CordSystem.RightHanded)
        {
            if (voxelSizes.Length != origins.Length || voxelSizes.Length != origins.Length)
                throw new ArgumentException("voxelSizes length must match voxels length.");

            // worst-case 24 vertices & 12 tris per voxel (no culling)
            List<Vector3> vertices = new(origins.Length * 24);
            List<TriFace> faces = new(origins.Length * 12);

            if (origins.Length == 0)
                return new MeshF(Array.Empty<Vector3>(), Array.Empty<TriFace>());

            // check if voxelSize is uniform
            Vector3 vSize0 = voxelSizes[0];
            bool uniform = true;
            for (int i = 1; i < origins.Length; i++)
                if (!voxelSizes[i].Equals(vSize0)) { uniform = false; break; }

            if (uniform)
            {
                CullWithMorton(origins, vSize0, cordSystem, vertices, faces);
            }
            else
            {
                // fallback: emit all faces (no culling)
                for (int i = 0; i < origins.Length; i++)
                {
                    Vector3 o = origins[i];
                    Vector3 size = voxelSizes[i];
                    Vector3 originUniform = new BBox(o, o + size).Min;
                    foreach (Vector3 dir in VoxelFaceBuilder.Directions)
                    {
                        VoxelFaceBuilder.MakeFace(originUniform, dir, vertices, faces, size, cordSystem);
                    }
                }
            }
            return MakeMesh(vertices, faces);
        }

        /// <summary>
        /// Generates a single mesh from a set of voxels. Internal faces are culled using <see cref="Morton"/> occupancy set (O(1) neighbor checks).
        /// </summary>
        /// <param name="origins">Voxel origins</param>
        /// <param name="voxelSizes">Per-voxel sizes; must match <paramref name="origins"/> length.</param>
        /// <param name="cordSystem">Coordinate system for winding (right- or left-handed).</param>
        /// <returns>A single combined mesh.</returns>
        /// <exception cref="ArgumentException">Lengths of <paramref name="origins"/> and <paramref name="voxelSizes"/> differ.</exception>
        /// <exception cref="InvalidOperationException">No vertices or faces were generated.</exception>
        /// <remarks>
        /// If <paramref name="voxelSizes"/> differ, culling is disabled and all six faces per voxel are emitted.
        /// <para>
        /// The culled path treats <see cref="OccupancyVoxel.Origin"/> as the <i>minimum corner</i> of the cell.
        /// </para>
        /// </remarks>
        public static MeshF GenerateMesh(
            List<Vector3> origins, List<Vector3> voxelSizes, CordSystem cordSystem = CordSystem.RightHanded)
        {
#if NET6_0_OR_GREATER
            return GenerateMesh(CollectionsMarshal.AsSpan(origins), CollectionsMarshal.AsSpan(voxelSizes), cordSystem);
#else
            return GenerateMesh(origins.ToArray(), voxelSizes.ToArray(), cordSystem);
#endif
        }

        /// <summary>
        /// Generates a single mesh from a set of voxels, assuming a uniform <paramref name="voxelSize"/>.
        /// Internal faces are culled using <see cref="Morton"/> occupancy set (O(1) neighbor checks).
        /// </summary>
        /// <param name="origins">Voxel origins</param>
        /// <param name="voxelSize">Uniform voxel size for all voxels.</param>
        /// <param name="cordSystem">Coordinate system for winding (right- or left-handed).</param>
        /// <returns>A single combined mesh.</returns>
        /// <exception cref="InvalidOperationException">No vertices or faces were generated.</exception>
        /// <remarks>
        /// <b>Origin convention:</b> the culled path treats <see cref="OccupancyVoxel.Origin"/> as the minimum corner of the cell.
        /// </remarks>
        public static MeshF GenerateMesh(
            ReadOnlySpan<Vector3> origins,
            Vector3 voxelSize,
            CordSystem cordSystem = CordSystem.RightHanded)
        {
            if (origins.Length == 0)
                return new MeshF(Array.Empty<Vector3>(), Array.Empty<TriFace>());

            List<Vector3> vertices = new(origins.Length * 24);
            List<TriFace> faces = new(origins.Length * 12);

            // direct to culled path (since it’s uniform)
            CullWithMorton(origins, voxelSize, cordSystem, vertices, faces);
            return MakeMesh(vertices, faces);
        }

        /// <summary>
        /// Generates a single mesh from a set of voxels, assuming a uniform <paramref name="voxelSize"/>.
        /// Internal faces are culled using <see cref="Morton"/> occupancy set (O(1) neighbor checks).
        /// </summary>
        /// <param name="origins">Voxel origins</param>
        /// <param name="voxelSize">Uniform voxel size for all voxels</param>
        /// <param name="cordSystem">Coordinate system for winding (right- or left-handed)</param>
        /// <returns>A single combined mesh</returns>
        /// <exception cref="InvalidOperationException">No vertices or faces were generated</exception>
        /// <remarks>
        /// <b>Origin convention:</b> the culled path treats <see cref="OccupancyVoxel.Origin"/> as the minimum corner of the cell.
        /// </remarks>
        public static MeshF GenerateMesh(
            List<Vector3> origins,
            Vector3 voxelSize,
            CordSystem cordSystem = CordSystem.RightHanded)
        {
#if NET6_0_OR_GREATER
            return GenerateMesh(CollectionsMarshal.AsSpan(origins), voxelSize, cordSystem);
#else
            return GenerateMesh(origins.ToArray(), voxelSize, cordSystem);
#endif
        }

        /// <summary>
        /// Internal: builds faces only where no Morton-neighbor exists (uniform cell size required).
        /// </summary>
        /// <param name="origins">Voxel origins</param>
        /// <param name="cell">Uniform voxel size</param>
        /// <param name="cordSystem">Coordinate system for winding</param>
        /// <param name="vertices">Output vertex buffer (appended)</param>
        /// <param name="faces">Output index buffer (appended)</param>
        /// <remarks>
        /// Uses the minimum-corner quantization origin for grid indexing.
        /// </remarks>
        private static void CullWithMorton(
            ReadOnlySpan<Vector3> origins,
            Vector3 cell,
            CordSystem cordSystem,
            List<Vector3> vertices,
            List<TriFace> faces)
        {
            // quantization origin (min corner)
            Vector3 originMorton = origins[0];
            Vector3 inv = new(1f / cell.X, 1f / cell.Y, 1f / cell.Z);

            // build occupancy
            HashSet<ulong> occupied = new(origins.Length);
            foreach (Vector3 v in origins)
            {
                (int ix, int iy, int iz) = ToIdx(v, originMorton, inv);
                occupied.Add(Morton.Encode((uint)ix, (uint)iy, (uint)iz));
            }

            // neighbor offsets
            ReadOnlySpan<(int dx, int dy, int dz, Vector3 dir)> N6 = stackalloc[]
            {
                ( 1, 0, 0, new Vector3( 1, 0, 0)),
                (-1, 0, 0, new Vector3(-1, 0, 0)),
                ( 0, 1, 0, new Vector3( 0, 1, 0)),
                ( 0,-1, 0, new Vector3( 0,-1, 0)),
                ( 0, 0, 1, new Vector3( 0, 0, 1)),
                ( 0, 0,-1, new Vector3( 0, 0,-1)),
            };

            // emit faces
            foreach (Vector3 v in origins)
            {
                (int ix, int iy, int iz) = ToIdx(v, originMorton, inv);

                foreach ((int dx, int dy, int dz, Vector3 dir) n in N6)
                {
                    ulong keyN = Morton.Encode(
                        (uint)(ix + n.dx), (uint)(iy + n.dy), (uint)(iz + n.dz));

                    if (occupied.Contains(keyN)) continue;
                    VoxelFaceBuilder.MakeFace(v, n.dir, vertices, faces, cell, cordSystem);
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static (int ix, int iy, int iz) ToIdx(Vector3 p, Vector3 min, Vector3 inv)
        {
            int ix = (int)MathFx.Floor((p.X - min.X) * inv.X);
            int iy = (int)MathFx.Floor((p.Y - min.Y) * inv.Y);
            int iz = (int)MathFx.Floor((p.Z - min.Z) * inv.Z);
            return (ix, iy, iz);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static MeshF MakeMesh(List<Vector3> vertices, List<TriFace> faces)
        {
            if (vertices.Count == 0 || faces.Count == 0)
                throw new InvalidOperationException("Generated mesh has no vertices or faces.");
            return new MeshF(vertices.ToArray(), faces.ToArray());
        }
    }
}
