using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using ZLab.Discrete.Algorithms.Encoding;
using ZLab.Discrete.Geometry;

namespace ZLab.Discrete.Operations.Meshing
{
    /// <summary>
    /// Naive voxel meshing utilities: emits per-voxel meshes or a single merged mesh.
    /// Uses face culling with Morton-coded occupancy when voxel size is uniform.
    /// </summary>
    /// <remarks>
    /// Assumes voxels instances lie on a rectilinear grid.
    /// For the culled path, all voxels must share an identical <c>voxelSize</c>.
    /// If voxel sizes vary, all six faces per voxel are emitted (no merging).
    /// </remarks>
    public static class DiscreteMesher
    {
        /// <summary>
        /// Gets axis-aligned bounding boxes for a set of voxels.
        /// </summary>
        /// <param name="origins">Voxel origins</param>
        /// <param name="voxelSizes">Per-voxel sizes; must match <paramref name="origins"/> length.</param>
        /// <param name="outBoxes">Pre-allocated output span to write bounding boxes into; must match <paramref name="origins"/> length.</param>
        /// <exception cref="ArgumentException">voxelSizes length must match voxels length.</exception>
        /// <remarks>
        /// This variant avoids allocations by writing into a pre-allocated output <see cref="Span{T}"/>.
        /// </remarks>
        public static void GetVoxelBounds(ReadOnlySpan<Vector3> origins, ReadOnlySpan<Vector3> voxelSizes, Span<BBox> outBoxes)
        {
            if (voxelSizes.Length != origins.Length)
                throw new ArgumentException("voxelSizes length must match voxels length.");
            if (outBoxes.Length < origins.Length)
                throw new ArgumentException("Output span is too small to hold all bounding boxes.", nameof(outBoxes));
            for (int i = 0; i < origins.Length; i++)
            {
                outBoxes[i] = new BBox(origins[i], origins[i] + voxelSizes[i]);
            }
        }

        /// <summary>
        /// Gets axis-aligned bounding boxes for a set of voxels.
        /// </summary>
        /// <param name="origins">Voxel origins</param>
        /// <param name="voxelSizes">Per-voxel sizes; must match <paramref name="origins"/> length.</param>
        /// <returns>Array of bounding boxes, one per input voxel.</returns>
        /// <exception cref="ArgumentException">voxelSizes length must match voxels length.</exception>
        /// <remarks>
        /// This variant allocates a <b>new array</b> for the output bounding boxes.
        /// </remarks>
        public static BBox[] GetVoxelBounds(ReadOnlySpan<Vector3> origins, ReadOnlySpan<Vector3> voxelSizes)
        {
            if (voxelSizes.Length != origins.Length)
                throw new ArgumentException("voxelSizes length must match voxels length.");
            if (origins.Length == 0) return Array.Empty<BBox>();
            BBox[] boxes = new BBox[origins.Length];
            GetVoxelBounds(origins, voxelSizes, boxes);
            return boxes;
        }

        /// <summary>
        /// Gets axis-aligned bounding boxes for a set of voxels (all voxels share the same <paramref name="voxelSize"/>).
        /// </summary>
        /// <param name="origins">Voxel origins</param>
        /// <param name="voxelSize">Per-voxel sizes</param>
        /// <param name="outBoxes">Pre-allocated output span to write bounding boxes into; must match <paramref name="origins"/> length.</param>
        /// <exception cref="ArgumentException">Output span is too small to hold all bounding boxes.</exception>
        /// <remarks>
        /// This variant avoids allocations by writing into a pre-allocated output <see cref="Span{T}"/>.
        /// </remarks>
        public static void GetVoxelBounds(ReadOnlySpan<Vector3> origins, Vector3 voxelSize, Span<BBox> outBoxes)
        {
            if (outBoxes.Length < origins.Length)
                throw new ArgumentException("Output span is too small to hold all bounding boxes.", nameof(outBoxes));
            for (int i = 0; i < origins.Length; i++)
            {
                Vector3 origin = origins[i];
                outBoxes[i] = new BBox(origin, origin + voxelSize);
            }
        }

        /// <summary>
        /// Gets axis-aligned bounding boxes for a set of voxels (all voxels share the same <paramref name="voxelSize"/>).
        /// </summary>
        /// <param name="origins">Voxel origins</param>
        /// <param name="voxelSize">Per-voxel sizes</param>
        /// <returns>Array of bounding boxes, one per input voxel.</returns>
        /// <remarks>
        /// This variant allocates a <b>new array</b> for the output bounding boxes.
        /// </remarks>
        public static BBox[] GetVoxelBounds(ReadOnlySpan<Vector3> origins, Vector3 voxelSize)
        {
            if (origins.Length == 0) return Array.Empty<BBox>();
            BBox[] boxes = new BBox[origins.Length];
            GetVoxelBounds(origins, voxelSize, boxes);
            return boxes;
        }


        /// <summary>
        /// Generates an individual quad-mesh per voxel (no face merging across voxels).
        /// </summary>
        /// <param name="origins">Voxel origins</param>
        /// <param name="voxelSizes">Per-voxel sizes; must match <paramref name="origins"/> length.</param>
        /// <param name="outMeshes">Pre-allocated output span to write meshes into; must match <paramref name="origins"/> length.</param>
        /// <param name="cordSystem">Coordinate system for winding (right- or left-handed).</param>
        /// <exception cref="ArgumentException">Lengths of <paramref name="origins"/> and <paramref name="voxelSizes"/> differ.</exception>
        /// <remarks>
        /// This variant avoids allocations by writing into a pre-allocated output <see cref="Span{T}"/>.
        /// </remarks>
        public static void GenerateMeshes(ReadOnlySpan<Vector3> origins, ReadOnlySpan<Vector3> voxelSizes, Span<MeshF> outMeshes, CordSystem cordSystem = CordSystem.RightHanded)
        {
            if (voxelSizes.Length != origins.Length)
                throw new ArgumentException("voxelSizes length must match voxels length.");
            if (outMeshes.Length < origins.Length)
                throw new ArgumentException("Output span is too small to hold all meshes.", nameof(outMeshes));
            for (int i = 0; i < origins.Length; i++)
            {
                Vector3 origin = origins[i];
                Vector3 size = voxelSizes[i];
                outMeshes[i] = new BBox(origin, origin + size).ToMesh(cordSystem);
            }
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
        /// <b>Memory-heavy:</b> each voxel becomes a standalone box mesh (24 verts, 12 tris).
        /// 
        /// This variant allocates a <b>new array</b> for the output meshes.
        /// </remarks>
        public static MeshF[] GenerateMeshes(
            ReadOnlySpan<Vector3> origins, ReadOnlySpan<Vector3> voxelSizes, CordSystem cordSystem = CordSystem.RightHanded)
        {
            if (voxelSizes.Length != origins.Length)
                throw new ArgumentException("voxelSizes length must match voxels length.");
            if (origins.Length == 0) return Array.Empty<MeshF>();
            MeshF[] meshes = new MeshF[origins.Length];
            GenerateMeshes(origins, voxelSizes, meshes, cordSystem);
            return meshes;
        }

        /// <summary>
        /// Generates an individual quad-mesh per voxel (all voxels share the same <paramref name="voxelSize"/>).
        /// </summary>
        /// <param name="origins">Voxel origins</param>
        /// <param name="voxelSize">Uniform voxel size for all voxels.</param>
        /// <param name="outMeshes">Pre-allocated output span to write meshes into; must match <paramref name="origins"/> length.</param>
        /// <param name="cord">Coordinate system for winding (right- or left-handed).</param>
        /// <exception cref="ArgumentException">Output span is too small to hold all meshes.</exception>
        /// <remarks>
        /// This variant avoids allocations by writing into a pre-allocated output <see cref="Span{T}"/>.
        /// </remarks>
        public static void GenerateMeshes(
            ReadOnlySpan<Vector3> origins, Vector3 voxelSize, Span<MeshF> outMeshes,
            CordSystem cord = CordSystem.RightHanded)
        {
            if (outMeshes.Length < origins.Length)
                throw new ArgumentException("Output span is too small to hold all meshes.", nameof(outMeshes));
            for (int i = 0; i < origins.Length; i++)
                outMeshes[i] = new BBox(origins[i], origins[i] + voxelSize).ToMesh(cord);
        }

        /// <summary>
        /// Generates an individual quad-mesh per voxel (all voxels share the same <paramref name="voxelSize"/>).
        /// </summary>
        /// <param name="origins">Voxel origins</param>
        /// <param name="voxelSize">Uniform voxel size for all voxels.</param>
        /// <param name="cordSystem">Coordinate system for winding (right- or left-handed).</param>
        /// <returns>Array of meshes, one per input voxel.</returns>
        /// <remarks>
        /// <b>Memory-heavy:</b> each voxel becomes a standalone box mesh (24 verts, 12 tris). This variant allocates a <b>new array</b> for the output meshes.
        /// </remarks>
        public static MeshF[] GenerateMeshes(
            ReadOnlySpan<Vector3> origins,
            Vector3 voxelSize,
            CordSystem cordSystem = CordSystem.RightHanded)
        {
            if (origins.Length == 0) return Array.Empty<MeshF>();
            MeshF[] meshes = new MeshF[origins.Length];
            GenerateMeshes(origins, voxelSize, meshes, cordSystem);
            return meshes;
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
        /// <b>Origin convention:</b> the culled path treats origin of voxels as the <i>minimum corner</i> of the cell.
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
                    foreach (Vector3 dir in DiscreteFaceBuilder.Directions)
                    {
                        DiscreteFaceBuilder.MakeFace(originUniform, dir, vertices, faces, size, cordSystem);
                    }
                }
            }
            return MakeMesh(vertices, faces);
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
        /// <b>Origin convention:</b> the culled path treats origin of voxels as the minimum corner of the cell.
        /// </remarks>
        public static MeshF GenerateMesh(
            ReadOnlySpan<Vector3> origins,
            Vector3 voxelSize,
            CordSystem cordSystem = CordSystem.RightHanded)
        {
            if (origins.Length == 0)
                return new MeshF();

            List<Vector3> vertices = new(origins.Length * 24);
            List<TriFace> faces = new(origins.Length * 12);

            // direct to culled path (since it’s uniform)
            CullWithMorton(origins, voxelSize, cordSystem, vertices, faces);
            return MakeMesh(vertices, faces);
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
            // compute true min-corner & reciprocals
            var (minCorner, inv) = ComputeQuantization(origins, cell);

            // occupancy set (capacity hint)
#if NET48_OR_GREATER || NETSTANDARD2_1_OR_GREATER || NETCOREAPP3_1_OR_GREATER
            HashSet<ulong> occupied = new(capacity: origins.Length);
#else
            HashSet<ulong> occupied = new HashSet<ulong>(); // no capacity ctor on older TFMs
#endif

            // build occupancy
            for (int i = 0; i < origins.Length; i++)
            {
                var (ix, iy, iz) = ToIdx(origins[i], minCorner, inv);
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

            // emit faces only where no Morton-neighbor exists
            for (int i = 0; i < origins.Length; i++)
            {
                Vector3 v = origins[i];
                (int ix, int iy, int iz) = ToIdx(v, minCorner, inv);

                foreach ((int dx, int dy, int dz, Vector3 dir) n in N6)
                {
                    int nx = ix + n.dx, ny = iy + n.dy, nz = iz + n.dz;
                    // Out-of-range neighbors should be treated as empty (face visible).
                    // We can skip explicit bounds checks since we’re using a set; just avoid uint underflow semantics confusion:
                    if (nx < 0 || ny < 0 || nz < 0)
                    {
                        DiscreteFaceBuilder.MakeFace(v, n.dir, vertices, faces, cell, cordSystem);
                        continue;
                    }

                    ulong keyN = Morton.Encode((uint)nx, (uint)ny, (uint)nz);
                    if (!occupied.Contains(keyN))
                        DiscreteFaceBuilder.MakeFace(v, n.dir, vertices, faces, cell, cordSystem);
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static (Vector3 min, Vector3 inv) ComputeQuantization(ReadOnlySpan<Vector3> origins, in Vector3 cell)
        {
            // true component-wise minimum across all voxel origins
            Vector3 min = origins[0];
            for (int i = 1; i < origins.Length; i++)
            {
                Vector3 v = origins[i];
                if (v.X < min.X) min.X = v.X;
                if (v.Y < min.Y) min.Y = v.Y;
                if (v.Z < min.Z) min.Z = v.Z;
            }

            // precompute reciprocals
            Vector3 inv = new(1f / cell.X, 1f / cell.Y, 1f / cell.Z);
            return (min, inv);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static (int ix, int iy, int iz) ToIdx(Vector3 p, Vector3 min, Vector3 inv)
        {
            const float eps = 1e-6f;
            float fx = (p.X - min.X) * inv.X;
            float fy = (p.Y - min.Y) * inv.Y;
            float fz = (p.Z - min.Z) * inv.Z;
            int ix = (int)MathF.Round(fx + (fx >= 0 ? eps : -eps));
            int iy = (int)MathF.Round(fy + (fy >= 0 ? eps : -eps));
            int iz = (int)MathF.Round(fz + (fz >= 0 ? eps : -eps));
            return (ix, iy, iz);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static MeshF MakeMesh(List<Vector3> vertices, List<TriFace> faces)
        {
            if (vertices.Count == 0 || faces.Count == 0)
                throw new InvalidOperationException("Generated mesh has no vertices or faces.");
            return new MeshF(vertices.ToArray(), faces.ToArray(), isClosed: true); // <- always closed (avoid watertight check)
        }
    }
}
