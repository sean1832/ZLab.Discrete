using System;
using System.Collections.Generic;
using System.Numerics;
using ZLab.Discrete.Algorithms.Collision;
using ZLab.Discrete.Geometry;
using ZLab.Discrete.Grids;
using ZLab.Discrete.Voxels;

namespace ZLab.Discrete.Operations.Rasterizing
{
    internal static class Rasterizer
    {
        public static List<Vector3> RasterizeFaceSparse(MeshF mesh, TriFace face, Vector3 voxelSize)
        {
            // Get the triangle's vertices
            face.QueryVertices(mesh.Vertices, out Vector3 v0, out Vector3 v1, out Vector3 v2);

            // Compute the AABB of the triangle
            BBox tri = new();
            tri.Expand(v0);
            tri.Expand(v1);
            tri.Expand(v2);

            // Integer voxel AABB (min inclusive, max inclusive)
            (int gx0, int gy0, int gz0) = GridConverter.WorldToGridMin(tri.Min, voxelSize);
            (int gx1, int gy1, int gz1) = GridConverter.WorldToGridMaxInclusive(tri.Max, voxelSize);

            int nx = gx1 - gx0 + 1;
            int ny = gy1 - gy0 + 1;
            int nz = gz1 - gz0 + 1;

            int cap = Math.Max(0, nx * ny * nz / 4); // heuristic
            List<Vector3> voxels = new(Math.Max(8, cap));

            // One reusable BBox to cut allocs
            BBox box = default;

            for (int z = gz0; z <= gz1; z++)
            {
                float z0 = z * voxelSize.Z, z1 = z0 + voxelSize.Z;
                for (int y = gy0; y <= gy1; y++)
                {
                    float y0 = y * voxelSize.Y, y1 = y0 + voxelSize.Y;
                    for (int x = gx0; x <= gx1; x++)
                    {
                        float x0 = x * voxelSize.X, x1 = x0 + voxelSize.X;

                        box.Min = new Vector3(x0, y0, z0);
                        box.Max = new Vector3(x1, y1, z1);

                        if (box.IsIntersectsTriangle(v0, v1, v2) ||
                            box.IsCoveredByTriangle(v0, v1, v2))
                        {
                            // intersects, add voxel min corner (world space origin)
                            voxels.Add(GridConverter.IndexToMinCorner(x, y, z, voxelSize));
                        }
                    }
                }
            }

            return voxels;
        }

        public static Vector3[] RasterizePolylineSparse(PolylineF polyline, Vector3 voxelSize,
            bool includeClosingEdge = true)
        {
            ReadOnlySpan<Vector3> pts = polyline.Vertices;
            int count = polyline.Count;
            if (count < 2) return Array.Empty<Vector3>();

            // Capacity guess: total length / min(voxelSize) * 2 (rough)
            float minVs = MathF.Min(voxelSize.X, MathF.Min(voxelSize.Y, voxelSize.Z));
            float approxLen = polyline.Length;
            int cap = Math.Max(16, (int)(approxLen / MathF.Max(minVs, 1e-6f)) * 2);

            HashSet<(int x, int y, int z)> visited = new(cap);

            int last = count - 1;
            for (int i = 0; i < last; i++)
                AddSegmentSparse(pts[i], pts[i + 1], voxelSize, visited);

            if (includeClosingEdge && polyline.IsClosed && count > 2)
                AddSegmentSparse(pts[last], pts[0], voxelSize, visited);

            // Convert grid indices to world-space voxel minima
            Vector3[] result = new Vector3[visited.Count];
            int k = 0;
            foreach ((int x, int y, int z) in visited)
                result[k++] = new Vector3(x * voxelSize.X, y * voxelSize.Y, z * voxelSize.Z);

            return result;
        }

        public static void RasterizePolylineInGrid(OccupancyGrid grid, PolylineF polyline)
        {
            ReadOnlySpan<Vector3> pts = polyline.Vertices;
            int n = polyline.Count;
            if (n < 2) return;

            Vector3 vs = grid.Meta.VoxelSize;
            float sx = vs.X, sy = vs.Y, sz = vs.Z;

            int gridMinX = grid.Meta.MinX, gridMinY = grid.Meta.MinY, gridMinZ = grid.Meta.MinZ;
            int gridMaxX = grid.Meta.MinX + grid.Meta.Nx - 1;
            int gridMaxY = grid.Meta.MinY + grid.Meta.Ny - 1;
            int gridMaxZ = grid.Meta.MinZ + grid.Meta.Nz - 1;

            // Whole-grid AABB in world space (origin assumed 0; mirror mesh path)
            BBox gridBox = new(
                new Vector3(gridMinX * sx, gridMinY * sy, gridMinZ * sz),
                new Vector3((gridMaxX + 1) * sx, (gridMaxY + 1) * sy, (gridMaxZ + 1) * sz)
            );

            // Reusable voxel box
            BBox voxelBox = default;

            // Process open chain
            for (int i = 0; i < n - 1; i++)
            {
                Vector3 p0 = pts[i], p1 = pts[i + 1];
                if ((p1 - p0).LengthSquared() == 0f) continue; // degenerate

                // Optional: quick reject against grid bounds
                if (!gridBox.IsIntersectsSegment(p0, p1)) continue;

                RasterizeSegment(grid, p0, p1, vs, gridMinX, gridMinY, gridMinZ,
                    gridMaxX, gridMaxY, gridMaxZ, ref voxelBox);
            }

            // Close the loop if needed
            if (polyline.IsClosed && n > 2)
            {
                Vector3 p0 = pts[n - 1], p1 = pts[0];
                if ((p1 - p0).LengthSquared() != 0f && gridBox.IsIntersectsSegment(p0, p1))
                {
                    RasterizeSegment(grid, p0, p1, vs, gridMinX, gridMinY, gridMinZ,
                        gridMaxX, gridMaxY, gridMaxZ, ref voxelBox);
                }
            }
        }

        public static void RasterizeFaceInGrid(OccupancyGrid grid, MeshF mesh, TriFace face)
        {
            face.QueryVertices(mesh.Vertices, out Vector3 v0, out Vector3 v1, out Vector3 v2);

            Vector3 vs = grid.Meta.VoxelSize;

            // Triangle AABB
            BBox tri = new();
            tri.Expand(v0);
            tri.Expand(v1);
            tri.Expand(v2);

            // Integer AABB in grid coords (then clamp to grid)
            (int gx0, int gy0, int gz0) = GridConverter.WorldToGridMin(tri.Min, vs);
            (int gx1, int gy1, int gz1) = GridConverter.WorldToGridMaxInclusive(tri.Max, vs);

            int minX = Math.Max(gx0, grid.Meta.MinX);
            int minY = Math.Max(gy0, grid.Meta.MinY);
            int minZ = Math.Max(gz0, grid.Meta.MinZ);
            int maxX = Math.Min(gx1, grid.Meta.MinX + grid.Meta.Nx - 1);
            int maxY = Math.Min(gy1, grid.Meta.MinY + grid.Meta.Ny - 1);
            int maxZ = Math.Min(gz1, grid.Meta.MinZ + grid.Meta.Nz - 1);

            if (minX > maxX || minY > maxY || minZ > maxZ) return;

            BBox box = default;

            for (int z = minZ; z <= maxZ; z++)
            {
                float z0 = z * vs.Z, z1 = z0 + vs.Z;
                for (int y = minY; y <= maxY; y++)
                {
                    float y0 = y * vs.Y, y1 = y0 + vs.Y;
                    for (int x = minX; x <= maxX; x++)
                    {
                        float x0 = x * vs.X, x1 = x0 + vs.X;

                        box.Min = new Vector3(x0, y0, z0);
                        box.Max = new Vector3(x1, y1, z1);

                        if (box.IsIntersectsTriangle(v0, v1, v2) ||
                            box.IsCoveredByTriangle(v0, v1, v2))
                        {
                            grid.SetValue((x, y, z), Occupancy.Boundary);
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Sets all voxels touched by the segment [p0,p1] in the occupancy grid to Boundary.
        /// </summary>
        private static void RasterizeSegment(
            OccupancyGrid grid, in Vector3 p0, in Vector3 p1, in Vector3 vs,
            int gridMinX, int gridMinY, int gridMinZ,
            int gridMaxX, int gridMaxY, int gridMaxZ,
            ref BBox voxelBox)
        {
            float sx = vs.X, sy = vs.Y, sz = vs.Z;

            // Segment AABB (world)
            Vector3 segMin = new(MathF.Min(p0.X, p1.X), MathF.Min(p0.Y, p1.Y), MathF.Min(p0.Z, p1.Z));
            Vector3 segMax = new(MathF.Max(p0.X, p1.X), MathF.Max(p0.Y, p1.Y), MathF.Max(p0.Z, p1.Z));

            // Grid AABB (inclusive indices), clamp to grid
            (int gx0, int gy0, int gz0) = GridConverter.WorldToGridMin(segMin, vs);
            (int gx1, int gy1, int gz1) = GridConverter.WorldToGridMaxInclusive(segMax, vs);

            int minX = Math.Max(gx0, gridMinX);
            int minY = Math.Max(gy0, gridMinY);
            int minZ = Math.Max(gz0, gridMinZ);
            int maxX = Math.Min(gx1, gridMaxX);
            int maxY = Math.Min(gy1, gridMaxY);
            int maxZ = Math.Min(gz1, gridMaxZ);
            if (minX > maxX || minY > maxY || minZ > maxZ) return;

            for (int z = minZ; z <= maxZ; z++)
            {
                float z0 = z * sz, z1 = z0 + sz;
                for (int y = minY; y <= maxY; y++)
                {
                    float y0 = y * sy, y1 = y0 + sy;
                    for (int x = minX; x <= maxX; x++)
                    {
                        float x0 = x * sx, x1 = x0 + sx;

                        voxelBox.Min = new Vector3(x0, y0, z0);
                        voxelBox.Max = new Vector3(x1, y1, z1);

                        if (voxelBox.IsIntersectsSegment(p0, p1))
                            grid.SetValue((x, y, z), Occupancy.Boundary);
                    }
                }
            }
        }

        /// <summary>
        /// Adds all voxel indices touched by the segment [p0,p1] to outSet.
        /// </summary>
        private static void AddSegmentSparse(in Vector3 p0, in Vector3 p1, in Vector3 vs, HashSet<(int x, int y, int z)> outSet)
        {
            // Direction and early-out for degenerate segments
            Vector3 dir = p1 - p0;
            float lenSq = dir.LengthSquared();
            if (lenSq == 0f)
            {
                (int x, int y, int z) c = GridConverter.WorldToGridMin(p0, vs);
                outSet.Add(c);
                return;
            }

            // Parametric t in [0,1], start voxel indices
            // We clip traversal to the segment's grid AABB to keep the loop finite/stable.
            Vector3 aabbMin = new Vector3(MathF.Min(p0.X, p1.X), MathF.Min(p0.Y, p1.Y), MathF.Min(p0.Z, p1.Z));
            Vector3 aabbMax = new Vector3(MathF.Max(p0.X, p1.X), MathF.Max(p0.Y, p1.Y), MathF.Max(p0.Z, p1.Z));
            (int ixMin, int iyMin, int izMin) = GridConverter.WorldToGridMin(aabbMin, vs);
            (int ixMax, int iyMax, int izMax) = GridConverter.WorldToGridMaxInclusive(aabbMax, vs);

            // Start at the voxel containing p0 (clamped to the AABB range)
            (int ix, int iy, int iz) = GridConverter.WorldToGridMin(p0, vs);
            ix = Math.Clamp(ix, ixMin, ixMax);
            iy = Math.Clamp(iy, iyMin, iyMax);
            iz = Math.Clamp(iz, izMin, izMax);

            // Step and boundary setup per axis
            int stepX = dir.X > 0 ? 1 : (dir.X < 0 ? -1 : 0);
            int stepY = dir.Y > 0 ? 1 : (dir.Y < 0 ? -1 : 0);
            int stepZ = dir.Z > 0 ? 1 : (dir.Z < 0 ? -1 : 0);

            // Next boundary positions in world
            float nextBoundaryX = stepX >= 0 ? (ix + 1) * vs.X : ix * vs.X;
            float nextBoundaryY = stepY >= 0 ? (iy + 1) * vs.Y : iy * vs.Y;
            float nextBoundaryZ = stepZ >= 0 ? (iz + 1) * vs.Z : iz * vs.Z;

            // t at which we hit the next boundary on each axis (and how much t to advance per voxel step)
            const float Tiny = 1e-20f; // avoids division by zero
            float invDx = 1f / (MathF.Abs(dir.X) + Tiny);
            float invDy = 1f / (MathF.Abs(dir.Y) + Tiny);
            float invDz = 1f / (MathF.Abs(dir.Z) + Tiny);

            float tMaxX = stepX != 0 ? (nextBoundaryX - p0.X) * (invDx * MathF.Sign(dir.X)) : float.PositiveInfinity;
            float tMaxY = stepY != 0 ? (nextBoundaryY - p0.Y) * (invDy * MathF.Sign(dir.Y)) : float.PositiveInfinity;
            float tMaxZ = stepZ != 0 ? (nextBoundaryZ - p0.Z) * (invDz * MathF.Sign(dir.Z)) : float.PositiveInfinity;

            float tDeltaX = stepX != 0 ? vs.X * invDx : float.PositiveInfinity;
            float tDeltaY = stepY != 0 ? vs.Y * invDy : float.PositiveInfinity;
            float tDeltaZ = stepZ != 0 ? vs.Z * invDz : float.PositiveInfinity;

            // March until we pass t=1 or leave the AABB range
            outSet.Add((ix, iy, iz));

            while (true)
            {
                if (tMaxX <= tMaxY && tMaxX <= tMaxZ)
                {
                    if (tMaxX > 1f) break;
                    ix += stepX;
                    if (ix < ixMin || ix > ixMax) break;
                    tMaxX += tDeltaX;
                }
                else if (tMaxY <= tMaxX && tMaxY <= tMaxZ)
                {
                    if (tMaxY > 1f) break;
                    iy += stepY;
                    if (iy < iyMin || iy > iyMax) break;
                    tMaxY += tDeltaY;
                }
                else
                {
                    if (tMaxZ > 1f) break;
                    iz += stepZ;
                    if (iz < izMin || iz > izMax) break;
                    tMaxZ += tDeltaZ;
                }

                outSet.Add((ix, iy, iz));
            }
        }
    }
}
