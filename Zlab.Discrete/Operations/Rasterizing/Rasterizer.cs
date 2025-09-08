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
        public static List<OccupancyVoxel> RasterizeFace(MeshF mesh, TriFace face, Vector3 voxelSize)
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
            List<OccupancyVoxel> voxels = new(Math.Max(8, cap));

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

                        if (BBoxIntersection.TriangleIntersectsAabb(v0, v1, v2, box))
                        {
                            voxels.Add(new OccupancyVoxel(
                                GridConverter.IndexToMinCorner(x, y, z, voxelSize),
                                Occupancy.Boundary));
                        }
                    }
                }
            }
            return voxels;
        }

        public static void RasterizeFaceInGrid(OccupancyGrid grid, MeshF mesh, TriFace face)
        {
            face.QueryVertices(mesh.Vertices, out Vector3 v0, out Vector3 v1, out Vector3 v2);

            Vector3 vs = grid.Meta.VoxelSize;

            // Triangle AABB
            BBox tri = new();
            tri.Expand(v0); tri.Expand(v1); tri.Expand(v2);

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

                        if (BBoxIntersection.TriangleIntersectsAabb(v0, v1, v2, box))
                        {
                            grid.SetValue((x, y, z), Occupancy.Boundary);
                        }
                    }
                }
            }
        }
    }
}
