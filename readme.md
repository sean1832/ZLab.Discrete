# Zlab.Discrete
Library for voxelization, meshing, and distance fields in managed C#.

## Overview
Zlab.Discrete is a fully managed C# library that provides a collection of voxelization algorithms for 3D meshes. It focuses on ease of use while offering flexibility through different voxelization techniques.
It is not as performant as some native libraries, but it is easier to integrate into C# projects and it tries its best to be performant for a managed library.

## Features
- Rasterization of 3D meshes using Bounding Box intersection
- Anisiotropic Voxelization (non-uniform voxel sizes)
- Dense occupancy grid representation
- Dense distance grid representation (Euclidean distance transform)
- Signed Distance Field (SDF) generation
- Meshing using face-culling algorithms
- Flood fill algorithms for interior voxel identification

## Frameworks
- .NET Framework 4.7.2
- .NET Framework 4.8
- .NET Standard 2.1
- .NET 7.0
- .NET 8.0
- .NET 9.0

## Usage
```csharp
using System;
using System.Collections.Generic;
using System.Numerics;
using ZLab.Discrete.Geometry;
using ZLab.Discrete.Grids;
using ZLab.Discrete.Operations.Meshing;
using ZLab.Discrete.Operations.Rasterizing;
using ZLab.Discrete.Voxels;

public class Example
{
    public static void Main()
    {
        // ---------------------------------------------------------------------
        // INPUT GEOMETRY (replace with your own data)
        // ---------------------------------------------------------------------
        // One triangle in XY plane (z = 0)
        Vector3[] vertices =
        {
            new Vector3(2, 2, 0),
            new Vector3(7, 2, 0),
            new Vector3(4, 6, 0),
        };
        TriFace[] faces = { new TriFace(0, 1, 2) };
        MeshF mesh = new MeshF(vertices, faces);

        // ---------------------------------------------------------------------
        // VOXELIZATION DOMAIN
        // ---------------------------------------------------------------------
        BBox bounds = new BBox(min: new Vector3(0, 0, -1), max: new Vector3(10, 10, 1));
        Vector3 voxelSize = new Vector3(1, 1, 1); // uniform cubic voxels
        OccupancyGrid grid = new OccupancyGrid(voxelSize, bounds);

        // ---------------------------------------------------------------------
        // RASTERIZE (MUTATES GRID)
        // ---------------------------------------------------------------------
        // Populates grid with Outside / Boundary / Inside.
        DenseRasterizer.Rasterize(grid, mesh, floodFill: true); // <- grid is mutated

        // ---------------------------------------------------------------------
        // SELECT VOXELS (here: Boundary only)
        // ---------------------------------------------------------------------
        List<Vector3> voxelOrigins = new();
        foreach ((Vector3 origin, Occupancy state) in grid.EnumerateVoxels())
        {
            if (state == Occupancy.Boundary)  // <- change this to select different states
                voxelOrigins.Add(origin);
        }

        // ---------------------------------------------------------------------
        // MESH EXTRACTION
        // ---------------------------------------------------------------------
        // a. Greedy/culled surface from occupied cells
        MeshF voxelMesh = VoxelMesher.GenerateMesh(voxelOrigins, voxelSize);

        // b. Optional: one cube per occupied voxel (useful for debugging)
        MeshF[] cubes = VoxelMesher.GenerateMeshes(voxelOrigins, voxelSize);

        // ---------------------------------------------------------------------
        // TERNARY MASK & SIGNED DISTANCE FIELD (SDF)
        // ---------------------------------------------------------------------
        // 0 = Outside, 1 = Inside, 2 = Boundary
        byte[] mask = grid.GetMaskTernary(flipSign: false);

        DistanceGrid distanceGrid = new DistanceGrid(grid); // shares geometry (dims/origin/voxel size)
        distanceGrid.BuildFromTernaryMask(mask, parallel: true);

        // ---------------------------------------------------------------------
        // SAMPLING
        // ---------------------------------------------------------------------
        float dCenter = distanceGrid.GetValue((5, 5, 0)); // grid-space integer index
        ReadOnlySpan<float> allDistances = distanceGrid.GetValues();
    }
}
```