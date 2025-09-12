# Getting Started
This guide will help you get started with **Zlab.Discrete** in your .NET project.

## Installation
You can install the library via NuGet Package Manager or the .NET CLI.
```bash
dotnet add package ZLab.Discrete
```

## Basic Usage

### Sparse Voxelization and Meshing
Following is a simple example demonstrating how to perform sparse voxelization of a triangle mesh and generate a voxel mesh.
```csharp
using System.Numerics;
using ZLab.Discrete.Geometry;
using ZLab.Discrete.Operations.Meshing;
using ZLab.Discrete.Operations.Rasterizing;

public class Example
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

    // Voxel Spec
    Vector3 size = new Vector3(1, 0.5f, 1); // <- Anisotropic voxel size
    public void Sparse()
    {
        MeshF mesh = new MeshF(vertices, faces); // <- build input mesh
        Vector3[] voxelOrigins = SparseRasterizer.Rasterize(mesh, size);
        MeshF voxelMesh = DiscreteMesher.GenerateMesh(voxelOrigins, size);
    }
}
```
See:
- [Mesh Building](usages/mesh-building.html) for more details on mesh construction.
- [Sparse Voxelization](usages/rasterization.html#sparse-rasterization) for more details on sparse voxelization.

### Dense Voxelization and Signed Distance Field (SDF) Generation
The following example shows how to create a dense voxel grid from a triangle mesh and compute its signed distance field.
```csharp
public void ComputeSdf()
{
    MeshF mesh = new MeshF(vertices, faces); // <- Build the mesh
    BBox bounds = new BBox(min: new Vector3(0, 0, -1), max: new Vector3(10, 10, 1));

    OccupancyGrid oGrid = new(size, bounds);
    DistanceGrid dGrid = new(oGrid); // Create distance grid from occupancy grid

    byte[] mask = oGrid.GetMaskTernary(); // get ternary mask from occupancy grid
    dGrid.BuildFromTernaryMask(mask); // build distance field from the mask
}
```
See: 
- [Dense Occupancy Grid](usages/dense-grid-definition.html) for more details on dense voxel grids.
- [Signed Distance Field](usages/signed-distance-field.html) for more details on SDF generation.

### Sampling the SDF
You can sample the signed distance field at any point in space using trilinear interpolation.
```csharp
public void SampleSdf(DistanceGrid dGrid)
{
    Vector3 samplePoint = new Vector3(4.5f, 4.5f, 0);
    float sdfValue = dGrid.SampleTrilinear(samplePoint);
    Vector3 sdfGradient = dGrid.SampleGradient(samplePoint);
}
```
See: 
- [Signed Distance Field](usages/signed-distance-field.html#sample-signed-distance-field-sdf) for more details on SDF sampling.

### Extract voxels from Occupancy Grid
Extract voxel origins of a specific occupancy state (e.g., Boundary) from the occupancy grid.
```csharp
public void ExtractVoxels(OccupancyGrid oGrid)
{
    List<Vector3> origins = new();
    oGrid.ForEachVoxel((origin, state) =>
    {
        if (state == Occupancy.Boundary) // <- change this to select different states
            origins.Add(origin);
    });
}
```
See:
- [Enumerate Voxels](usages/dense-grid-definition.html) for more details on voxel enumeration.