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
### Mesh building
```csharp
// Triangles only, indices must be valid for verts[]
Vector3[] verts = { /* ... */ };
TriFace[] faces = { new TriFace(a,b,c), /* ... */ };
MeshF mesh = new MeshF(verts, faces);
```
If you are converting from another mesh format, and you know the mesh is closed (watertight), you can construct mesh with:
```csharp
MeshF mesh = new MeshF(verts, faces, isClosed:true);
```
to skip the interal watertight check, which can be slow for large meshes.


### Define dense voxel grid region
```csharp
// Axis-aligned bounds and uniform/non-uniform voxel size
BBox bounds = new(min: new Vector3(0,0,0), max: new Vector3(10,10,10));
Vector3 size = new(0.5f, 1f, 0.5f);
OccupancyGrid grid = new(size, bounds);
```

### Rasterization
#### Dense rasterization
This will result in a dense occupancy grid where each voxel is classified as `Inside`, `Outside`, or `Boundary`.
```csharp
// Flood fill classifies interior; requires a closed mesh
DenseRasterizer.Rasterize(grid, mesh, floodFill: true);
```
- `grid` will be mutated.
- `floodFill` is optional and defaults to `false`. It is recommended to set it to `true` if the mesh is closed (watertight) to correctly identify interior voxels. This is a post-processing step that fills in the interior of the mesh after rasterization.

#### Sparse rasterization
This will only return a list of voxel origins that are classified as `Boundary`.
```csharp
List<Vector3> voxelOrigins = SparseRasterizer.Rasterize(Mesh, _voxelSize);
```


### Enumerate voxels inside the grid
#### Pass as `List<Vector3>`
```csharp
List<Vector3> voxelOrigins = new();
foreach ((Vector3 origin, Occupancy state) in Grid.EnumerateVoxels())
{
    // Outside | Inside | Boundary
    if (state == Occupancy.Boundary)  // <- change this to select different states
        voxelOrigins.Add(origin);
}
```

#### Pass as `ReadOnlySpan`
```csharp
long boundaryCount = grid.CountState(Occupancy.Boundary);
Vector3[] buffer = new[boundaryCount];
int i = 0;
foreach ((Vector3 origin, Occupancy state) in grid.EnumerateVoxels())
{
    if (state == Occupancy.Boundary)  // <- change this to select different states
        buffer[i++] = origin;
}
// Trim to actual size, avoid extra allocations
ReadOnlySpan<Vector3> voxelOrigins = buffer.AsSpan(0, i);
```

### Mesh Extraction
#### Face-culling meshing
This will result a single mesh with internal faces removed.
```csharp
MeshF mesh = VoxelMesher.GenerateMesh(voxelOrigins, size);
```

#### Naive meshing
This will result individual mesh for each voxel.
```csharp
MeshF[] meshes = VoxelMesher.GenerateMeshes(voxelOrigins, size);
```

### Grid Masking
This will result a byte array where each voxel is classified as `0: Outside`, `1: Inside`, or `2: Boundary`.
```csharp
byte[] mask = grid.GetMaskTernary(); // 0: Outside, 1: Inside, 2: Boundary
```

### Distance Field
This will result a dense distance grid where each voxel stores the Euclidean distance to the nearest surface voxel.
```csharp
DistanceGrid distGrid = new(grid); // Copy grid structure from occupancy grid
distGrid.BuildFromTernaryMask(mask, parallel:true); // Build distance field from ternary mask
```
#### Distance queries
```csharp
float distance = distGrid.GetValue(new Vector3(1,2,1)); // Get distance at a specific point (must be inside the grid bounds)
ReadOnlySpan<float> allDistances = distGrid.GetValues(); // Get all distances as a flat array
```
