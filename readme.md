# Zlab.Discrete
Library for voxelization, meshing, and distance fields in managed C#.

## Overview
Zlab.Discrete is a fully managed C# library that provides a collection of voxelization algorithms for 3D meshes. It focuses on ease of use while offering flexibility through different voxelization techniques.
It is not as performant as some native libraries, but it is easier to integrate into C# projects and it tries its best to be performant for a managed library.

## Features
- Anisotropic Voxelization (non-uniform voxel sizes)
- Dense occupancy grid representation
- Dense distance grid representation (Euclidean distance transform)
- Signed Distance Field (SDF) generation
- Sampling SDF with nearest neighbor or trilinear interpolation
- Gradient sampling from SDF
- Sparse voxelization (boundary voxels only)
- Meshing using face-culling algorithms
- BFS Flood fill algorithms for interior voxel identification

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
OccupancyGrid oGrid = new(size, bounds);
```

### Rasterization
#### Dense rasterization
This will result in a dense occupancy grid where each voxel is classified as `Inside`, `Outside`, or `Boundary`.
```csharp
// Flood fill classifies interior; requires a closed mesh
DenseRasterizer.Rasterize(oGrid, mesh, floodFill: true);
```
- `oGrid` will be mutated.
- `floodFill` is optional and defaults to `false`. It is recommended to set it to `true` if the mesh is closed (watertight) to correctly identify interior voxels. This is a post-processing step that fills in the interior of the mesh after rasterization.

#### Sparse rasterization
This will only return a list of voxel origins that are classified as `Boundary`.
```csharp
List<Vector3> voxelOrigins = SparseRasterizer.Rasterize(Mesh, _voxelSize);
```


### Enumerate voxels inside the grid
#### Single threaded
```csharp
List<Vector3> voxelOrigins = new();
oGrid.ForEachVoxel((origin, state) => {
    if (state == Occupancy.Boundary)  // <- change this to select different states
        voxelOrigins.Add(origin);
})
```
#### Multi-threaded
```csharp
ConcurrentBag<Vector3> voxelOrigins = new();
oGrid.ForEachVoxelParallel((origin, state) => {
    if (state == Occupancy.Boundary)  // <- change this to select different states
        voxelOrigins.Add(origin);
})
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

### Grid Masking & Distance Field Creation
```csharp
DistanceGrid dGrid = new(oGrid); // Create distance grid from occupancy grid
int count = (int)oGrid.Meta.Count; // Total number of voxels in the grid
byte[] buffer = ArrayPool<byte>.Shared.Rent(count); // Rent buffer from shared pool
try
{
    Span<byte> mask = buffer.AsSpan(0, count); // trim to actual size
    oGrid.GetMaskTernary(mask); // mask[i] = 0 (Outside), 1 (Inside), 2 (Boundary)
    dGrid.BuildFromTernaryMask(mask); // Build distance field from mask
}
finally
{
    ArrayPool<byte>.Shared.Return(buffer); // return buffer to pool
}
```

### Sample Signed Distance Field (SDF)
Sample an arbitrary point in space using nearest voxell or trilinear interpolation. You can also sample the gradient at that point.
```csharp
Vector3 point = new(1.0f, 2.0f, 3.0f);
float distanceDiscrete = dGrid.GetValue(point); // nearest voxel (will throw if out of bounds)
float distance = dGrid.SampleTrilinear(point); // trilinear interpolation
Vector3 gradient = dGrid.SampleGradient(point); // gradient at point
```

## License
This project is dual-licensed:

- **Open Source (GPL3.0)** — see [LICENSE](./LICENSE)  
- **Commercial License** — see [LICENSE-COMMERCIAL](./LICENSE-COMMERCIAL).

If you are a researcher, student, or using this project in an open-source context,  
use the GPL3.0 license.  

If you want to use this in proprietary software or closed-source products,  
you must obtain a commercial license. (contact: dev@zekezhang.com)

## References
- Akenine-Moller, T. (2001). Fast 3D Triangle-Box Overlap Testing. Journal of Graphics Tools, 6(1), 29–33. https://doi.org/10.1080/10867651.2001.10487535
- Felzenszwalb, P. F., & Huttenlocher, D. P. (2012). Distance Transforms of Sampled Functions. Theory of Computing, 8(1), 415–428. https://doi.org/10.4086/toc.2012.v008a019
