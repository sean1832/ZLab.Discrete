# Mesh Extraction
## Face-culling meshing
This will result a single mesh with internal faces removed.
```csharp
MeshF mesh = DiscreteMesher.GenerateMesh(origins, size);
```

## Naive meshing
This will result individual mesh for each voxel.
```csharp
// allocates a new array for the meshes
MeshF[] meshes = DiscreteMesher.GenerateMeshes(origins, size);
```
if you want no allocation at all, you can also do:
```csharp
MeshF[] buffer = ArrayPool<MeshF>.Shared.Rent(origins.Count);
Span<MeshF> meshSpan = buffer.AsSpan(0, origins.Count); // trim to actual size
try
{
    DiscreteMesher.GenerateMeshes(origins, size, meshSpan);
}
finally
{
    // remember to return the array to the pool
    ArrayPool<MeshF>.Shared.Return(buffer);
}
```