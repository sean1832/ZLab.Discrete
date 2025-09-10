# Rasterization

## Dense rasterization
This will result in a dense occupancy grid where each voxel is classified as `Inside`, `Outside`, or `Boundary`.
```csharp
// Flood fill classifies interior; requires a closed mesh
DenseRasterizer.Rasterize(oGrid, mesh, floodFill: true);
```
- `oGrid` will be mutated.
- `floodFill` is optional and defaults to `false`. It is recommended to set it to `true` if the mesh is closed (watertight) to correctly identify interior voxels. This is a post-processing step that fills in the interior of the mesh after rasterization.

## Sparse rasterization
This will only return a list of voxel origins that are classified as `Boundary`.
```csharp
List<Vector3> origins = SparseRasterizer.Rasterize(Mesh, size);
```