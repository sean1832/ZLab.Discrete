# Enumerate voxels inside the grid

## Single threaded
```csharp
List<Vector3> origins = new();
oGrid.ForEachVoxel((origin, state) => {
    if (state == Occupancy.Boundary)  // <- change this to select different states
        origins.Add(origin);
})
```

## Multi-threaded
```csharp
ConcurrentBag<Vector3> origins = new();
oGrid.ForEachVoxelParallel((origin, state) => {
    if (state == Occupancy.Boundary)  // <- change this to select different states
        origins.Add(origin);
})
```