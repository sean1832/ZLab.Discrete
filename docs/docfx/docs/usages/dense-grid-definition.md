# Define dense voxel grid region
```csharp
// Axis-aligned bounds and uniform/non-uniform voxel size
BBox bounds = new(min: new Vector3(0,0,0), max: new Vector3(10,10,10));
Vector3 size = new(0.5f, 1f, 0.5f);
OccupancyGrid oGrid = new(size, bounds);
```