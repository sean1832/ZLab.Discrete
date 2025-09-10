# Signed Distance Field (SDF)

## Grid Masking & Distance Field Creation
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

## Sample Signed Distance Field (SDF)
Sample an arbitrary point in space using nearest voxell or trilinear interpolation. You can also sample the gradient at that point.
```csharp
Vector3 point = new(1.0f, 2.0f, 3.0f);
float distanceDiscrete = dGrid.GetValue(point); // nearest voxel (will throw if out of bounds)
float distance = dGrid.SampleTrilinear(point); // trilinear interpolation
Vector3 gradient = dGrid.SampleGradient(point); // gradient at point
```