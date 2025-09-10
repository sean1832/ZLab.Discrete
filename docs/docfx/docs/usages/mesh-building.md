# Mesh building
To construct a [MeshF](/api/ZLab.Discrete.Geometry.MeshF.html), you need to provide an array of vertices and an array of faces. Currently, only triangular faces ([TriFace](/api/ZLab.Discrete.Geometry.TriFace.html)) are supported.

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
Where setting `isClosed` to `true` or `false` skips the interal watertight check, which can be slow for large meshes.