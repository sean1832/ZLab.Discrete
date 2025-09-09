using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using ZLab.Discrete.Geometry;

namespace ZLab.Discrete.Operations.Meshing
{
    internal static class VoxelFaceBuilder
    {
        private static readonly Vector3[] _dirs = new[]
        {
            new Vector3( 1,  0,  0), // +X
            new Vector3(-1,  0,  0), // -X
            new Vector3( 0,  1,  0), // +Y
            new Vector3( 0, -1,  0), // -Y
            new Vector3( 0,  0,  1), // +Z
            new Vector3( 0,  0, -1), // -Z
        };

        public static ReadOnlySpan<Vector3> Directions => _dirs;

        // Optional: if you want to loop faces without directions
        private static readonly (int axis, int sign)[] FaceDefs = new[]
        {
            (0, +1), (0, -1), // X+, X-
            (1, +1), (1, -1), // Y+, Y-
            (2, +1), (2, -1), // Z+, Z-
        };

        /// <summary>
        /// Make one quad face (two triangles) for the given voxel side.
        /// Uses unified axis/sign path (no 6-way switch).
        /// </summary>
        public static void MakeFace(Vector3 origin, Vector3 direction, List<Vector3> vertices,
            List<TriFace> faces, Vector3 voxelSize, CordSystem cordSystem)
        {
            if (!TryToAxisSign(direction, out int axis, out int sign))
                throw new ArgumentException("direction must be axis-aligned +-X/+-Y/+-Z", nameof(direction));

            int baseIndex = vertices.Count;
            Vector3 min = origin;
            Vector3 max = origin + voxelSize;

            // 4 corners for each face (RH outward). Index order: 0-1-2-3 around the quad.
            Vector3 a, b, c, d;

            switch (axis)
            {
                case 0: // X face
                    {
                        float x = sign > 0 ? max.X : min.X;
                        // rectangle on the plane x=const, oriented in Y-Z
                        a = new Vector3(x, min.Y, min.Z);
                        b = new Vector3(x, max.Y, min.Z);
                        c = new Vector3(x, max.Y, max.Z);
                        d = new Vector3(x, min.Y, max.Z);
                        // for -X, reverse the loop to keep outward normal
                        if (sign < 0) (b, d) = (d, b);
                    }
                    break;

                case 1: // Y face
                    {
                        float y = sign > 0 ? max.Y : min.Y;
                        // rectangle on the plane y=const, oriented in X-Z
                        a = new Vector3(min.X, y, min.Z);
                        b = new Vector3(max.X, y, min.Z);
                        c = new Vector3(max.X, y, max.Z);
                        d = new Vector3(min.X, y, max.Z);
                        if (sign < 0) (b, d) = (d, b);
                    }
                    break;

                default: // 2: Z face
                    {
                        float z = sign > 0 ? max.Z : min.Z;
                        // rectangle on the plane z=const, oriented in X-Y
                        a = new Vector3(min.X, min.Y, z);
                        b = new Vector3(max.X, min.Y, z);
                        c = new Vector3(max.X, max.Y, z);
                        d = new Vector3(min.X, max.Y, z);
                        if (sign < 0) (b, d) = (d, b);
                    }
                    break;
            }

            // Append verts
            vertices.Add(a);
            vertices.Add(b);
            vertices.Add(c);
            vertices.Add(d);

            // RH: (0,1,2) + (0,2,3). For LH, flip winding.
            if (cordSystem == CordSystem.RightHanded)
            {
                faces.Add(new TriFace(baseIndex + 0, baseIndex + 1, baseIndex + 2));
                faces.Add(new TriFace(baseIndex + 0, baseIndex + 2, baseIndex + 3));
            }
            else
            {
                faces.Add(new TriFace(baseIndex + 0, baseIndex + 2, baseIndex + 1));
                faces.Add(new TriFace(baseIndex + 0, baseIndex + 3, baseIndex + 2));
            }
        }


        /// <summary>
        /// Map a direction vector (±unit axis) to (axis, sign).
        /// Returns false if not axis-aligned.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool TryToAxisSign(in Vector3 dir, out int axis, out int sign)
        {
            // Expect exact cardinals; if you want robustness use a small epsilon test.
            if (dir.X > 0f) { axis = 0; sign = +1; return true; }
            if (dir.X < 0f) { axis = 0; sign = -1; return true; }
            if (dir.Y > 0f) { axis = 1; sign = +1; return true; }
            if (dir.Y < 0f) { axis = 1; sign = -1; return true; }
            if (dir.Z > 0f) { axis = 2; sign = +1; return true; }
            if (dir.Z < 0f) { axis = 2; sign = -1; return true; }

            axis = -1; sign = 0; return false;
        }
    }
}
