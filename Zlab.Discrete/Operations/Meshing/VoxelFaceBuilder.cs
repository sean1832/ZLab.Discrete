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

            // Build the four corners on the stack (no allocations)
            Span<Vector3> fv = stackalloc Vector3[4];
            Vector3 min = origin;
            Vector3 max = origin + voxelSize;
            FillFaceVerticesUnified(min, max, axis, sign, cordSystem, fv);

            // Append verts
            vertices.Add(fv[0]);
            vertices.Add(fv[1]);
            vertices.Add(fv[2]);
            vertices.Add(fv[3]);

            // Two tris with consistent winding (outward normal)
            faces.Add(new TriFace(baseIndex + 0, baseIndex + 1, baseIndex + 2));
            faces.Add(new TriFace(baseIndex + 0, baseIndex + 2, baseIndex + 3));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void FillFaceVerticesUnified(
            Vector3 min, Vector3 max, int axis, int sign, // axis: 0=x,1=y,2=z ; sign: +1/-1
            CordSystem cordSystem, Span<Vector3> fv)
        {
            // Compute plane point p0 and edge vectors u, v for the face
            Vector3 p0, u, v;

            if (axis == 0) // X face
            {
                float x = sign > 0 ? max.X : min.X;
                p0 = new Vector3(x, min.Y, min.Z);
                u = new Vector3(0, max.Y - min.Y, 0); // along +Y
                v = new Vector3(0, 0, max.Z - min.Z); // along +Z
                // Winding: flip u for LH vs RH (or swap u/v)
                if (sign < 0) (u, v) = (v, u); // example: adjust to keep outward normal
            }
            else if (axis == 1) // Y face
            {
                float y = sign > 0 ? max.Y : min.Y;
                p0 = new Vector3(min.X, y, min.Z);
                u = new Vector3(max.X - min.X, 0, 0); // +X
                v = new Vector3(0, 0, max.Z - min.Z); // +Z
                if (cordSystem == CordSystem.LeftHanded) u = -u; // toggle winding
                if (sign < 0) v = -v; // ensure outward normal
            }
            else // Z face
            {
                float z = sign > 0 ? max.Z : min.Z;
                p0 = new Vector3(min.X, min.Y, z);
                u = new Vector3(max.X - min.X, 0, 0); // +X
                v = new Vector3(0, max.Y - min.Y, 0); // +Y
                if (sign < 0) u = -u;
            }

            // Quad corners
            fv[0] = p0;
            fv[1] = p0 + u;
            fv[2] = p0 + u + v;
            fv[3] = p0 + v;
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
