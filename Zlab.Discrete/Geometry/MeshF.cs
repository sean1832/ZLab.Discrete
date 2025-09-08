using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace ZLab.Discrete.Geometry
{
    /// <summary>
    /// Triangle mesh with float precision vertices.
    /// </summary>
    public sealed class MeshF
    {
        /// <summary>
        /// Array of vertex positions.
        /// </summary>
        public readonly Vector3[] Vertices;

        /// <summary>
        /// Array of faces, each face is an array of vertex indices. (must be triangles)
        /// </summary>
        public readonly TriFace[] Faces;

        /// <summary>
        /// Indicates if the mesh is closed (watertight).
        /// </summary>
        public readonly bool IsClosed;

        /// <summary>
        /// Checks if the mesh is valid (at least 3 vertices and 1 face).
        /// </summary>
        public bool IsValid => Vertices.Length > 2 && Faces.Length > 0;

        private BBox? _bounds;

        /// <summary>
        /// Creates a mesh and automatically checks if it is closed. (slow for large meshes, explicitly specify closeness if possible)
        /// </summary>
        public MeshF(Vector3[] vertices, TriFace[] faces)
        {
            Vertices = vertices;
            Faces = faces;
            IsClosed = CheckWaterTight();
        }

        /// <summary>
        /// Creates a mesh with explicit specified closeness. (no watertight check)
        /// </summary>
        public MeshF(Vector3[] vertices, TriFace[] faces, bool isClosed)
        {
            Vertices = vertices;
            Faces = faces;
            IsClosed = isClosed;
        }

        public BBox GetBounds()
        {
            _bounds ??= ComputeBounds();
            return _bounds.Value;
        }
        public void RecomputeBounds()
        {
            _bounds = ComputeBounds();
        }

        /// <summary>
        /// Computes the axis-aligned bounding box (AABB) of the mesh.
        /// </summary>
        private BBox ComputeBounds()
        {
            if (Vertices.Length == 0) return new BBox();
            Vector3 min = Vertices[0];
            Vector3 max = Vertices[0];
            foreach (Vector3 v in Vertices)
            {
                min = Vector3.Min(min, v);
                max = Vector3.Max(max, v);
            }
            return new BBox(min, max);
        }

        /// <summary>
        /// Enumerates the axis-aligned bounding boxes (AABB) of each triangle face in the mesh.
        /// <code>
        /// foreach (BBox triBox in mesh.EnumerateTriangleBounds())
        /// {
        ///     // Process each triangle bounding box (triBox)
        /// }
        /// </code>
        /// </summary>
        public IEnumerable<BBox> EnumerateTriangleBounds()
        {
            foreach (TriFace face in Faces)
            {
                face.QueryVertices(Vertices, out Vector3 v0, out Vector3 v1, out Vector3 v2);
                Vector3 min = Vector3.Min(v0, Vector3.Min(v1, v2));
                Vector3 max = Vector3.Max(v0, Vector3.Max(v1, v2));
                yield return new BBox(min, max);
            }
        }


        #region Private Methods

        // Check if the mesh is watertight (closed manifold)
        private bool CheckWaterTight()
        {
            int vCount = Vertices.Length;
            int fCount = Faces.Length;
            if (fCount == 0) return false;

            Dictionary<ulong, int> edges = new(fCount * 2);

            foreach (TriFace f in Faces)
            {
                int a = f.A, b = f.B, c = f.C;
                if (a == b || b == c || c == a) return false;

                Acc(a, b);
                Acc(b, c);
                Acc(c, a);
            }

            foreach (KeyValuePair<ulong, int> kv in edges)
            {
                Unpack(kv.Value, out int total, out int pos, out int neg);
                if (total != 2) return false; // open or non-manifold edge
                if (pos != 1 || neg != 1) return false; // inconsistent orientation
            }
            return true;


            // local helper to accumulate one directed edge
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            void Acc(int a, int b)
            {
                if ((uint)a >= (uint)vCount || (uint)b >= (uint)vCount) { throw new InvalidOperationException("Index out of range"); }
                if (a == b) { throw new InvalidOperationException("Degenerate edge"); }

                bool pos = a < b; // orientation relative to (min,max)
                int min = pos ? a : b;
                int max = pos ? b : a;

                ulong key = (ulong)(uint)min << 32 | (uint)max;
#if NETFRAMEWORK
                int packed = edges.TryGetValue(key, out int val) ? val : 0;
#else
                // .NET 6+ has GetValueOrDefault
                int packed = edges.GetValueOrDefault(key, 0);
#endif
                Unpack(packed, out int total, out int posCnt, out int negCnt);
                total++; if (pos) posCnt++; else negCnt++;
                edges[key] = Pack(total, posCnt, negCnt);
            }
        }

        // Pack an undirected edge (min,max) into a 64-bit key
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static ulong EdgeKey(int a, int b)
        {
            uint u = (uint)Math.Min(a, b);
            uint v = (uint)Math.Max(a, b);
            return (ulong)u << 32 | v;
        }

        // Pack counts into one int: bits 0-9 total, 10-19 pos, 20-29 neg (each 10 bits is plenty)
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static int Pack(int total, int pos, int neg) => total & 0x3FF | (pos & 0x3FF) << 10 | (neg & 0x3FF) << 20;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void Unpack(int packed, out int total, out int pos, out int neg)
        {
            total = packed & 0x3FF;
            pos = packed >> 10 & 0x3FF;
            neg = packed >> 20 & 0x3FF;
        }

        #endregion
    }
}

