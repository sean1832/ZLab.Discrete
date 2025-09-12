using System;
using System.Buffers;
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
        /// Creates an empty mesh. (Invalid)
        /// </summary>
        public MeshF()
        {
            Vertices = Array.Empty<Vector3>();
            Faces = Array.Empty<TriFace>();
            IsClosed = false;
            _bounds = null;
        }

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

        /// <summary>
        /// Gets the axis-aligned bounding box (AABB) of the mesh.
        /// </summary>
        /// <returns></returns>
        public BBox GetBounds()
        {
            _bounds ??= ComputeBounds();
            return _bounds.Value;
        }

        /// <summary>
        /// Recomputes the axis-aligned bounding box (AABB) of the mesh.
        /// </summary>
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
            // Watertight (closed 2-manifold) check using sort+scan.
            // Conditions per edge group (same undirected key):
            //   - total == 2 (no more, no less)
            //   - sum(sign) == 0 (one +1, one -1) => consistent opposite orientation

            int vCount = Vertices.Length;
            int fCount = Faces.Length;
            if (fCount == 0) return false;

            // Validate faces quickly and count edges
            // (early exits keep us from allocating for obviously-bad meshes)
            for (int i = 0; i < fCount; i++)
            {
                TriFace f = Faces[i];
                int a = f.A, b = f.B, c = f.C;
                if ((uint)a >= (uint)vCount || (uint)b >= (uint)vCount || (uint)c >= (uint)vCount) return false;
                if (a == b || b == c || c == a) return false;
            }

            int totalEdges = fCount * 3;

            // Pool big buffers
            ArrayPool<ulong> keyPool = ArrayPool<ulong>.Shared;
            ArrayPool<sbyte> signPool = ArrayPool<sbyte>.Shared;

            ulong[] keys = keyPool.Rent(totalEdges);
            sbyte[] signs = signPool.Rent(totalEdges);

            int w = 0;
            try
            {
                // Emit edges (tight, branchless-ish inner loop)
                // If faces are huge, you can parallelize this block safely.
                for (int i = 0; i < fCount; i++)
                {
                    TriFace f = Faces[i];
                    // (a,b), (b,c), (c,a)

                    keys[w] = MakeEdgeKey(f.A, f.B, out sbyte s);
                    signs[w++] = s;

                    keys[w] = MakeEdgeKey(f.B, f.C, out s);
                    signs[w++] = s;

                    keys[w] = MakeEdgeKey(f.C, f.A, out s);
                    signs[w++] = s;
                }

                // Sort by key; keep signs aligned
                // Works on .NET Framework and .NET Core+.
                Array.Sort(keys, signs, 0, w);

                // Linear scan equal-key runs
                int iRun = 0;
                while (iRun < w)
                {
                    ulong k = keys[iRun];
                    int total = 0;
                    int sum = 0;

                    do
                    {
                        total++;
                        sum += signs[iRun];
                        iRun++;
                    }
                    while (iRun < w && keys[iRun] == k);

                    // Require exactly two half-edges per undirected edge,
                    // with opposite orientation (+1 and -1).
                    if (total != 2 || sum != 0)
                        return false;
                }

                return true;
            }
            finally
            {
                // Return pooled arrays without clearing (fast).
                keyPool.Return(keys, clearArray: false);
                signPool.Return(signs, clearArray: false);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static ulong MakeEdgeKey(int i, int j, out sbyte sign)
        {
            // normalize to undirected key (min,max); track orientation as sign
            bool pos = i < j;
            int a = pos ? i : j;
            int b = pos ? j : i;
            sign = pos ? (sbyte)+1 : (sbyte)-1;
            return ((ulong)(uint)a << 32) | (uint)b;
        }
        #endregion
    }
}

