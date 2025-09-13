using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace ZLab.Discrete.Geometry
{
    /// <summary>
    /// Polyline defined by a sequence of 3D vertices that make up the line segments.
    /// </summary>
    public sealed class PolylineF
    {
        // Tunable tolerance for closedness in world units (float).
        /// <summary>
        /// Tolerance for determining if the polyline is closed (distance between first and last vertex).
        /// </summary>
        public const float ClosedEpsilon = 1e-6f;
        private const float ClosedEpsilonSq = ClosedEpsilon * ClosedEpsilon;

        private Vector3[] _buffer;
        private int _count;          // logical length

        /// <summary>
        /// Number of vertices.
        /// </summary>
        public int Count => _count;

        /// <summary>
        /// Read-only view of the vertices.
        /// </summary>
        public ReadOnlySpan<Vector3> Vertices => _buffer.AsSpan(0, _count);

        /// <summary>
        /// Indicates if the polyline is closed (the last vertex connects to the first).
        /// </summary>
        public bool IsClosed { get; private set; }

        /// <summary>
        /// Valid if there are at least two vertices.
        /// </summary>
        public bool IsValid => _count > 1;

        /// <summary>
        /// Total length (perimeter if closed).
        /// </summary>
        public float Length { get; private set; }

        /// <summary>
        /// Creates an empty polyline. (Invalid)
        /// </summary>
        /// <remarks>
        /// You shouldn't use an empty polyline for any meaningful operations.
        /// Useful only for initialization or placeholder purposes.
        /// </remarks>
        public PolylineF()
        {
            _buffer = Array.Empty<Vector3>();
            _count = 0;
            IsClosed = false;
            Length = 0f;
        }

        /// <summary>
        /// Creates a polyline and automatically checks if it is closed.
        /// </summary>
        /// <param name="vertices">Array of vertices defining the polyline</param>
        /// <remarks>
        /// Calculating closedness involves checking the distance between the first and last vertex. This is a simple check but may be inefficient for very large polylines.
        /// If you already know the polyline's closedness, consider using the constructor that accepts an explicit closedness parameter.
        /// </remarks>
        public PolylineF(Vector3[] vertices)
        {
            if (vertices is null) throw new ArgumentNullException(nameof(vertices));
            _buffer = (Vector3[])vertices.Clone();
            _count = _buffer.Length;
            IsClosed = DetectClosed(_buffer.AsSpan(0, _count));
            Length = ComputeLength(_buffer.AsSpan(0, _count));
        }

        /// <summary>
        /// Creates a polyline with the given vertices and explicit closedness.
        /// </summary>
        /// <param name="vertices">Array of vertices defining the polyline</param>
        /// <param name="isClosed">Whether the polyline is closed</param>
        /// <remarks>
        /// This constructor does not check if the first and last vertices are the same when isClosed is true.
        /// Use this constructor when you already know the closedness of the polyline to avoid unnecessary computations.
        /// </remarks>
        public PolylineF(Vector3[] vertices, bool isClosed)
        {
            if (vertices is null) throw new ArgumentNullException(nameof(vertices));
            _buffer = (Vector3[])vertices.Clone();
            _count = _buffer.Length;
            IsClosed = isClosed; // caller decides semantics
            Length = ComputeLength(_buffer.AsSpan(0, _count));
        }

        /// <summary>
        /// Appends a vertex to the end of the polyline, updating length and closedness.
        /// </summary>
        /// <param name="vertex">Vertex to append</param>
        public void Append(Vector3 vertex)
        {
            EnsureCapacity(_count + 1);

            // incremental length: add segment from previous last to new
            if (_count > 0)
                Length += (float)Distance(_buffer[_count - 1], vertex);

            _buffer[_count++] = vertex;

            // update closedness
            IsClosed = DetectClosed(_buffer.AsSpan(0, _count));
        }

        /// <summary>
        /// Appends multiple vertices to the end of the polyline, updating length and closedness.
        /// </summary>
        /// <param name="vertices">Array of vertices to append</param>
        public void Append(ReadOnlySpan<Vector3> vertices)
        {
            int count = vertices.Length;
            if (count == 0) return;

            EnsureCapacity(_count + count);

            // incremental length across the boundary (old tail -> new head)
            if (_count > 0)
                Length += (float)Distance(_buffer[_count - 1], vertices[0]);

            // incremental length inside the appended span
            double segSum = 0.0;
            for (int i = 1; i < count; i++)
                segSum += Distance(vertices[i - 1], vertices[i]);
            Length += (float)segSum;

            // copy the new vertices into place
            vertices.CopyTo(_buffer.AsSpan(_count));
            _count += count;

            // update closedness with new endpoint
            IsClosed = DetectClosed(_buffer.AsSpan(0, _count));
        }

        #region Private methods

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void EnsureCapacity(int need)
        {
            if (_buffer.Length >= need) return;
            int newCap = _buffer.Length == 0 ? 8 : _buffer.Length * 2;
            if (newCap < need) newCap = need;
            Array.Resize(ref _buffer, newCap);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool DetectClosed(ReadOnlySpan<Vector3> v)
        {
            if (v.Length < 3) return false;
            Vector3 d = v[0] - v[^1];
            return d.LengthSquared() <= ClosedEpsilonSq;
        }

        private static float ComputeLength(ReadOnlySpan<Vector3> v)
        {
            int n = v.Length;
            if (n < 2) return 0f;
            double sum = 0.0;
            for (int i = 1; i < n; i++)
                sum += Distance(v[i - 1], v[i]);
            return (float)sum;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static double Distance(in Vector3 a, in Vector3 b)
        {
            Vector3 d = a - b;
            return Math.Sqrt(d.LengthSquared());
        }

        #endregion


    }
}
