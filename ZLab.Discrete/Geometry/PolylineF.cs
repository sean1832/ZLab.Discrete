using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace ZLab.Discrete.Geometry
{
    /// <summary>
    /// Polyline defined by a sequence of 3D vertices that make up the line segments.
    /// </summary>
    public sealed class PolylineF
    {
        /// <summary>
        /// Vertices of the polyline.
        /// </summary>
        public readonly Vector3[] Vertices;

        /// <summary>
        /// Indicates if the polyline is closed (the last vertex connects to the first).
        /// </summary>
        public readonly bool IsClosed;

        /// <summary>
        /// Checks if the polyline is valid (at least 2 vertices).
        /// </summary>
        public bool IsValid => Vertices.Length > 1;
        private float? _length;

        /// <summary>
        /// Creates an empty polyline. (Invalid)
        /// </summary>
        /// <remarks>
        /// You shouldn't use an empty polyline for any meaningful operations.
        /// Useful only for initialization or placeholder purposes.
        /// </remarks>
        public PolylineF()
        {
            Vertices = Array.Empty<Vector3>();
            IsClosed = false;
            _length = null;
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
            Vertices = vertices;
            IsClosed = IsPolylineClose();
            _length = null;
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
            Vertices = vertices;
            IsClosed = isClosed;
            _length = null;
        }

        /// <summary>
        /// Calculates the total length of the polyline.
        /// </summary>
        public float Length => GetLength();

        private float GetLength()
        {
            if (_length.HasValue) return _length.Value;
            float len = 0f;
            for (int i = 1; i < Vertices.Length; i++)
            {
                len += Vector3.Distance(Vertices[i - 1], Vertices[i]);
            }
            if (IsClosed && Vertices.Length > 2)
            {
                len += Vector3.Distance(Vertices[^1], Vertices[0]);
            }
            _length = len;
            return len;
        }

        private bool IsPolylineClose()
        {
            if (Vertices.Length < 3) return false;
            return Vector3.Distance(Vertices[0], Vertices[^1]) < 1e-6f;
        }
    }
}
