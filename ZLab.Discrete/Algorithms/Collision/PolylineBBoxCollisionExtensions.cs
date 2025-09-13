using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using ZLab.Discrete.Geometry;

namespace ZLab.Discrete.Algorithms.Collision
{
    /// <summary>
    /// Extensions for polyline and bounding box collision detection.
    /// </summary>
    internal static class PolylineBBoxCollisionExtensions
    {
        /// <summary>
        /// Checks if a polyline intersects with an axis-aligned bounding box (AABB).
        /// </summary>
        /// <param name="bBox">The axis-aligned bounding box to test against</param>
        /// <param name="polyline">The polyline defined by a sequence of vertices</param>
        /// <param name="isClosed">Whether the polyline is closed (last vertex connects to the first)</param>
        /// <returns>True if the polyline intersects the bounding box, false otherwise</returns>
        /// <remarks>
        /// Used Liang–Barsky / slab method for segment-box intersection tests.
        /// </remarks>
        public static bool IsIntersectsPolyline(this in BBox bBox, ReadOnlySpan<Vector3> polyline, bool isClosed)
        {
            int vertexCount = polyline.Length;
            if (vertexCount == 0 || vertexCount == 1)
                return false; // No segments to test

            // if any vertex is inside the box, intersection is true
            for (int i = 0; i < vertexCount; i++)
            {
                if (ContainsPoint(bBox, polyline[i]))
                    return true;
            }

            // Test each segment against the box
            Vector3 segmentStart = polyline[0];
            for (int i = 0; i < vertexCount; i++)
            {
                Vector3 segmentEnd = polyline[i];
                if (IsSegmentIntersectAabb(segmentStart, segmentEnd, bBox.Min, bBox.Max))
                    return true;
                segmentStart = segmentEnd;
            }

            // If closed, test the closing segment
            if (isClosed && IsSegmentIntersectAabb(polyline[vertexCount - 1], polyline[0], bBox.Min, bBox.Max))
                return true;

            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool ContainsPoint(in BBox box, in Vector3 point)
        {
            return point.X >= box.Min.X && point.X <= box.Max.X  // X axis
                            && point.Y >= box.Min.Y && point.Y <= box.Max.Y // Y axis
                            && point.Z >= box.Min.Z && point.Z <= box.Max.Z; // Z axis
        }

        /// <summary>
        /// Liang–Barsky / slab test: clips the segment against the AABB on each axis.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool IsSegmentIntersectAabb(in Vector3 startPoint, in Vector3 endPoint,
            in Vector3 aabbMin, in Vector3 aabbMax)
        {
            const float parallelEpsilon = 1e-12f;

            Vector3 direction = endPoint - startPoint;
            float tEnter = 0f; // max of entering ts
            float tExit = 1f; // min of exiting ts

            // X axis
            if (!ClipAgainstAxis(startPoint.X, direction.X, aabbMin.X, aabbMax.X, ref tEnter, ref tExit, parallelEpsilon))
                return false;

            // Y axis
            if (!ClipAgainstAxis(startPoint.Y, direction.Y, aabbMin.Y, aabbMax.Y, ref tEnter, ref tExit, parallelEpsilon))
                return false;

            // Z axis
            if (!ClipAgainstAxis(startPoint.Z, direction.Z, aabbMin.Z, aabbMax.Z, ref tEnter, ref tExit, parallelEpsilon))
                return false;

            return tEnter <= tExit;
        }

        /// <summary>
        /// Clips a 1D segment p(t) = origin + t * delta against the interval [min, max].
        /// Updates the valid parameter range [tEnter, tExit]. Returns false if empty.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool ClipAgainstAxis(float origin, float delta, float min, float max,
            ref float tEnter, ref float tExit, float parallelEpsilon)
        {
            if (MathF.Abs(delta) < parallelEpsilon)
            {
                // Segment is parallel to this axis; accept only if origin lies within the slab.
                return origin >= min && origin <= max;
            }

            float invDelta = 1f / delta;
            float tNear = (min - origin) * invDelta;
            float tFar = (max - origin) * invDelta;

            // Swap if necessary to ensure tNear <= tFar
            if (tNear > tFar)
            {
                (tNear, tFar) = (tFar, tNear);
            }

            if (tNear > tEnter) tEnter = tNear;
            if (tFar < tExit) tExit = tFar;

            return tEnter <= tExit;
        }
    }
}
