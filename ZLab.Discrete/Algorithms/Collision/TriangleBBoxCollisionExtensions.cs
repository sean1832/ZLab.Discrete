using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using ZLab.Discrete.Geometry;

namespace ZLab.Discrete.Algorithms.Collision
{
    /// <summary>
    /// Axis-aligned bounding box (AABB) intersection tests.
    /// </summary>
    internal static class TriangleBBoxCollisionExtensions
    {
        private const float SAT_EPS = 1e-5f;
        private const float CONS_PAD = 1e-4f;
        /// <summary>
        /// Tests if a triangle intersects an axis-aligned bounding box (AABB).
        /// </summary>
        /// <remarks>
        /// Reference: <a href="https://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/code/tribox_tam.pdf">
        /// Akenine-Moller, T. (2001). Fast 3D Triangle-Box Overlap Testing.</a>
        /// </remarks>
        /// <param name="box">Axis-aligned bounding box to test with</param>
        /// <param name="v0">first vertex of a triangle</param>
        /// <param name="v1">second vertex of a triangle</param>
        /// <param name="v2">third vertex of a triangle</param>
        /// <returns>Triangle intersects the bounding box or not</returns>
        public static bool IsIntersectsTriangle(this in BBox box, in Vector3 v0, in Vector3 v1, in Vector3 v2)
        {
            // Canonical Akenine-Moller triangle-box overlap test

            // Move triangle into box's local coordinate frame
            Vector3 boxCenter = box.Center;
            Vector3 boxHalfSize = (box.Max - box.Min) * 0.5f;

            Vector3 v0b = v0 - boxCenter;
            Vector3 v1b = v1 - boxCenter;
            Vector3 v2b = v2 - boxCenter;


            // early out: triangle AABB VS box AABB
            BBox triangleBox = TriangleBounds(v0b, v1b, v2b);
            if (triangleBox.Min.X > boxHalfSize.X || triangleBox.Max.X < -boxHalfSize.X) return false; // No intersection on X-axis
            if (triangleBox.Min.Y > boxHalfSize.Y || triangleBox.Max.Y < -boxHalfSize.Y) return false; // No intersection on Y-axis
            if (triangleBox.Min.Z > boxHalfSize.Z || triangleBox.Max.Z < -boxHalfSize.Z) return false; // No intersection on Z-axis


            // Compute triangle edges
            Vector3 e0 = v1b - v0b;
            Vector3 e1 = v2b - v1b;
            Vector3 e2 = v0b - v2b;

            // Test the triangle normal
            Vector3 normal = Vector3.Cross(e0, e1);
            if (!PlaneBoxOverlap(normal, v0b, boxHalfSize)) return false;

            // 9 axis tests (edge cross box axes). Expanded, branch-light, no arrays.
            if (!AxisTest_X01(e0.Z, e0.Y, v0b, v2b, boxHalfSize)) return false;
            if (!AxisTest_Y02(e0.Z, e0.X, v0b, v2b, boxHalfSize)) return false;
            if (!AxisTest_Z12(e0.Y, e0.X, v1b, v2b, boxHalfSize)) return false;

            if (!AxisTest_X01(e1.Z, e1.Y, v0b, v2b, boxHalfSize)) return false;
            if (!AxisTest_Y02(e1.Z, e1.X, v0b, v2b, boxHalfSize)) return false;
            if (!AxisTest_Z12(e1.Y, e1.X, v0b, v1b, boxHalfSize)) return false;

            if (!AxisTest_X01(e2.Z, e2.Y, v0b, v1b, boxHalfSize)) return false;
            if (!AxisTest_Y02(e2.Z, e2.X, v0b, v1b, boxHalfSize)) return false;
            if (!AxisTest_Z12(e2.Y, e2.X, v0b, v2b, boxHalfSize)) return false;

            return true; // Intersection occurs
        }

        /// <summary>
        /// Tests if an axis-aligned bounding box (AABB) is fully covered by a triangle.
        /// </summary>
        /// <remarks>
        /// Use this test in addition to <see cref="IsIntersectsTriangle"/> to catch
        /// </remarks>
        /// <returns>Triangle fully covers the bounding box or not</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsCoveredByTriangle(this in BBox box, in Vector3 v0, in Vector3 v1, in Vector3 v2)
        {
            Vector3 c = box.Center;
            Vector3 half = (box.Max - box.Min) * 0.5f;

            // Triangle plane normal (unnormalized)
            Vector3 n = Vector3.Cross(v1 - v0, v2 - v0);
            float nLen2 = n.LengthSquared();
            if (nLen2 < 1e-12f) return false; // degenerate triangle: treat as empty

            // --- Conservative plane-vs-box test (no sqrt, works for anisotropic voxels) ---
            // distance from plane, but without dividing by |n|
            float distNum = Vector3.Dot(n, c - v0);
            // projection radius of the box onto the plane normal (also without dividing by |n|)
            float r = MathF.Abs(n.X) * half.X + MathF.Abs(n.Y) * half.Y + MathF.Abs(n.Z) * half.Z;

            // small extra safety band to avoid "edge-only" speckling on thin triangles
            if (distNum > r + CONS_PAD || distNum < -r - CONS_PAD)
                return false;

            // --- Project center to the triangle plane using n / |n|^2 (no normalize/sqrt) ---
            // p = c - (dot(n, c - v0) / |n|^2) * n
            Vector3 p = c - (distNum / nLen2) * n;

            // --- Barycentric test in the triangle basis (stable because p is on the plane) ---
            Vector3 e0 = v1 - v0;
            Vector3 e1 = v2 - v0;
            Vector3 vp = p - v0;

            float d00 = Vector3.Dot(e0, e0);
            float d01 = Vector3.Dot(e0, e1);
            float d11 = Vector3.Dot(e1, e1);
            float d20 = Vector3.Dot(vp, e0);
            float d21 = Vector3.Dot(vp, e1);

            float denom = d00 * d11 - d01 * d01;
            if (MathF.Abs(denom) < 1e-20f) return false; // nearly collinear

            float v = (d11 * d20 - d01 * d21) / denom;
            float w = (d00 * d21 - d01 * d20) / denom;
            float u = 1.0f - v - w;

            // Slightly conservative acceptance to avoid "gridline gaps"
            const float BARY_EPS = -1e-5f;
            return (u >= BARY_EPS) & (v >= BARY_EPS) & (w >= BARY_EPS);
        }

        #region Private Methods

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static BBox TriangleBounds(in Vector3 v0, in Vector3 v1, in Vector3 v2)
        {
            Vector3 min = Vector3.Min(v0, Vector3.Min(v1, v2));
            Vector3 max = Vector3.Max(v0, Vector3.Max(v1, v2));
            return new BBox(min, max);
        }

        // Edge cross X-axis tests
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool AxisTest_X01(float aZ, float aY, in Vector3 v0, in Vector3 v2, in Vector3 h)
        {
            // Project triangle onto axis L = (0, -aZ, aY)
            float p0 = aZ * v0.Y - aY * v0.Z;
            float p2 = aZ * v2.Y - aY * v2.Z;
            float min = MathF.Min(p0, p2), max = MathF.Max(p0, p2);
            float r = MathF.Abs(aZ) * h.Y + MathF.Abs(aY) * h.Z + SAT_EPS;
            return !(min > r || max < -r);
        }

        // Edge cross Y-axis tests
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool AxisTest_Y02(float aZ, float aX, Vector3 v0, Vector3 v2, Vector3 h)
        {
            // L = (aZ, 0, -aX)
            float p0 = -aZ * v0.X + aX * v0.Z;
            float p2 = -aZ * v2.X + aX * v2.Z;
            float min = MathF.Min(p0, p2), max = MathF.Max(p0, p2);
            float r = MathF.Abs(aZ) * h.X + MathF.Abs(aX) * h.Z + SAT_EPS;
            return !(min > r || max < -r);
        }

        // Edge cross Z-axis tests
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool AxisTest_Z12(float aY, float aX, in Vector3 v1, in Vector3 v2, in Vector3 h)
        {
            // L = (-aY, aX, 0)
            float p1 = aY * v1.X - aX * v1.Y;
            float p2 = aY * v2.X - aX * v2.Y;
            float min = MathF.Min(p1, p2), max = MathF.Max(p1, p2);
            float r = MathF.Abs(aY) * h.X + MathF.Abs(aX) * h.Y + SAT_EPS;
            return !(min > r || max < -r);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool PlaneBoxOverlap(in Vector3 normal, in Vector3 vert, in Vector3 half)
        {
            // Compute projection interval radius of box onto plane normal
            // Using "positive" and "negative" vertices trick.
            float vMinX = normal.X > 0 ? -half.X - vert.X : half.X - vert.X;
            float vMaxX = normal.X > 0 ? half.X - vert.X : -half.X - vert.X;
            float vMinY = normal.Y > 0 ? -half.Y - vert.Y : half.Y - vert.Y;
            float vMaxY = normal.Y > 0 ? half.Y - vert.Y : -half.Y - vert.Y;
            float vMinZ = normal.Z > 0 ? -half.Z - vert.Z : half.Z - vert.Z;
            float vMaxZ = normal.Z > 0 ? half.Z - vert.Z : -half.Z - vert.Z;

            float dMin = normal.X * vMinX + normal.Y * vMinY + normal.Z * vMinZ;
            if (dMin > SAT_EPS) return false;

            float dMax = normal.X * vMaxX + normal.Y * vMaxY + normal.Z * vMaxZ;
            return dMax >= -SAT_EPS;
        }
        #endregion
    }
}
