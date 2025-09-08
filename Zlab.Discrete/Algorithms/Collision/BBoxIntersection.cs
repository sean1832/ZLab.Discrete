using System.Numerics;
using System.Runtime.CompilerServices;
using ZLab.Discrete.Core;
using ZLab.Discrete.Geometry;

namespace ZLab.Discrete.Algorithms.Collision
{
    /// <summary>
    /// Axis-aligned bounding box (AABB) intersection tests.
    /// </summary>
    internal static class BBoxIntersection
    {
        /// <summary>
        /// Tests if a triangle intersects an axis-aligned bounding box (AABB).
        /// </summary>
        /// <remarks>
        /// Reference: <a href="https://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/code/tribox_tam.pdf">
        /// Akenine-Moller, T. (2001). Fast 3D Triangle-Box Overlap Testing.</a>
        /// </remarks>
        /// <param name="v0">first vertex of a triangle</param>
        /// <param name="v1">second vertex of a triangle</param>
        /// <param name="v2">third vertex of a triangle</param>
        /// <param name="box">Axis-aligned bounding box to test with</param>
        /// <returns>Triangle intersects the bounding box or not</returns>
        public static bool TriangleIntersectsAabb(Vector3 v0, in Vector3 v1, in Vector3 v2, BBox box)
        {
            // Canonical Akenine-Moller triangle-box overlap test

            // Move triangle into box's local coordinate frame
            Vector3 boxCenter = (box.Min + box.Max) * 0.5f;
            Vector3 boxHalfSize = (box.Max - box.Min) * 0.5f;

            Vector3 v0b = v0 - boxCenter;
            Vector3 v1b = v1 - boxCenter;
            Vector3 v2b = v2 - boxCenter;


            // early out: triangle AABB VS box AABB
            BBox triangleBox = TriangleBounds(in v0b, in v1b, in v2b);
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
        private static bool AxisTest_X01(float aZ, float aY, Vector3 v0, Vector3 v2, Vector3 h)
        {
            // Project triangle onto axis L = (0, -aZ, aY)
            float p0 = aZ * v0.Y - aY * v0.Z;
            float p2 = aZ * v2.Y - aY * v2.Z;
            float min = MathFx.Min(p0, p2);
            float max = MathFx.Max(p0, p2);
            float r = MathFx.Abs(aZ) * h.Y + MathFx.Abs(aY) * h.Z;
            return !(min > r || max < -r);
        }

        // Edge cross Y-axis tests
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool AxisTest_Y02(float aZ, float aX, Vector3 v0, Vector3 v2, Vector3 h)
        {
            // L = (aZ, 0, -aX)
            float p0 = -aZ * v0.X + aX * v0.Z;
            float p2 = -aZ * v2.X + aX * v2.Z;
            float min = MathFx.Min(p0, p2);
            float max = MathFx.Max(p0, p2);
            float r = MathFx.Abs(aZ) * h.X + MathFx.Abs(aX) * h.Z;
            return !(min > r || max < -r);
        }

        // Edge cross Z-axis tests
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool AxisTest_Z12(float aY, float aX, Vector3 v1, Vector3 v2, Vector3 h)
        {
            // L = (-aY, aX, 0)
            float p1 = aY * v1.X - aX * v1.Y;
            float p2 = aY * v2.X - aX * v2.Y;
            float min = MathFx.Min(p1, p2);
            float max = MathFx.Max(p1, p2);
            float r = MathFx.Abs(aY) * h.X + MathFx.Abs(aX) * h.Y;
            return !(min > r || max < -r);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool PlaneBoxOverlap(Vector3 normal, Vector3 vert, Vector3 half)
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
            if (dMin > 1e-6f) return false; // fully on positive side

            float dMax = normal.X * vMaxX + normal.Y * vMaxY + normal.Z * vMaxZ;
            return dMax >= -1e-6f;
        }
        #endregion
    }
}
