using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using ZLab.Discrete.Operations.Meshing;

namespace ZLab.Discrete.Geometry
{
    /// <summary>
    /// Axis-aligned bounding box (AABB) in 3D space.
    /// </summary>
    public struct BBox
    {
        // ==================
        // Properties
        // ==================

        /// <summary>
        /// Minimum corner of the bounding box.
        /// </summary>
        public Vector3 Min;
        /// <summary>
        /// Maximum corner of the bounding box.
        /// </summary>
        public Vector3 Max;

        /// <summary>
        /// Center point of the bounding box.
        /// </summary>
        public Vector3 Center => (Min + Max) * 0.5f;

        /// <summary>
        /// Cartesian dimension of the bounding box. (Width, Height, Depth)
        /// </summary>
        public Vector3 Size => Max - Min;

        /// <summary>
        /// Surface area of the box (2*(xy + yz + zx)). Returns 0 for invalid boxes.
        /// </summary>
        public float SurfaceArea
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => IsValid
                ? 2f * (Size.X * Size.Y + Size.Y * Size.Z + Size.Z * Size.X)
                : 0f;
        }

        /// <summary>
        /// Checks if the bounding box is valid (not degenerate).
        /// </summary>
        public readonly bool IsValid => !IsDegenerate();

        /// <summary>
        /// An empty bounding box (degenerate). Use Expand() to grow it.
        /// </summary>
        public static BBox Empty => new BBox();

        /// <summary>
        /// An infinite bounding box.
        /// </summary>
        public static BBox Infinite => new BBox(new Vector3(float.MinValue), new Vector3(float.MaxValue));


        // ==================
        // Constructors
        // ==================

        /// <summary>
        /// Creates a bounding box from minimum and maximum corner points.
        /// </summary>
        /// <param name="min">Minimum corner</param>
        /// <param name="max">Maximum corner</param>
        public BBox(Vector3 min, Vector3 max)
        {
            Min = min;
            Max = max;
        }

        /// <summary>
        /// Creates an empty bounding box. The box is degenerate until expanded.
        /// </summary>
        public BBox()
        {
            Min = new Vector3(float.MaxValue);
            Max = new Vector3(float.MinValue);
        }

        // ==================
        // Public Methods
        // ==================

        /// <summary>
        /// Create a box from possibly unordered corners by normalizing per axis.
        /// </summary>
        public static BBox FromUnordered(Vector3 a, Vector3 b)
        {
            return new BBox(Vector3.Min(a, b), Vector3.Max(a, b));
        }

        /// <summary>
        /// String representation of the bounding box.
        /// </summary>
        /// <returns>string</returns>
        public override string ToString()
        {
            return $"BBox[Min({Min.X},{Min.Y},{Min.Z}), Max({Max.X},{Max.Y},{Max.Z})]";
        }


        /// <summary>
        /// Checks if this bounding box intersects with another bounding box.
        /// </summary>
        /// <param name="other">Other bounding box</param>
        /// <returns>intersects or not</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public readonly bool Intersects(BBox other)
        {
            return Min.X <= other.Max.X && Max.X >= other.Min.X &&
                   Min.Y <= other.Max.Y && Max.Y >= other.Min.Y &&
                   Min.Z <= other.Max.Z && Max.Z >= other.Min.Z;
        }

        /// <summary>
        /// Checks if a point is contained within the bounding box.
        /// </summary>
        /// <param name="point">The point to test</param>
        /// <returns>Point in bounding box or not</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public readonly bool Contains(Vector3 point)
        {
            return point.X >= Min.X && point.X <= Max.X &&
                   point.Y >= Min.Y && point.Y <= Max.Y &&
                   point.Z >= Min.Z && point.Z <= Max.Z;
        }

        /// <summary>
        /// Checks if another bounding box is fully contained within this bounding box.
        /// </summary>
        public readonly bool Contains(BBox other)
        {
            return other.Min.X >= Min.X && other.Max.X <= Max.X &&
                   other.Min.Y >= Min.Y && other.Max.Y <= Max.Y &&
                   other.Min.Z >= Min.Z && other.Max.Z <= Max.Z;
        }

        /// <summary>
        /// Expands the bounding box to include another bounding box.
        /// </summary>
        /// <param name="other">Other bounding box</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Expand(BBox other)
        {
            Min = Vector3.Min(Min, other.Min);
            Max = Vector3.Max(Max, other.Max);
        }

        /// <summary>
        /// Expands the bounding box to include a point.
        /// </summary>
        /// <param name="point">Point to include</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Expand(Vector3 point)
        {
            Min = Vector3.Min(Min, point);
            Max = Vector3.Max(Max, point);
        }

        /// <summary>
        /// Uniformly pad the box by a non-negative margin.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Pad(float margin)
        {
            if (margin < 0f) return;
            Vector3 m = new Vector3(margin);
            Min -= m; Max += m;
        }

        // ----------------- Meshing ------------------
        /// <summary>
        /// Converts the bounding box to a mesh representation (12 triangles, 8 vertices).
        /// Winding is CCW for RightHanded; flipped for LeftHanded.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public readonly MeshF ToMesh(CordSystem cordSystem)
        {
            Vector3 v0 = new(Min.X, Min.Y, Min.Z);
            Vector3 v1 = new(Max.X, Min.Y, Min.Z);
            Vector3 v2 = new(Max.X, Max.Y, Min.Z);
            Vector3 v3 = new(Min.X, Max.Y, Min.Z);
            Vector3 v4 = new(Min.X, Min.Y, Max.Z);
            Vector3 v5 = new(Max.X, Min.Y, Max.Z);
            Vector3 v6 = new(Max.X, Max.Y, Max.Z);
            Vector3 v7 = new(Min.X, Max.Y, Max.Z);
            Vector3[] vertices = { v0, v1, v2, v3, v4, v5, v6, v7 };

            // Right-handed, CCW outward
            TriFace[] faces = {
                // bottom (z = Min.Z)
                new(0, 2, 1), new(0, 3, 2),
                // top    (z = Max.Z)
                new(4, 5, 6), new(4, 6, 7),
                // y- (front)
                new(0, 5, 4), new(0, 1, 5),
                // y+ (back)
                new(2, 7, 6), new(2, 3, 7),
                // x- (left)
                new(0, 7, 3), new(0, 4, 7),
                // x+ (right)
                new(1, 6, 5), new(1, 2, 6)
            };

            if (cordSystem == CordSystem.LeftHanded)
            {
                // Flip winding by swapping B and C
                for (int i = 0; i < faces.Length; i++)
                {
                    TriFace f = faces[i];
                    faces[i] = new TriFace(f.A, f.C, f.B);
                }
            }

            return new MeshF(vertices, faces);
        }

        /// <summary>
        /// Write all 8 corners to a Span (length >= 8). Order: (x,y,z)
        /// </summary>
        public readonly void GetAllCorners(Span<Vector3> dst)
        {
            dst[0] = new Vector3(Min.X, Min.Y, Min.Z);
            dst[1] = new Vector3(Max.X, Min.Y, Min.Z);
            dst[2] = new Vector3(Min.X, Max.Y, Min.Z);
            dst[3] = new Vector3(Max.X, Max.Y, Min.Z);
            dst[4] = new Vector3(Min.X, Min.Y, Max.Z);
            dst[5] = new Vector3(Max.X, Min.Y, Max.Z);
            dst[6] = new Vector3(Min.X, Max.Y, Max.Z);
            dst[7] = new Vector3(Max.X, Max.Y, Max.Z);
        }

        // ==================
        // Private Methods
        // ==================
        /// <summary>
        /// Checks if the bounding box is degenerate (invalid).
        /// </summary>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private readonly bool IsDegenerate()
        {
            return Min.X > Max.X || Min.Y > Max.Y || Min.Z > Max.Z;
        }
    }
}
