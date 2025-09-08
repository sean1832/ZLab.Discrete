using System.Numerics;
using System.Runtime.CompilerServices;
using ZLab.Discrete.Geometry;

namespace ZLab.Discrete.Voxels
{
    public readonly struct DistanceVoxel
    {
        public readonly Vector3 Origin; // min corner
        public readonly float Distance;

        public DistanceVoxel(Vector3 origin, float distance)
        {
            Origin = origin;
            Distance = distance;
        }

        public float[] ToArray() => new float[] { Origin.X, Origin.Y, Origin.Z, Distance };

        public BBox ComputeBounds(Vector3 size)
        {
            return new BBox(Origin, Origin + size);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3 Center(Vector3 size) => Origin + 0.5f * size;

        public override string ToString() => $"origin: {Origin}, dist: {Distance}";
    }
}
