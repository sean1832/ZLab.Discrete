using System.Numerics;
using ZLab.Discrete.Geometry;

namespace ZLab.Discrete.Voxels
{
    public readonly struct OccupancyVoxel
    {
        public readonly Vector3 Origin;
        public readonly Occupancy State;
        public OccupancyVoxel(Vector3 origin, Occupancy state)
        {
            Origin = origin;
            State = state;
        }
        public BBox GetBounds(Vector3 size)
        {
            return new BBox(Origin, Origin + size);
        }

        public override string ToString() => $"origin: {Origin}, state: {State}";
    }
}
