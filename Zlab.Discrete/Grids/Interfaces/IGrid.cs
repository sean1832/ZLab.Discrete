using System;

namespace ZLab.Discrete.Grids.Interfaces
{
    public interface IGrid<T>
    {
        T GetValue((int x, int y, int z) index);
        void SetValue((int x, int y, int z) index, T value);
        ReadOnlySpan<T> GetRawBuffer();
    }
}
