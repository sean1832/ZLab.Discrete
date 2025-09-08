namespace ZLab.Discrete.Operations.Meshing
{
    /// <summary>
    /// Coordinate system type used in meshing operations.
    /// </summary>
    public enum CordSystem: byte
    {
        /// <summary>
        /// OpenGL and DirectX use right-handed coordinate systems
        /// </summary>
        RightHanded = 0,

        /// <summary>
        /// Rhino3D uses left-handed coordinate system
        /// </summary>
        LeftHanded = 1
    }
}
