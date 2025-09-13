using ZLab.Discrete.Geometry;

namespace DiscreteTests.MeshFTests
{
    public class MeshF_FromObjFile_Tests
    {
        [Fact]
        public void FromObjFile_Pyramid_IsClosed_With5VerticesAnd6Faces()
        {
            // Arrange
            const string obj = """
            # Pyramid: 5 vertices, 6 faces, correct windings
            v  0   0   0
            v  1   0   0
            v  1   1   0
            v  0   1   0
            v  0.5 0.5 1.6

            f  5 1 2
            f  5 2 3
            f  5 3 4
            f  5 4 1
            f  1 3 2
            f  1 4 3
            """;

            string path = CreateTempFile(obj);

            try
            {
                // Act
                MeshF mesh = MeshF.FromObjFile(path);

                // Assert
                Assert.Equal(5, mesh.Vertices.Length);
                Assert.Equal(6, mesh.Faces.Length);
                Assert.True(mesh.IsClosed);
            }
            finally
            {
                SafeDelete(path);
            }
        }

        [Fact]
        public void FromObjFile_FaceReferencesMissingVertex_Throws()
        {
            // Arrange
            const string obj = """
            v 0 0 0
            v 1 0 0
            v 1 1 0
            f 1 2 4   # 4 does not exist
            """;
            string path = CreateTempFile(obj);

            try
            {
                // Act + Assert
                var ex = Assert.Throws<IndexOutOfRangeException>(() => MeshF.FromObjFile(path));
                Assert.Contains("Face vertex index '4' resolved to 0-based", ex.Message, StringComparison.OrdinalIgnoreCase);
            }
            finally
            {
                SafeDelete(path);
            }
        }

        [Fact]
        public void FromObjFile_QuadFace_ThrowsNotSupported()
        {
            // Arrange
            const string obj = """
            v 0 0 0
            v 1 0 0
            v 1 1 0
            v 0 1 0
            f 1 2 3 4
            """;
            string path = CreateTempFile(obj);

            try
            {
                // Act + Assert
                Assert.Throws<NotSupportedException>(() => MeshF.FromObjFile(path));
            }
            finally
            {
                SafeDelete(path);
            }
        }

        // --- helpers ---
        private static string CreateTempFile(string content)
        {
            string path = Path.Combine(Path.GetTempPath(), Path.GetRandomFileName());
            File.WriteAllText(path, content);
            return path;
        }

        private static void SafeDelete(string? path)
        {
            try { if (!string.IsNullOrEmpty(path) && File.Exists(path)) File.Delete(path); }
            catch { /* ignore for tests */ }
        }
    }
}