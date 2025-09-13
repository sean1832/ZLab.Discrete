using System.Numerics;
using System.Text;
using ZLab.Discrete.Geometry;
using static System.Net.Mime.MediaTypeNames;

namespace DiscreteTests.MeshFTests
{
    public class MeshF_FromObjFile_Tests
    {

        // --- helpers ---
        private static string TempFile()
            => Path.Combine(Path.GetTempPath(), Path.GetRandomFileName());

        private static void WriteText(string path, string text, bool withBom = false)
        {
            using StreamWriter sw = new(path, false, new UTF8Encoding(withBom));
            sw.NewLine = "\n";
            sw.Write(text);
        }

        private static void SafeDelete(string? path)
        {
            try { if (!string.IsNullOrEmpty(path) && File.Exists(path)) File.Delete(path); }
            catch { /* ignore for tests */ }
        }

        // --- tests ---

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

            string path = TempFile();
            WriteText(path, obj);

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

            string path = TempFile();
            WriteText(path, obj);

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
            string path = TempFile();
            WriteText(path, obj);

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
        [Fact]
        public void FromObjFile_Loads_WithTabs_And_InlineComments_And_Ignores_Optional_W()
        {
            string path = TempFile();
            try
            {
                string obj = """
                             # mixed whitespace and comments
                                v	0	0	0	1   # w ignored
                             v 1  0   0
                             v 1  1   0   1
                             v 0  1   0
                             v 0.5 0.5 1.6
                             f  5 1 2   # tri
                             f  5 2 3
                             f  5 3 4
                             f  5 4 1
                             f  1 3 2
                             f  1 4 3
                             """;
                WriteText(path, obj);

                MeshF mesh = MeshF.FromObjFile(path);

                Assert.Equal(5, mesh.Vertices.Length);
                Assert.Equal(6, mesh.Faces.Length);
                Assert.True(mesh.IsValid);
            }
            finally { SafeDelete(path); }
        }

        [Fact]
        public void FromObjFile_Supports_Negative_Indices_Relative_To_End()
        {
            string path = TempFile();
            try
            {
                // 4 vertices; face uses -1,-2,-3 => v4,v3,v2
                string obj = """
                             v 0 0 0
                             v 1 0 0
                             v 1 1 0
                             v 0 1 0
                             f -1 -2 -3
                             """;
                WriteText(path, obj);

                MeshF mesh = MeshF.FromObjFile(path);
                Assert.Single(mesh.Faces);
                TriFace f = mesh.Faces[0];
                Assert.Equal(3, f.A); // last vertex (index 3)
                Assert.Equal(2, f.B); // third
                Assert.Equal(1, f.C); // second
            }
            finally { SafeDelete(path); }
        }

        [Fact]
        public void FromObjFile_Throws_On_Quad_Face()
        {
            string path = TempFile();
            try
            {
                string obj = """
                             v 0 0 0
                             v 1 0 0
                             v 1 1 0
                             v 0 1 0
                             f 1 2 3 4
                             """;
                WriteText(path, obj);

                Assert.Throws<NotSupportedException>(() => MeshF.FromObjFile(path));
            }
            finally { SafeDelete(path); }
        }

        [Fact]
        public void FromObjFile_Throws_On_Zero_Index()
        {
            string path = TempFile();
            try
            {
                string obj = """
                             v 0 0 0
                             v 1 0 0
                             v 1 1 0
                             f 0 2 3
                             """;
                WriteText(path, obj);

                FormatException ex = Assert.Throws<FormatException>(() => MeshF.FromObjFile(path));
                Assert.Contains("vertex index cannot be 0", ex.Message, StringComparison.OrdinalIgnoreCase);
            }
            finally { SafeDelete(path); }
        }

        [Fact]
        public void FromObjFile_Throws_On_OutOfRange_Index()
        {
            string path = TempFile();
            try
            {
                string obj = """
                             v 0 0 0
                             v 1 0 0
                             v 1 1 0
                             f 1 2 5
                             """;
                WriteText(path, obj);

                Assert.Throws<IndexOutOfRangeException>(() => MeshF.FromObjFile(path));
            }
            finally { SafeDelete(path); }
        }


        [Theory]
        [InlineData("v 0 0", "Invalid vertex line")]                 // too few coords
        [InlineData("v a b c", "Invalid vertex numbers")]             // not numbers
        [InlineData("f 1 2", "Only triangular faces")]                // too few
        public void Load_Throws_On_Malformed_Lines(string badLine, string expectMsgPart)
        {
            string path = TempFile();
            try
            {
                string obj = $"v 0 0 0\nv 1 0 0\nv 1 1 0\n{badLine}\n";
                WriteText(path, obj);
                var ex = Assert.ThrowsAny<Exception>(() => MeshF.FromObjFile(path));
                Assert.Contains(expectMsgPart, ex.Message, StringComparison.OrdinalIgnoreCase);
            }
            finally { SafeDelete(path); }
        }

        [Fact]
        public void Load_Ignores_Unknown_Records_And_Blank_Lines()
        {
            string path = TempFile();
            try
            {
                string obj = """
                             # header
                             o MyObject
                             g G1
                             s off

                             v 0 0 0
                             v 1 0 0
                             v 1 1 0
                             v 0 1 0

                             usemtl whatever
                             mtllib some.mtl

                             f 1 2 3
                             f 1 3 4
                             """;
                WriteText(path, obj);

                MeshF mesh = MeshF.FromObjFile(path);
                Assert.Equal(4, mesh.Vertices.Length);
                Assert.Equal(2, mesh.Faces.Length);
            }
            finally { SafeDelete(path); }
        }

        [Fact]
        public void ToObjFile_Overwrites_Existing_File()
        {
            string path = TempFile();
            try
            {
                File.WriteAllText(path, "garbage"); // pre-existing

                var mesh = new MeshF(
                    new[] { new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(0, 1, 0) },
                    new[] { new TriFace(0, 1, 2) }, false);

                mesh.ToObjFile(path);

                string content = File.ReadAllText(path);
                Assert.DoesNotContain("garbage", content);
                Assert.Contains("v 0 0 0", content);
                Assert.Contains("f 1 2 3", content);
            }
            finally { SafeDelete(path); }
        }

    }
}