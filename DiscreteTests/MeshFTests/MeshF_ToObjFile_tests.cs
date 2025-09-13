using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using ZLab.Discrete.Geometry;

namespace DiscreteTests.MeshFTests
{
    public class MeshF_ToObjFile_tests
    {
        // --- Helpers -------------------------------------------------------------

        private static string TempFile()
            => Path.Combine(Path.GetTempPath(), Path.GetRandomFileName());

        private static void SafeDelete(string path)
        {
            try { if (File.Exists(path)) File.Delete(path); } catch { }
        }

        private static void WriteText(string path, string text, bool withBom = false)
        {
            using var sw = new StreamWriter(path, false, new UTF8Encoding(withBom));
            sw.NewLine = "\n";
            sw.Write(text);
        }

        [Fact]
        public void ToObjFile_Pyramid_With5VerticesAnd6Faces()
        {
            // Arrange
            Vector3[] vertices =
            [
                new Vector3(0, 0, 0),
                new Vector3(1, 0, 0),
                new Vector3(1, 1, 0),
                new Vector3(0, 1, 0),
                new Vector3(0.5f, 0.5f, 1.6f)
            ];

            TriFace[] faces =
            [
                new TriFace(4, 0, 1),
                new TriFace(4, 1, 2),
                new TriFace(4, 2, 3),
                new TriFace(4, 3, 0),
                new TriFace(0, 2, 1),
                new TriFace(0, 3, 2)
            ];
            MeshF mesh = new(vertices, faces, isClosed: true);
            string path = Path.Combine(Path.GetTempPath(), Path.GetRandomFileName());
            try
            {
                // Act
                mesh.ToObjFile(path);

                // Assert
                string[] lines = File.ReadAllLines(path);
                Assert.Contains("v 0 0 0", lines);
                Assert.Contains("v 1 0 0", lines);
                Assert.Contains("v 1 1 0", lines);
                Assert.Contains("v 0 1 0", lines);
                Assert.Contains("v 0.5 0.5 1.6", lines);
                Assert.Contains("f 5 1 2", lines);
                Assert.Contains("f 5 2 3", lines);
                Assert.Contains("f 5 3 4", lines);
                Assert.Contains("f 5 4 1", lines);
                Assert.Contains("f 1 3 2", lines);
                Assert.Contains("f 1 4 3", lines);
            }
            finally
            {
                SafeDelete(path);
            }
        }

        [Fact]
        public void ToObjFile_WritesUtf8_NoBom_And_LF_Only()
        {
            // Arrange
            var mesh = new MeshF(
                new[] { new Vector3(0, 0, 0), new Vector3(1, 2, 3), new Vector3(4, 5, 6) },
                new[] { new TriFace(0, 1, 2) },
                isClosed: false);

            string path = TempFile();
            try
            {
                // Act
                mesh.ToObjFile(path);

                // Assert: no BOM
                byte[] bytes = File.ReadAllBytes(path);
                var utf8Bom = new byte[] { 0xEF, 0xBB, 0xBF };
                Assert.False(bytes.Length >= 3 &&
                             bytes[0] == utf8Bom[0] &&
                             bytes[1] == utf8Bom[1] &&
                             bytes[2] == utf8Bom[2]);

                // Assert: LF only
                string text = File.ReadAllText(path); // normalizes lines in memory, so check raw bytes for CR
                Assert.DoesNotContain("\r", Encoding.UTF8.GetString(bytes));
            }
            finally { SafeDelete(path); }
        }

        [Fact]
        public void ToObjFile_IsCultureInvariant()
        {
            var originalCulture = CultureInfo.CurrentCulture;
            try
            {
                CultureInfo.CurrentCulture = new CultureInfo("de-DE"); // comma decimal locale

                var mesh = new MeshF(
                    new[] { new Vector3(0.5f, 1.25f, 2.5f) },
                    new[] { new TriFace(0, 0, 0) },
                    isClosed: false);

                string path = TempFile();
                try
                {
                    mesh.ToObjFile(path);
                    string content = File.ReadAllText(path);

                    // Should use '.' regardless of current culture
                    Assert.Contains("v 0.5 1.25 2.5", content);
                    Assert.Contains("f 1 1 1", content);
                }
                finally { SafeDelete(path); }
            }
            finally
            {
                CultureInfo.CurrentCulture = originalCulture;
            }
        }
    }
}
