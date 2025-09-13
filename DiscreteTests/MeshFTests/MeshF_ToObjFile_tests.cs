using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using ZLab.Discrete.Geometry;

namespace DiscreteTests.MeshFTests
{
    public class MeshF_ToObjFile_tests
    {
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

        private static void SafeDelete(string path)
        {
            try
            {
                if (File.Exists(path))
                    File.Delete(path);
            }
            catch
            {
                /* ignore for tests */
            }
        }
    }
}
