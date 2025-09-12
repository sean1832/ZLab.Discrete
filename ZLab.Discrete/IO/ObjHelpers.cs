using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Numerics;
using System.Text;
using ZLab.Discrete.Geometry;

namespace ZLab.Discrete.IO
{
    internal static class ObjHelpers
    {
        /// <summary>
        /// Whitespace characters for splitting lines (space and tab).
        /// </summary>
        private static readonly char[] Ws = { ' ', '\t' };

        /// <summary>
        /// Loads a mesh from an OBJ file. Only supports vertices (v) and triangular faces (f).
        /// </summary>
        /// <param name="path">Path to the OBJ file.</param>
        /// <returns>Loaded mesh.</returns>
        /// <exception cref="NotSupportedException">Thrown if the file contains non-triangular faces.</exception>
        public static MeshF Load(string path)
        {
            List<Vector3> vertices = new();
            List<TriFace> faces = new();

            // TODO: use spans when available

            foreach (string raw in File.ReadLines(path))
            {
                // Trim leading whitespace and strip inline comments
                if (string.IsNullOrWhiteSpace(raw)) continue;
                string line = raw.TrimStart();
                int hash = line.IndexOf('#');
#if NET472_OR_GREATER
                if (hash >= 0)
                   line = line.Substring(0, hash);   // NET Framework friendly
#else
                if (hash >= 0) line = line[..hash]; // remove inline comment
#endif

                if (string.IsNullOrWhiteSpace(line)) continue;

                // Split on whitespace (space or tab)
                string[] parts = line.Split(Ws, StringSplitOptions.RemoveEmptyEntries);
                if (parts.Length == 0) continue;

                switch (parts[0])
                {
                    case "v":
                        if (parts.Length < 4)
                            throw new FormatException($"Malformed vertex line: '{raw}'");
                        // Accept optional w component (parts.Length >= 5). Ignore it.
                        float x = float.Parse(parts[1], CultureInfo.InvariantCulture);
                        float y = float.Parse(parts[2], CultureInfo.InvariantCulture);
                        float z = float.Parse(parts[3], CultureInfo.InvariantCulture);
                        vertices.Add(new Vector3(x, y, z));
                        break;

                    case "f":
                        // only support triangles by design.
                        if (parts.Length != 4)
                            throw new NotSupportedException("Only triangular faces are supported (exactly 3 vertices on 'f' line).");

                        int a = ParseFaceIndex(parts[1], vertices.Count, raw);
                        int b = ParseFaceIndex(parts[2], vertices.Count, raw);
                        int c = ParseFaceIndex(parts[3], vertices.Count, raw);
                        faces.Add(new TriFace(a, b, c));
                        break;
                }
            }

            return new MeshF(vertices.ToArray(), faces.ToArray());
        }

        /// <summary>
        /// Saves a mesh to an OBJ file. Only saves vertices (v) and triangular faces (f).
        /// </summary>
        /// <param name="path">Path to save the OBJ file.</param>
        /// <param name="mesh">Mesh to save.</param>
        public static void Save(string path, MeshF mesh)
        {
            // Overwrite without pre-delete; explicit UTF-8, no BOM
            using StreamWriter writer = new(path, false, new UTF8Encoding(encoderShouldEmitUTF8Identifier: false));

            foreach (Vector3 v in mesh.Vertices)
            {
                writer.Write("v ");
                writer.Write(v.X.ToString(CultureInfo.InvariantCulture));
                writer.Write(' ');
                writer.Write(v.Y.ToString(CultureInfo.InvariantCulture));
                writer.Write(' ');
                writer.Write(v.Z.ToString(CultureInfo.InvariantCulture));
                writer.WriteLine();
            }

            foreach (TriFace f in mesh.Faces)
            {
                // OBJ is 1-based
                writer.Write("f ");
                writer.Write((f.A + 1).ToString(CultureInfo.InvariantCulture));
                writer.Write(' ');
                writer.Write((f.B + 1).ToString(CultureInfo.InvariantCulture));
                writer.Write(' ');
                writer.Write((f.C + 1).ToString(CultureInfo.InvariantCulture));
                writer.WriteLine();
            }
        }

        private static int ParseFaceIndex(string token, int vertexCount, string rawLineForError)
        {
            // token can be "v", "v/vt", "v//vn", or "v/vt/vn"
            // We only use the vertex index (first field).
            if (string.IsNullOrWhiteSpace(token))
                throw new FormatException($"Empty face token in line: '{rawLineForError}'");

            string[] sub = token.Split('/');
            if (sub.Length == 0 || string.IsNullOrEmpty(sub[0]))
                throw new FormatException($"Missing vertex index in face token '{token}' (line: '{rawLineForError}')");

            int idx = int.Parse(sub[0], NumberStyles.Integer, CultureInfo.InvariantCulture);

            // Negative indices are relative to the end; -1 is last vertex.
            idx = idx < 0 ? vertexCount + idx : idx - 1;

            if ((uint)idx >= (uint)vertexCount) // unsigned trick for single bound check
                throw new IndexOutOfRangeException($"Face index {idx} out of range [0,{vertexCount - 1}] (line: '{rawLineForError}')");

            return idx;
        }
    }
}
