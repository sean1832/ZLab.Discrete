using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using ZLab.Discrete.Geometry;

namespace ZLab.Discrete.IO
{
    internal static class ObjParser
    {
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

            foreach (string raw in File.ReadLines(path))
            {
                // Trim leading whitespace and strip inline comments
                if (string.IsNullOrWhiteSpace(raw)) continue;
                ReadOnlySpan<char> line = raw.AsSpan();
                line = TrimStart(line);

                // Strip comments (starting with #)
                int hash = line.IndexOf('#');
                if (hash >= 0)
                    line = line[..hash];
                line = Trim(line);
                if (line.IsEmpty) continue;

                // first token: "v" or "f" (others ignored)
                ReadOnlySpan<char> first = NextToken(ref line);
                if (first.IsEmpty) continue;

                if (first.Length != 1) continue; // ignore multi-char tokens

                char kind = first[0];
                switch (kind)
                {
                    case 'v':
                    {
                        // v x y z [w]
                        ReadOnlySpan<char> xTok = NextToken(ref line);
                        ReadOnlySpan<char> yTok = NextToken(ref line);
                        ReadOnlySpan<char> zTok = NextToken(ref line);
                        if (xTok.IsEmpty || yTok.IsEmpty || zTok.IsEmpty)
                            throw new FormatException($"Invalid vertex line (expected at least 3 coordinates): '{raw}'");

                        if (!float.TryParse(xTok, NumberStyles.Float, CultureInfo.InvariantCulture, out float x) ||
                            !float.TryParse(yTok, NumberStyles.Float, CultureInfo.InvariantCulture, out float y) ||
                            !float.TryParse(zTok, NumberStyles.Float, CultureInfo.InvariantCulture, out float z))
                        {
                            throw new FormatException($"Invalid vertex numbers in line: '{raw}'");
                        }

                        // Ignore optional w coordinate
                        vertices.Add(new Vector3(x, y, z));
                        continue;
                    }
                    case 'f':
                    {
                        // f a b c ... (only triangles supported)
                        ReadOnlySpan<char> aTok = NextToken(ref line);
                        ReadOnlySpan<char> bTok = NextToken(ref line);
                        ReadOnlySpan<char> cTok = NextToken(ref line);

                        // ensure exactly 3 and no trailing tokens
                        if (aTok.IsEmpty || bTok.IsEmpty || cTok.IsEmpty || !NextToken(ref line).IsEmpty)
                            throw new NotSupportedException("Only triangular faces are supported (exactly 3 vertices on 'f' line).");

                        int a = ParseFaceIndex(aTok, vertices.Count, raw);
                        int b = ParseFaceIndex(bTok, vertices.Count, raw);
                        int c = ParseFaceIndex(cTok, vertices.Count, raw);

                        faces.Add(new TriFace(a, b, c));
                        continue;
                    }
                }
                // ignore other kinds
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
            // Overwrite; UTF-8 no BOM; normalized LF line endings
            using StreamWriter writer = new(path, append: false, new UTF8Encoding(encoderShouldEmitUTF8Identifier: false));
            writer.NewLine = "\n";

            // Reusable number buffer; 64 chars is plenty for float/int in "R"
            Span<char> num = stackalloc char[64];

            // Write vertices
            foreach (Vector3 vertex in mesh.Vertices)
            {
                // v x y z
                writer.Write("v ");
                if (!vertex.X.TryFormat(num, out int w, format: "R", provider: CultureInfo.InvariantCulture))
                    throw new IOException("Failed to format vertex X.");
                writer.Write(num[..w]);
                writer.Write(' ');

                if (!vertex.Y.TryFormat(num, out w, format: "R", provider: CultureInfo.InvariantCulture))
                    throw new IOException("Failed to format vertex Y.");
                writer.Write(num[..w]);
                writer.Write(' ');

                if (!vertex.Z.TryFormat(num, out w, format: "R", provider: CultureInfo.InvariantCulture))
                    throw new IOException("Failed to format vertex Z.");
                writer.Write(num[..w]);

                writer.WriteLine();
            }

            // Write faces
            foreach (TriFace face in mesh.Faces)
            {
                // f a b c  (1-based indices)
                writer.Write("f ");

                int a1 = face.A + 1;
                if (!a1.TryFormat(num, out int w, provider: CultureInfo.InvariantCulture))
                    throw new IOException("Failed to format face index A.");
                writer.Write(num[..w]);
                writer.Write(' ');

                int b1 = face.B + 1;
                if (!b1.TryFormat(num, out w, provider: CultureInfo.InvariantCulture))
                    throw new IOException("Failed to format face index B.");
                writer.Write(num[..w]);
                writer.Write(' ');

                int c1 = face.C + 1;
                if (!c1.TryFormat(num, out w, provider: CultureInfo.InvariantCulture))
                    throw new IOException("Failed to format face index C.");
                writer.Write(num[..w]);

                writer.WriteLine();
            }
        }

        /// <summary>
        /// Parse OBJ face vertex token like "12/5/7" or "12//7" or "12" and return 0-based vertex index.
        /// Supports negative indices (relative to end). Disallows 0.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static int ParseFaceIndex(ReadOnlySpan<char> token, int vertexCount, string rawLineForError)
        {
            if (token.IsEmpty)
                throw new FormatException($"Empty face token in line: '{rawLineForError}'");

            int slash = token.IndexOf('/');
            ReadOnlySpan<char> vertSpan = (slash >= 0) ? token[..slash] : token;

            if (vertSpan.IsEmpty)
                throw new FormatException($"Missing vertex index in face token '{token.ToString()}' (line: '{rawLineForError}')");

            if (!int.TryParse(vertSpan, NumberStyles.Integer, CultureInfo.InvariantCulture, out int original))
                throw new FormatException($"Invalid vertex index '{vertSpan.ToString()}' (line: '{rawLineForError}')");

            if (original == 0)
                throw new FormatException($"OBJ vertex index cannot be 0 (line: '{rawLineForError}')");

            // Convert to 0-based:
            // positive: 1-based -> 0-based
            // negative: relative to end (e.g. -1 => last => vertexCount-1)
            int idx = (original > 0) ? (original - 1) : (vertexCount + original);

            if ((uint)idx >= (uint)vertexCount)
                throw new IndexOutOfRangeException(
                    $"Face vertex index '{original}' resolved to 0-based {idx}, " +
                    $"but valid range is [0,{vertexCount - 1}] (line: '{rawLineForError}')");

            return idx;
        }

        // ================================
        // SPAN TOKENIZER
        // ================================
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static ReadOnlySpan<char> NextToken(ref ReadOnlySpan<char> line)
        {
            line = TrimStart(line);
            if (line.IsEmpty)
                return ReadOnlySpan<char>.Empty;
            int i = 0;
            // find next whitespace
            while (i < line.Length && !IsSpace(line[i]))
                i++;
            ReadOnlySpan<char> token = line[..i]; // extract token
            line = (i < line.Length) ? line[i..] : ReadOnlySpan<char>.Empty; // advance
            return token; // return token
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static ReadOnlySpan<char> Trim(ReadOnlySpan<char> s)
        {
            int start = 0;
            int end = s.Length - 1;
            while (start <= end && IsSpace(s[start]))
                start++; // trim start
            while (end >= start && IsSpace(s[end]))
                end--;   // trim end
            return (start <= end) ? s.Slice(start, end - start + 1) : ReadOnlySpan<char>.Empty;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static ReadOnlySpan<char> TrimStart(ReadOnlySpan<char> s)
        {
            int start = 0;
            while (start < s.Length && IsSpace(s[start]))
                start++;
            return (start == 0) ? s : s[start..];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool IsSpace(char c) => c == ' ' || c == '\t';
    }
}
