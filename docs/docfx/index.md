# Zlab.Discrete
Library for voxelization, meshing, and distance fields in managed C#.

## Overview
Zlab.Discrete is a fully managed C# library that provides a collection of voxelization algorithms for 3D meshes. It focuses on ease of use while offering flexibility through different voxelization techniques.
It is not as performant as some native libraries, but it is easier to integrate into C# projects and it tries its best to be performant for a managed library.

## Features
- Anisotropic Voxelization (non-uniform voxel sizes)
- Dense occupancy grid representation
- Dense distance grid representation (Euclidean distance transform)
- Signed Distance Field (SDF) generation
- Sampling SDF with nearest neighbor or trilinear interpolation
- Gradient sampling from SDF
- Sparse voxelization (boundary voxels only)
- Meshing using face-culling algorithms
- BFS Flood fill algorithms for interior voxel identification

## Supported Frameworks
- .NET Framework 4.7.2
- .NET Framework 4.8
- .NET Standard 2.1
- .NET 7.0
- .NET 8.0
- .NET 9.0