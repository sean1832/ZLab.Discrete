# Zlab.Discrete
![Image](Zlab.Discrete/images/icon_dark_trans.png)
A fully managed .NET library for voxelization, meshing, and distance field computation.

## Overview

**Zlab.Discrete** provides tools for constructing and processing voxel grids from 3D geometry. It is written entirely in C#, requires no external dependencies, and is designed for easy integration into .NET projects.

## Features

- **Voxelization**
  - Anisotropic voxelization (non-uniform voxel sizes)
  - Dense occupancy grids
  - Sparse boundary voxelization

- **Distance Fields**
  - Euclidean distance transform (exact, isotropic and anisotropic)
  - Signed distance field (SDF) generation
  - Sampling with nearest-neighbor or trilinear interpolation
  - Gradient computation from SDFs

- **Meshing**
  - Face-culling voxel meshing

- **Grid Processing**
  - BFS-based flood fill for interior/exterior classification

## Use Cases
- Preprocessing 3D meshes into voxel grids
- Generating SDFs for geometry processing, physics, or rendering
- Mesh extraction from voxel volumes
- Spatial analysis and computational geometry research


## Supported Frameworks
- .NET Framework 4.7.2
- .NET Framework 4.8
- .NET Standard 2.1
- .NET 7.0
- .NET 8.0
- .NET 9.0