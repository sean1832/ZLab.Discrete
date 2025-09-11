# Zlab.Discrete
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="icon_dark.png">
  <source media="(prefers-color-scheme: light)" srcset="icon_light.png">
  <img alt="Icon" src="icon_light.png" width="200">
</picture>


A fully managed .NET library for voxelization, meshing, and distance field computation.

- [Documentation](https://sean1832.github.io/ZLab.Discrete/)

## Overview

**Zlab.Discrete** provides tools for constructing and processing voxel grids from 3D geometry. It is written entirely in C#, requires no external dependencies, and is designed for easy integration into .NET projects.

## Features

**Voxelization**
  - Anisotropic voxelization (non-uniform voxel sizes)
  - Dense occupancy grids
  - Sparse boundary voxelization

**Distance Fields**
  - Euclidean distance transform (exact, isotropic and anisotropic)
  - Signed distance field (SDF) generation
  - Sampling with nearest-neighbor or trilinear interpolation
  - Gradient computation from SDFs

**Meshing**
  - Face-culling voxel meshing

**Grid Processing**
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



## License
This project is dual-licensed:

- **Open Source (GPL 3.0)** — see [LICENSE](./LICENSE)  
- **Commercial License** — see [LICENSE-COMMERCIAL](./LICENSE-COMMERCIAL).

If you are a researcher, student, or using this project in an open-source context,  
use the GPL 3.0 license.  

If you want to use this in proprietary software or closed-source products,  
you must obtain a commercial license. (contact: dev@zekezhang.com)