# STD core

Source: https://github.com/hku-mars/STD
Revision: `6e5903cfa2aaed5ca3c9e7d21424eebcca7655bb`
Authors: Chongjian Yuan, Jiarong Lin and upstream contributors.
License: GPL-2.0; the upstream license is preserved in `LICENSE`.

This directory contains the upstream descriptor extraction, hashed triangle
retrieval and plane-based candidate verification. The application uses its
existing NDT and Miao graph optimizer for final registration and optimization.

Port changes:
- Remove ROS 1 parameter loading, visualization, pose-file helpers and the
  optional Ceres plane-ICP refinement; no ROS 1/Ceres/GTSAM dependency is needed.
- Isolate symbols in `lightning::std_descriptor` and use explicit hash functors.
- Size frame voting storage from the indexed plane history instead of 20,000.
- Free each voxel's owned plane and allocate projection images on the heap.
- Respect neighbor counts and projection-image boundaries for sparse clouds;
  empty feature clouds return no match.
- Remove the unused random-number draw in input downsampling.

The algorithm's original naming/layout is otherwise retained for comparison.
Integration and runtime settings live in
`src/core/descriptor/std_descriptor.cc` and the descriptor factory.
