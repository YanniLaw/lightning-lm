# Scan Context core

Source: https://github.com/gisbi-kim/scancontext

Revision: `93672835bb85e9c03fedf0fbaf2d0491df12161e`

The upstream Scan Context code is attributed to KAIST and Naver Labs and is
licensed under CC BY-NC-SA 4.0; see `LICENSE`. The bundled nanoflann and its
KD-tree adaptor retain their own BSD license notices in the original headers.

This directory retains the original polar maximum-height descriptor, ring and
sector keys, KD-tree candidate search, circular sector alignment, and cosine
distance. The application performs NDT registration and graph optimization.

Integration changes:

- Remove ROS 1, OpenCV and per-query timing output; place symbols in a namespace.
- Make descriptor and retrieval settings configurable and cache sector keys.
- Search the caller's eligible history before adding the current descriptor;
  build the KD-tree from a stable prefix and cap K to the available entries.
- Handle empty descriptors and sector comparisons, non-finite points, and
  `atan2(0, 0)` safely. Return the descriptor distance and sector shift.

The adapter in `../scan_context_descriptor.cc` handles level alignment, actual
backend keyframe IDs and the rotation from query IMU to reference IMU.
