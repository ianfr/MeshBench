#pragma once

// Procedural cube builders shared by the MeshBench unit tests.

#include "MeshTypes.h"

namespace meshbench {
namespace testutil {

// Closed unit cube [0,1]^3, outward oriented, 8 verts / 12 faces.
inline Mesh makeClosedCube() {
  Mesh m;
  Mesh::Vertex_index v[8];
  v[0] = m.add_vertex(makePoint(0, 0, 0));
  v[1] = m.add_vertex(makePoint(1, 0, 0));
  v[2] = m.add_vertex(makePoint(1, 1, 0));
  v[3] = m.add_vertex(makePoint(0, 1, 0));
  v[4] = m.add_vertex(makePoint(0, 0, 1));
  v[5] = m.add_vertex(makePoint(1, 0, 1));
  v[6] = m.add_vertex(makePoint(1, 1, 1));
  v[7] = m.add_vertex(makePoint(0, 1, 1));
  const int tris[12][3] = {
      {0, 2, 1}, {0, 3, 2}, // bottom
      {4, 5, 6}, {4, 6, 7}, // top
      {0, 1, 5}, {0, 5, 4}, // front
      {2, 3, 7}, {2, 7, 6}, // back
      {0, 4, 7}, {0, 7, 3}, // left
      {1, 2, 6}, {1, 6, 5}, // right
  };
  for (const auto& t : tris)
    m.add_face(v[t[0]], v[t[1]], v[t[2]]);
  return m;
}

// Same cube with the top face removed: one 4-edge hole loop.
inline Mesh makeOpenCube() {
  Mesh m;
  Mesh::Vertex_index v[8];
  v[0] = m.add_vertex(makePoint(0, 0, 0));
  v[1] = m.add_vertex(makePoint(1, 0, 0));
  v[2] = m.add_vertex(makePoint(1, 1, 0));
  v[3] = m.add_vertex(makePoint(0, 1, 0));
  v[4] = m.add_vertex(makePoint(0, 0, 1));
  v[5] = m.add_vertex(makePoint(1, 0, 1));
  v[6] = m.add_vertex(makePoint(1, 1, 1));
  v[7] = m.add_vertex(makePoint(0, 1, 1));
  const int tris[10][3] = {
      {0, 2, 1}, {0, 3, 2}, // bottom
      {0, 1, 5}, {0, 5, 4}, // front
      {2, 3, 7}, {2, 7, 6}, // back
      {0, 4, 7}, {0, 7, 3}, // left
      {1, 2, 6}, {1, 6, 5}, // right
  };
  for (const auto& t : tris)
    m.add_face(v[t[0]], v[t[1]], v[t[2]]);
  return m;
}

} // namespace testutil
} // namespace meshbench
