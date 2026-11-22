// Unit tests for the CGAL mesh operations (MeshOps).
//
// One focused test per feature: centering, slicing, booleans, alpha
// wrapping, plus a Gmsh availability check when compiled with Gmsh.

#include "MeshDocument.h"
#include "MeshOps.h"
#include "OperationSearch.h"
#include "ViewOrientations.h"
#include "test_utils.h"

#include <CGAL/Polygon_mesh_processing/measure.h>

#include <QDir>
#include <QFile>
#include <QTemporaryDir>
#include <QtTest>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

using namespace meshbench;
namespace PMP = CGAL::Polygon_mesh_processing;

static Mesh translatedCube(double dx, double dy, double dz) {
  Mesh m = testutil::makeClosedCube();
  for (auto v : m.vertices()) {
    Point3 p = m.point(v);
    m.point(v) = makePoint(toDouble(p.x()) + dx, toDouble(p.y()) + dy,
                           toDouble(p.z()) + dz);
  }
  return m;
}

// Cube whose top face is a fan around a point near the front edge; the small
// fan triangle {A,B,O} is replaced by a raised bump of three fine triangles.
// Autoclean should remove the bump and re-fill the triangular hole.
static Mesh makeBumpyCube() {
  Mesh m;
  Mesh::Vertex_index v[8];
  v[0] = m.add_vertex(makePoint(0, 0, 0));
  v[1] = m.add_vertex(makePoint(1, 0, 0));
  v[2] = m.add_vertex(makePoint(1, 1, 0));
  v[3] = m.add_vertex(makePoint(0, 1, 0));
  v[4] = m.add_vertex(makePoint(0, 0, 1)); // A
  v[5] = m.add_vertex(makePoint(1, 0, 1)); // B
  v[6] = m.add_vertex(makePoint(1, 1, 1)); // C
  v[7] = m.add_vertex(makePoint(0, 1, 1)); // D
  const auto O = m.add_vertex(makePoint(0.5, 0.1, 1));
  const auto apex = m.add_vertex(makePoint(0.4, 0.05, 1.1));

  // Bottom and sides (top face is rebuilt as a fan below).
  const int tris[10][3] = {
      {0, 2, 1}, {0, 3, 2},           // bottom
      {0, 1, 5}, {0, 5, 4},           // front
      {2, 3, 7}, {2, 7, 6},           // back
      {0, 4, 7}, {0, 7, 3},           // left
      {1, 2, 6}, {1, 6, 5},           // right
  };
  for (const auto& t : tris)
    m.add_face(v[t[0]], v[t[1]], v[t[2]]);

  const auto A = v[4];
  const auto B = v[5];
  const auto C = v[6];
  const auto D = v[7];
  // Top fan around O. {A,B,O} is the small hole covered by the bump.
  m.add_face(B, C, O);
  m.add_face(C, D, O);
  m.add_face(D, A, O);
  // Raised fine bump on the {A,B,O} hole.
  m.add_face(A, B, apex);
  m.add_face(B, O, apex);
  m.add_face(O, A, apex);
  return m;
}

class TstMeshOps : public QObject {
  Q_OBJECT
private slots:
  void centerMovesCentroidToOrigin() {
    Mesh m = translatedCube(5.0, -3.0, 2.0);
    QString error;
    QVERIFY2(centerAtVolumeCentroid(m, error), qPrintable(error));
    const MeshStats s = computeStats(m);
    QVERIFY(s.valid);
    const double cx = (s.bboxMin[0] + s.bboxMax[0]) / 2;
    const double cy = (s.bboxMin[1] + s.bboxMax[1]) / 2;
    const double cz = (s.bboxMin[2] + s.bboxMax[2]) / 2;
    QVERIFY(std::abs(cx) < 1e-6);
    QVERIFY(std::abs(cy) < 1e-6);
    QVERIFY(std::abs(cz) < 1e-6);
    // Unit cube volume preserved.
    QVERIFY(std::abs(std::abs(s.volume) - 1.0) < 1e-6);
  }

  void sliceHalvesVolume() {
    const Mesh cube = testutil::makeClosedCube();
    Mesh out;
    QString error;
    QVERIFY2(clipWithPlane(cube, makePoint(0.5, 0.0, 0.0),
                           Vector3(1, 0, 0), out, error),
             qPrintable(error));
    const MeshStats s = computeStats(out);
    QVERIFY(s.valid);
    QVERIFY(std::abs(std::abs(s.volume) - 0.5) < 1e-6);
    QVERIFY(s.bboxMax[0] <= 0.5 + 1e-9);
  }

  void booleanUnionAndIntersection() {
    const Mesh a = testutil::makeClosedCube();
    const Mesh b = translatedCube(0.5, 0.0, 0.0); // overlap volume 0.5
    QString error;
    Mesh u, i;
    QVERIFY2(booleanOp(a, b, BooleanOp::Union, u, error), qPrintable(error));
    QVERIFY2(booleanOp(a, b, BooleanOp::Intersection, i, error),
             qPrintable(error));
    const MeshStats su = computeStats(u);
    const MeshStats si = computeStats(i);
    QVERIFY(su.closed && su.boundaryEdges == 0);
    QVERIFY(si.closed && si.boundaryEdges == 0);
    QVERIFY(std::abs(std::abs(su.volume) - 1.5) < 1e-6);
    QVERIFY(std::abs(std::abs(si.volume) - 0.5) < 1e-6);
  }

  // Pipeline stages default to automatic opacity (opaque when active,
  // faint otherwise); the inspector pins an explicit value per stage.
  void pipelineStageOpacityDefaultsToAuto() {
    PipelineNode node;
    QVERIFY(node.opacity < 0.0);
    node.opacity = 0.5;
    QVERIFY(node.opacity == 0.5);
  }

  void alphaWrapParamsAreRelativeToDiag() {
    double alpha = 0.0, offset = 0.0;
    QString error;
    const double diag = std::sqrt(3.0); // unit cube diagonal
    QVERIFY(alphaWrapParams(diag, 20.0, 1000.0, alpha, offset, error));
    QVERIFY(std::abs(alpha - diag / 20.0) < 1e-12);
    QVERIFY(std::abs(offset - diag / 1000.0) < 1e-12);
    QVERIFY(!alphaWrapParams(diag, 0.0, 1000.0, alpha, offset, error));
    QVERIFY(!alphaWrapParams(diag, 20.0, -1.0, alpha, offset, error));
    QVERIFY(!alphaWrapParams(0.0, 20.0, 1000.0, alpha, offset, error));
  }

  void alphaWrapClosesOpenCube() {
    const Mesh open = testutil::makeOpenCube();
    Mesh out;
    QString error;
    // Loose wrap: alpha larger than the hole, offset above faceting noise.
    QVERIFY2(alphaWrap(open, 0.6, 0.15, out, error), qPrintable(error));
    const MeshStats s = computeStats(out);
    QVERIFY(s.valid);
    QVERIFY2(s.closed, "alpha wrap must return a watertight mesh");
    QVERIFY(std::abs(s.volume) > 0.5); // roughly the unit cube
  }

  void isotropicRemeshRefinesAndPreservesCubeFeatures() {
    const Mesh cube = testutil::makeClosedCube(); // [0,1]^3
    Mesh out;
    QString error;
    QVERIFY2(isotropicRemesh(cube, 0.25, 2, 30.0, out, error),
             qPrintable(error));
    const MeshStats s = computeStats(out);
    QVERIFY(s.valid);
    QVERIFY2(s.closed, "remeshed closed mesh must stay watertight");
    QVERIFY2(s.faces > 50, "target length 0.25 must refine the 12-face cube");
    // Marked sharp edges (the cube's 12 edges) are split/smoothed only
    // along their own line, so the cube's bounds stay put.
    for (int k = 0; k < 3; ++k) {
      QVERIFY(std::abs(s.bboxMin[k] - 0.0) < 1e-6);
      QVERIFY(std::abs(s.bboxMax[k] - 1.0) < 1e-6);
    }
    QVERIFY(std::abs(std::abs(s.volume) - 1.0) < 0.02);
  }

  void isotropicRemeshRejectsInvalidParameters() {
    const Mesh cube = testutil::makeClosedCube();
    Mesh out;
    QString error;
    QVERIFY(!isotropicRemesh(cube, 0.0, 2, 30.0, out, error));
    QVERIFY(!error.isEmpty());
    QVERIFY(!isotropicRemesh(cube, 0.25, 0, 30.0, out, error));
    QVERIFY(!isotropicRemesh(cube, 0.25, 2, 181.0, out, error));
    Mesh empty;
    QVERIFY(!isotropicRemesh(empty, 0.25, 2, 30.0, out, error));
  }

  void extractFeatureEdgesFindsCubeEdges() {
    const Mesh cube = testutil::makeClosedCube(); // [0,1]^3
    FeatureEdges edges;
    QString error;
    QVERIFY2(extractFeatureEdges(cube, 30.0, true, edges, error),
             qPrintable(error));
    // A closed cube has exactly its 12 sharp edges (face diagonals are
    // coplanar and excluded).
    QCOMPARE(edges.edges.size(), static_cast<std::size_t>(12));
    QCOMPARE(edges.points.size(), static_cast<std::size_t>(8));
    for (const auto& p : edges.points) {
      for (double c : p)
        QVERIFY(c >= -1e-9 && c <= 1.0 + 1e-9);
    }
  }

  void extractFeatureEdgesBoundaryOption() {
    const Mesh open = testutil::makeOpenCube(); // one 4-edge top hole
    FeatureEdges withBoundary;
    FeatureEdges withoutBoundary;
    QString error;
    // At 180 degrees no dihedral angle qualifies; only the open boundary
    // can produce edges.
    QVERIFY2(extractFeatureEdges(open, 180.0, true, withBoundary, error),
             qPrintable(error));
    QCOMPARE(withBoundary.edges.size(), static_cast<std::size_t>(4));
    QVERIFY2(extractFeatureEdges(open, 180.0, false, withoutBoundary, error),
             qPrintable(error));
    QVERIFY(withoutBoundary.empty());
  }

  void featureEdgeMeshSavesAndLoads() {
    const Mesh cube = testutil::makeClosedCube();
    FeatureEdges original;
    QString error;
    QVERIFY2(extractFeatureEdges(cube, 30.0, true, original, error),
             qPrintable(error));
    QTemporaryDir tmp;
    QVERIFY(tmp.isValid());
    const QString path = tmp.filePath(QStringLiteral("cube.eMesh"));
    QVERIFY2(writeFeatureEdgeMesh(path, original, error), qPrintable(error));

    QFile file(path);
    QVERIFY(file.open(QIODevice::ReadOnly | QIODevice::Text));
    const QString text = QString::fromUtf8(file.readAll());
    file.close();
    QVERIFY(text.contains(QStringLiteral("class       featureEdgeMesh;")));
    QVERIFY(text.contains(QStringLiteral("// points:")));
    QVERIFY(text.contains(QStringLiteral("// edges:")));

    FeatureEdges loaded;
    QVERIFY2(loadFeatureEdges(path, loaded, error), qPrintable(error));
    QCOMPARE(loaded.points.size(), original.points.size());
    QCOMPARE(loaded.edges.size(), original.edges.size());
    for (std::size_t i = 0; i < loaded.points.size(); ++i) {
      for (int k = 0; k < 3; ++k)
        QVERIFY(std::abs(loaded.points[i][k] - original.points[i][k]) < 1e-12);
    }
    for (std::size_t i = 0; i < loaded.edges.size(); ++i) {
      QCOMPARE(loaded.edges[i][0], original.edges[i][0]);
      QCOMPARE(loaded.edges[i][1], original.edges[i][1]);
    }
  }

  void snapToFeatureEdgesRestoresCubeFeatures() {
    const Mesh cube = testutil::makeClosedCube();
    FeatureEdges edges;
    QString error;
    QVERIFY2(extractFeatureEdges(cube, 30.0, true, edges, error),
             qPrintable(error));
    Mesh out;
    QVERIFY2(snapToFeatureEdges(cube, edges, 0.25, 20.0, out, error),
             qPrintable(error));
    const MeshStats s = computeStats(out);
    QVERIFY(s.valid);
    QVERIFY2(s.closed, "snapped mesh must stay watertight");
    QVERIFY2(s.faces > 12, "feature snapping must refine the cube");
    for (int k = 0; k < 3; ++k) {
      QVERIFY(std::abs(s.bboxMin[k] - 0.0) < 1e-6);
      QVERIFY(std::abs(s.bboxMax[k] - 1.0) < 1e-6);
    }
    QVERIFY(std::abs(std::abs(s.volume) - 1.0) < 0.02);
    // The facet-angle criterion should keep every triangle well away from
    // zero area (a sliver would have an area near machine epsilon).
    double minArea = std::numeric_limits<double>::max();
    for (auto f : out.faces())
      minArea = std::min(minArea, toDouble(PMP::face_area(f, out)));
    QVERIFY2(minArea > 1e-8, "snapping must not create sliver triangles");
  }

  void snapToFeatureEdgesWorksAfterAlphaWrap() {
    const Mesh original = testutil::makeClosedCube();
    const Mesh open = testutil::makeOpenCube();
    FeatureEdges edges;
    QString error;
    QVERIFY2(extractFeatureEdges(original, 30.0, true, edges, error),
             qPrintable(error));
    Mesh wrapped;
    QVERIFY2(alphaWrap(open, 0.6, 0.15, wrapped, error), qPrintable(error));
    Mesh out;
    QVERIFY2(snapToFeatureEdges(wrapped, edges, 0.25, 20.0, out, error),
             qPrintable(error));
    const MeshStats s = computeStats(out);
    QVERIFY(s.valid);
    QVERIFY2(s.closed, "feature-snapped alpha wrap must stay watertight");
    QVERIFY2(s.faces > 12, "snapping must produce a usable surface mesh");
    QVERIFY(std::abs(s.volume) > 0.5);
    double minArea = std::numeric_limits<double>::max();
    for (auto f : out.faces())
      minArea = std::min(minArea, toDouble(PMP::face_area(f, out)));
    QVERIFY2(minArea > 1e-8, "snapping must not create sliver triangles");
  }

  void snapToFeatureEdgesRejectsInvalidInput() {
    const Mesh cube = testutil::makeClosedCube();
    FeatureEdges edges;
    QString error;
    QVERIFY2(extractFeatureEdges(cube, 30.0, true, edges, error),
             qPrintable(error));
    Mesh out;
    QVERIFY(!snapToFeatureEdges(cube, FeatureEdges{}, 0.25, 20.0, out, error));
    QVERIFY(!snapToFeatureEdges(cube, edges, 0.0, 20.0, out, error));
    QVERIFY(!snapToFeatureEdges(cube, edges, 0.25, 31.0, out, error));
    Mesh empty;
    QVERIFY(!snapToFeatureEdges(empty, edges, 0.25, 20.0, out, error));
  }

  void autoCleanRemovesSmallBumpAndPatches() {
    Mesh m = makeBumpyCube();
    AutoCleanParams params;
    params.detailSizeRatio = 0.6;
    params.maxRegionFraction = 0.9;
    params.minSmallFaces = 3;
    params.growRings = 0;
    params.maxHoleEdges = 64;
    AutoCleanResult result;
    QString error;
    QVERIFY2(autoCleanMesh(m, params, result, error), qPrintable(error));
    QCOMPARE(result.clustersRemoved, 1);
    QCOMPARE(result.facesRemoved, 3);
    QCOMPARE(result.holesPatched, 1);
    QVERIFY2(result.patchFacesAdded > 0,
             "the removed bump must leave a filled hole");
    const MeshStats s = computeStats(m);
    QVERIFY(s.valid);
    QVERIFY2(s.closed, "autoclean must close the hole left by the removed "
                       "detail cluster");
    QVERIFY(s.faces >= 12);
  }

  void autoCleanLeavesUniformMeshAlone() {
    Mesh cube = testutil::makeClosedCube();
    AutoCleanParams params; // default parameters
    AutoCleanResult result;
    QString error;
    QVERIFY2(autoCleanMesh(cube, params, result, error), qPrintable(error));
    QCOMPARE(result.clustersRemoved, 0);
    QCOMPARE(result.facesRemoved, 0);
    QCOMPARE(computeStats(cube).faces, static_cast<std::size_t>(12));
  }

  // Remeshing must actually remesh: a 12-face unit cube at target size
  // 0.1 comes back with hundreds of faces, not the input mesh.
  void gmshRemeshRespectsTargetSize() {
    const Mesh cube = testutil::makeClosedCube();
    QTemporaryDir tmp;
    QVERIFY(tmp.isValid());
    QString error;
    const QString in = tmp.filePath(QStringLiteral("in.stl"));
    const QString out = tmp.filePath(QStringLiteral("out.stl"));
    QVERIFY2(saveMesh(in, cube, error), qPrintable(error));
    QVERIFY2(remeshStlWithGmsh(in, out, 0.1, error), qPrintable(error));
    Mesh re;
    QVERIFY2(loadMesh(out, re, error), qPrintable(error));
    const MeshStats s = computeStats(re);
    QVERIFY(s.valid);
    QVERIFY2(s.faces > 100, "remesh at size 0.1 must refine the 12-face cube");
    QVERIFY2(s.closed, "remesh must stay watertight");
  }

  // A torus (handle topology) remeshes too: the classify fallback chain
  // covers shapes one fixed angle cannot parametrize.
  void gmshRemeshHandlesTorus() {
    QTemporaryDir tmp;
    QVERIFY(tmp.isValid());
    QString error;
    PrimitiveParams pp;
    pp.shape = PrimitiveShape::Torus;
    pp.dims = {1.0, 0.3};
    pp.meshSize = 0.2;
    const QString prim = tmp.filePath(QStringLiteral("torus.stl"));
    QVERIFY2(createPrimitiveStl(pp, prim, error), qPrintable(error));
    Mesh input;
    QVERIFY2(loadMesh(prim, input, error), qPrintable(error));
    const int inFaces = computeStats(input).faces;
    QVERIFY(inFaces > 0);
    const QString out = tmp.filePath(QStringLiteral("torus_re.stl"));
    QVERIFY2(remeshStlWithGmsh(prim, out, 0.1, error), qPrintable(error));
    Mesh re;
    QVERIFY2(loadMesh(out, re, error), qPrintable(error));
    const MeshStats s = computeStats(re);
    QVERIFY(s.valid);
    QVERIFY2(s.closed, "remeshed torus must stay watertight");
    QVERIFY2(s.faces != inFaces, "remesh must regenerate, not passthrough");
  }

  void gmshReportsVersion() {
    const QString v = gmshVersion();
    QVERIFY2(!v.isEmpty(), "linked Gmsh library should report a version");
  }

  void gmshCreatesClosedBox() {
    PrimitiveParams p;
    p.shape = PrimitiveShape::Box;
    p.dims = {2.0, 1.0, 0.5};
    p.meshSize = 0.25;
    const QString path =
        QDir::temp().filePath(QStringLiteral("meshbench_test_box.stl"));
    QString error;
    QVERIFY2(createPrimitiveStl(p, path, error), qPrintable(error));
    Mesh box;
    QVERIFY2(loadMesh(path, box, error), qPrintable(error));
    const MeshStats s = computeStats(box);
    QVERIFY(s.valid);
    QVERIFY2(s.closed, "meshed Gmsh box must be watertight");
    QVERIFY(std::abs(std::abs(s.volume) - 1.0) < 1e-6); // 2*1*0.5
  }

  void extractSubmeshKeepsOnlySelectedFaces() {
    const Mesh cube = testutil::makeClosedCube(); // 12 triangles
    // Faces 0+1 are the bottom (z=0) quad split in two; they share an edge.
    const Mesh sub = extractSubmesh(cube, QSet<int>{0, 1});
    const MeshStats s = computeStats(sub);
    QVERIFY(s.valid);
    QCOMPARE(s.faces, static_cast<std::size_t>(2));
    QCOMPARE(sub.num_vertices(), static_cast<std::size_t>(4));
    QVERIFY(s.bboxMin[2] >= -1e-9 && s.bboxMax[2] <= 1e-9); // still at z=0
    QVERIFY(std::abs(std::abs(s.volume)) < 1e-9); // open sheet, no volume
  }

  void extractSubmeshIgnoresBadIds() {
    const Mesh cube = testutil::makeClosedCube();
    QCOMPARE(computeStats(extractSubmesh(cube, {})).faces,
             static_cast<std::size_t>(0));
    QCOMPARE(computeStats(extractSubmesh(cube, QSet<int>{-1, 999})).faces,
             static_cast<std::size_t>(0));
    // Full selection round-trips every face.
    QSet<int> all;
    for (int i = 0; i < 12; ++i)
      all.insert(i);
    QCOMPARE(computeStats(extractSubmesh(cube, all)).faces,
             computeStats(cube).faces);
  }

  void removeMeshFacesCopiesAndDropsUnusedVertices() {
    const Mesh cube = testutil::makeClosedCube();
    Mesh out;
    int removed = 0;
    QString error;
    QVERIFY2(removeMeshFaces(cube, QSet<int>{0, 1}, out, removed, error),
             qPrintable(error));
    QCOMPARE(removed, 2);
    QCOMPARE(out.num_faces(), static_cast<std::size_t>(10));
    // The input stage is untouched: deletions become a new tree object.
    QCOMPARE(cube.num_faces(), static_cast<std::size_t>(12));

    // Keep one triangle: every other face and all unused vertices go away.
    QSet<int> most;
    for (int i = 1; i < 12; ++i)
      most.insert(i);
    Mesh one;
    int oneRemoved = 0;
    QVERIFY2(removeMeshFaces(cube, most, one, oneRemoved, error),
             qPrintable(error));
    QCOMPARE(oneRemoved, 11);
    QCOMPARE(one.num_faces(), static_cast<std::size_t>(1));
    QCOMPARE(one.num_vertices(), static_cast<std::size_t>(3));

    // Ids that do not name a face are ignored and report no deletion.
    Mesh none;
    int noneRemoved = 0;
    QVERIFY(!removeMeshFaces(cube, QSet<int>{-1, 999}, none, noneRemoved,
                             error));
    QCOMPARE(noneRemoved, 0);
    QVERIFY(!error.isEmpty());
  }

  void removeFeatureEdgesCompactsPoints() {
    const Mesh cube = testutil::makeClosedCube();
    FeatureEdges edges;
    QString error;
    QVERIFY2(extractFeatureEdges(cube, 30.0, true, edges, error),
             qPrintable(error));
    QCOMPARE(edges.edges.size(), static_cast<std::size_t>(12));

    const QSet<int> drop{0, 1, 2, 3};
    FeatureEdges out;
    int removed = 0;
    QVERIFY2(removeFeatureEdges(edges, drop, out, removed, error),
             qPrintable(error));
    QCOMPARE(removed, 4);
    QCOMPARE(out.edges.size(), edges.edges.size() - 4);
    // Surviving edges keep their geometry and all indices stay in range.
    std::size_t k = 0;
    for (std::size_t i = 0; i < edges.edges.size(); ++i) {
      if (drop.contains(static_cast<int>(i)))
        continue;
      const auto& src = edges.edges[i];
      const auto& got = out.edges[k++];
      QVERIFY(got[0] >= 0 && got[0] < static_cast<int>(out.points.size()));
      QVERIFY(got[1] >= 0 && got[1] < static_cast<int>(out.points.size()));
      for (int axis = 0; axis < 3; ++axis) {
        QVERIFY(std::abs(out.points[static_cast<std::size_t>(got[0])][axis] -
                         edges.points[static_cast<std::size_t>(src[0])][axis]) <
                1e-12);
        QVERIFY(std::abs(out.points[static_cast<std::size_t>(got[1])][axis] -
                         edges.points[static_cast<std::size_t>(src[1])][axis]) <
                1e-12);
      }
    }

    // Removing the only edge that used a point drops that point too.
    FeatureEdges chain;
    chain.points = {{{0.0, 0.0, 0.0}}, {{1.0, 0.0, 0.0}}, {{2.0, 0.0, 0.0}}};
    chain.edges = {{{0, 1}}, {{1, 2}}};
    FeatureEdges compacted;
    int chainRemoved = 0;
    QVERIFY2(removeFeatureEdges(chain, QSet<int>{0}, compacted, chainRemoved,
                                error),
             qPrintable(error));
    QCOMPARE(chainRemoved, 1);
    QCOMPARE(compacted.points.size(), static_cast<std::size_t>(2));
    QCOMPARE(compacted.edges.size(), static_cast<std::size_t>(1));
    QCOMPARE(compacted.edges[0][0], 0);
    QCOMPARE(compacted.edges[0][1], 1);
    QCOMPARE(compacted.points[0][0], 1.0);
    QCOMPARE(compacted.points[1][0], 2.0);

    FeatureEdges none;
    int noneRemoved = 0;
    QVERIFY(!removeFeatureEdges(edges, QSet<int>{-1, 999}, none, noneRemoved,
                                error));
    QCOMPARE(noneRemoved, 0);
    QVERIFY(none.empty());
  }

  void translateMeshShiftsBBox() {
    Mesh m = testutil::makeClosedCube(); // [0,1]^3
    translateMesh(m, 2.0, -1.0, 0.5);
    const MeshStats s = computeStats(m);
    QVERIFY(s.valid);
    QVERIFY(std::abs(s.bboxMin[0] - 2.0) < 1e-9);
    QVERIFY(std::abs(s.bboxMax[0] - 3.0) < 1e-9);
    QVERIFY(std::abs(s.bboxMin[1] + 1.0) < 1e-9);
    QVERIFY(std::abs(s.bboxMin[2] - 0.5) < 1e-9);
    QCOMPARE(s.faces, static_cast<std::size_t>(12));
    // Unit volume preserved: pure translation.
    QVERIFY(std::abs(std::abs(s.volume) - 1.0) < 1e-9);
  }

  void transformTranslatesMesh() {
    Mesh m = testutil::makeClosedCube(); // [0,1]^3
    TransformParams p;
    p.translate[0] = 3.0;
    p.translate[1] = -2.0;
    p.translate[2] = 1.5;
    QString error;
    QVERIFY2(transformMesh(m, p, error), qPrintable(error));
    const MeshStats s = computeStats(m);
    QVERIFY(s.valid);
    QVERIFY(std::abs(s.bboxMin[0] - 3.0) < 1e-9);
    QVERIFY(std::abs(s.bboxMax[0] - 4.0) < 1e-9);
    QVERIFY(std::abs(s.bboxMin[1] + 2.0) < 1e-9);
    QVERIFY(std::abs(s.bboxMin[2] - 1.5) < 1e-9);
    QCOMPARE(s.faces, static_cast<std::size_t>(12));
    QVERIFY(std::abs(std::abs(s.volume) - 1.0) < 1e-9);
  }

  // 45 degrees about the bbox center keeps the center fixed but expands the
  // axis-aligned box to 0.5 +/- sqrt(2)/2; volume must stay unchanged.
  void transformRotatesAboutMeshCenter() {
    Mesh m = testutil::makeClosedCube(); // [0,1]^3
    TransformParams p;
    p.rotateDeg[2] = 45.0;
    p.rotateAboutCenter = true;
    QString error;
    QVERIFY2(transformMesh(m, p, error), qPrintable(error));
    const MeshStats s = computeStats(m);
    QVERIFY(s.valid);
    const double ext = std::sqrt(2.0) / 2.0;
    QVERIFY(std::abs(s.bboxMin[0] - (0.5 - ext)) < 1e-9);
    QVERIFY(std::abs(s.bboxMax[0] - (0.5 + ext)) < 1e-9);
    QVERIFY(std::abs(s.bboxMin[1] - (0.5 - ext)) < 1e-9);
    QVERIFY(std::abs(s.bboxMax[1] - (0.5 + ext)) < 1e-9);
    QVERIFY(std::abs(s.bboxMin[2] - 0.0) < 1e-9);
    QVERIFY(std::abs(s.bboxMax[2] - 1.0) < 1e-9);
    QVERIFY(std::abs(std::abs(s.volume) - 1.0) < 1e-9);
  }

  // Rotation is applied first, in the world frame, then translation. A +90
  // degree Z rotation about the origin maps [0,1]^2 to [-1,0]x[0,1].
  void transformRotatesAboutOriginThenTranslates() {
    Mesh m = testutil::makeClosedCube(); // [0,1]^3
    TransformParams p;
    p.rotateDeg[2] = 90.0;
    p.rotateAboutCenter = false;
    p.translate[0] = 2.0;
    QString error;
    QVERIFY2(transformMesh(m, p, error), qPrintable(error));
    const MeshStats s = computeStats(m);
    QVERIFY(s.valid);
    QVERIFY(std::abs(s.bboxMin[0] - 1.0) < 1e-9);
    QVERIFY(std::abs(s.bboxMax[0] - 2.0) < 1e-9);
    QVERIFY(std::abs(s.bboxMin[1] - 0.0) < 1e-9);
    QVERIFY(std::abs(s.bboxMax[1] - 1.0) < 1e-9);
    QVERIFY(std::abs(s.bboxMin[2] - 0.0) < 1e-9);
    QVERIFY(std::abs(s.bboxMax[2] - 1.0) < 1e-9);
    QVERIFY(std::abs(std::abs(s.volume) - 1.0) < 1e-9);
  }

  void transformRejectsInvalidParameters() {
    Mesh m = testutil::makeClosedCube();
    TransformParams p;
    p.translate[0] = std::numeric_limits<double>::infinity();
    QString error;
    QVERIFY(!transformMesh(m, p, error));
    QVERIFY(!error.isEmpty());
    QCOMPARE(computeStats(m).faces, static_cast<std::size_t>(12)); // untouched

    Mesh empty;
    TransformParams ok;
    error.clear();
    QVERIFY(!transformMesh(empty, ok, error));
    QVERIFY(!error.isEmpty());
  }

  void operationSearchMatchesAllTerms() {
    QVERIFY(operationSearchMatches(QStringLiteral("Isotropic remesh..."),
                                   QStringLiteral("iso")));
    QVERIFY(operationSearchMatches(QStringLiteral("Snap to feature edges..."),
                                   QStringLiteral("snap edges")));
    QVERIFY(operationSearchMatches(QStringLiteral("Alpha wrap..."), QString()));
    QVERIFY(!operationSearchMatches(QStringLiteral("Alpha wrap..."),
                                    QStringLiteral("snap")));
    QVERIFY(!operationSearchMatches(QStringLiteral("Alpha wrap..."),
                                    QStringLiteral("alpha missing")));
  }

  void standardViewBasesAreUnitAndOrthogonal() {
    const std::array<StandardView, 6> views = {
        StandardView::PosX, StandardView::NegX, StandardView::PosY,
        StandardView::NegY, StandardView::PosZ, StandardView::NegZ};
    for (StandardView v : views) {
      const ViewBasis b = standardViewBasis(v);
      const double len = std::sqrt(b.cameraOffset[0] * b.cameraOffset[0] +
                                   b.cameraOffset[1] * b.cameraOffset[1] +
                                   b.cameraOffset[2] * b.cameraOffset[2]);
      const double dot = b.cameraOffset[0] * b.up[0] +
                         b.cameraOffset[1] * b.up[1] +
                         b.cameraOffset[2] * b.up[2];
      QVERIFY(std::abs(len - 1.0) < 1e-12);
      QVERIFY(std::abs(dot) < 1e-12);
    }
    // Menu order is +X, -X, +Y, -Y, +Z, -Z; +Z/-Z use Y as up.
    QCOMPARE(standardViewBasis(StandardView::PosX).cameraOffset[0], 1.0);
    QCOMPARE(standardViewBasis(StandardView::NegX).cameraOffset[0], -1.0);
    QCOMPARE(standardViewBasis(StandardView::PosY).cameraOffset[1], 1.0);
    QCOMPARE(standardViewBasis(StandardView::NegY).cameraOffset[1], -1.0);
    QCOMPARE(standardViewBasis(StandardView::PosZ).cameraOffset[2], 1.0);
    QCOMPARE(standardViewBasis(StandardView::NegZ).cameraOffset[2], -1.0);
    QCOMPARE(standardViewBasis(StandardView::PosZ).up[1], 1.0);
    QCOMPARE(standardViewBasis(StandardView::NegZ).up[1], 1.0);
  }

  void mergeMeshesConcatenates() {
    const Mesh a = testutil::makeClosedCube(); // 12 faces, 8 verts
    Mesh b = testutil::makeClosedCube();
    // Shift the copy so the inputs merely touch at x=1.
    for (auto v : b.vertices()) {
      Point3 p = b.point(v);
      b.point(v) = makePoint(toDouble(p.x()) + 1.0, toDouble(p.y()),
                             toDouble(p.z()));
    }
    const Mesh m = mergeMeshes({&a, &b});
    const MeshStats s = computeStats(m);
    QVERIFY(s.valid);
    // Faces add up; touching-vertex inputs stay split (no stitching).
    QCOMPARE(s.faces, static_cast<std::size_t>(24));
    QCOMPARE(m.num_vertices(), static_cast<std::size_t>(16));
    QVERIFY(std::abs((s.bboxMax[0] - s.bboxMin[0]) - 2.0) < 1e-9);
  }

  void mergeMeshesSkipsNullAndEmpty() {
    const Mesh a = testutil::makeClosedCube();
    QCOMPARE(computeStats(mergeMeshes({})).faces, static_cast<std::size_t>(0));
    QCOMPARE(computeStats(mergeMeshes({nullptr, &a})).faces,
             computeStats(a).faces);
  }

  // The Combine dialog's Union path folds N stages with sequential pairwise
  // unions; three overlapping cubes must come out as one watertight solid.
  void booleanUnionChainsThreeObjects() {
    const Mesh a = testutil::makeClosedCube(); // [0,1]^3
    const Mesh b = translatedCube(0.5, 0.0, 0.0); // [0.5,1.5]
    const Mesh c = translatedCube(1.0, 0.0, 0.0); // [1,2]
    QString error;
    Mesh ab, abc;
    QVERIFY2(booleanOp(a, b, BooleanOp::Union, ab, error), qPrintable(error));
    QVERIFY2(booleanOp(ab, c, BooleanOp::Union, abc, error), qPrintable(error));
    const MeshStats s = computeStats(abc);
    QVERIFY(s.valid);
    QVERIFY2(s.closed && s.boundaryEdges == 0,
             "chained union must stay watertight");
    QVERIFY(std::abs(std::abs(s.volume) - 2.0) < 1e-6); // [0,2]x[0,1]x[0,1]
    QVERIFY(std::abs((s.bboxMax[0] - s.bboxMin[0]) - 2.0) < 1e-9);
  }

  // Selection round-trip: split a mesh into complementary subsets (as the
  // viewport selection does) and recombine them with merge; every face
  // must survive and the bounding box must round-trip.
  void extractSubsetsRecombineToWhole() {
    const Mesh cube = testutil::makeClosedCube(); // 12 faces
    QSet<int> lo, hi;
    for (int i = 0; i < 12; ++i)
      (i < 6 ? lo : hi).insert(i);
    const Mesh a = extractSubmesh(cube, lo);
    const Mesh b = extractSubmesh(cube, hi);
    QCOMPARE(computeStats(a).faces, static_cast<std::size_t>(6));
    QCOMPARE(computeStats(b).faces, static_cast<std::size_t>(6));
    const Mesh m = mergeMeshes({&a, &b});
    const MeshStats s = computeStats(m);
    QVERIFY(s.valid);
    QCOMPARE(s.faces, computeStats(cube).faces);
    const MeshStats c = computeStats(cube);
    for (int k = 0; k < 3; ++k) {
      QVERIFY(std::abs(s.bboxMin[k] - c.bboxMin[k]) < 1e-9);
      QVERIFY(std::abs(s.bboxMax[k] - c.bboxMax[k]) < 1e-9);
    }
  }

  void removeRandomTrianglesIsExactAndSeeded() {
    QString error;
    Mesh a = testutil::makeClosedCube(); // 12 faces
    const int removed = removeRandomTriangles(a, 3, 42, error);
    QCOMPARE(removed, 3);
    QVERIFY(error.isEmpty());
    MeshStats sa = computeStats(a);
    QCOMPARE(sa.faces, static_cast<std::size_t>(9));
    QVERIFY(sa.boundaryEdges > 0);

    // Same seed, fresh cube: identical puncture pattern.
    Mesh b = testutil::makeClosedCube();
    QCOMPARE(removeRandomTriangles(b, 3, 42, error), 3);
    const MeshStats sb = computeStats(b);
    QCOMPARE(sb.faces, sa.faces);
    QCOMPARE(sb.boundaryEdges, sa.boundaryEdges);

    // Asking for more than exist removes all of them.
    Mesh c = testutil::makeClosedCube();
    QCOMPARE(removeRandomTriangles(c, 100, 7, error), 12);
    QCOMPARE(computeStats(c).faces, static_cast<std::size_t>(0));
  }
};

QTEST_MAIN(TstMeshOps)
#include "tst_meshops.moc"
