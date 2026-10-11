// Unit tests for the CGAL mesh operations (MeshOps).
//
// One focused test per feature: centering, slicing, booleans, alpha
// wrapping, plus a Gmsh availability check when compiled with Gmsh.

#include "MeshDocument.h"
#include "MeshOps.h"
#include "test_utils.h"

#include <QDir>
#include <QTemporaryDir>
#include <QtTest>
#include <cmath>

using namespace meshbench;

static Mesh translatedCube(double dx, double dy, double dz) {
  Mesh m = testutil::makeClosedCube();
  for (auto v : m.vertices()) {
    Point3 p = m.point(v);
    m.point(v) = makePoint(toDouble(p.x()) + dx, toDouble(p.y()) + dy,
                           toDouble(p.z()) + dz);
  }
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

#ifdef MESHBENCH_HAVE_GMSH
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
#endif

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
