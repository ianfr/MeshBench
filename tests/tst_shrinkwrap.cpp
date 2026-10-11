// Unit tests for the interactive shrinkwrap (Shrinkwrap).
//
// 1. The enclosing sphere surrounds the target with the expected resolution.
// 2. Snapping converges wrap vertices onto the target surface.
// 3. Zero weights leave the wrap untouched (weights drive the heuristic).

#include "MeshOps.h"
#include "Shrinkwrap.h"
#include "test_utils.h"

#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_tree.h>

#include <QtTest>
#include <cmath>

using namespace meshbench;

class TstShrinkwrap : public QObject {
  Q_OBJECT
private slots:
  void enclosingSphereSurroundsTarget() {
    const Mesh cube = testutil::makeClosedCube();
    const Mesh sphere = makeEnclosingSphere(cube, 2);
    // Icosphere: V = 10*4^s + 2.
    QCOMPARE(sphere.num_vertices(), static_cast<std::size_t>(162));
    const MeshStats s = computeStats(sphere);
    QVERIFY(s.valid);
    // Unit cube bbox [0,1]^3 must sit strictly inside the sphere bbox.
    QVERIFY(s.bboxMin[0] < 0.0 && s.bboxMax[0] > 1.0);
    QVERIFY(s.bboxMin[1] < 0.0 && s.bboxMax[1] > 1.0);
    QVERIFY(s.bboxMin[2] < 0.0 && s.bboxMax[2] > 1.0);
  }

  void snapConvergesToSurface() {
    const Mesh cube = testutil::makeClosedCube();
    Mesh wrap = makeEnclosingSphere(cube, 2);
    ShrinkParams p;
    p.iterations = 30;
    p.step = 0.5;
    p.smooth = 0.0;
    QString error;
    QVERIFY2(snapShrinkwrap(cube, {}, wrap, p, error), qPrintable(error));

    using Primitive = CGAL::AABB_face_graph_triangle_primitive<Mesh>;
    using Traits = CGAL::AABB_traits<Kernel, Primitive>;
    CGAL::AABB_tree<Traits> tree(cube.faces().begin(), cube.faces().end(), cube);
    double maxDist = 0.0;
    for (auto v : wrap.vertices()) {
      const double d =
          std::sqrt(toDouble(tree.squared_distance(wrap.point(v))));
      maxDist = std::max(maxDist, d);
    }
    QVERIFY2(maxDist < 1e-4, qPrintable(QString::number(maxDist)));
  }

  void uniformityEvensSpacing() {
    const Mesh cube = testutil::makeClosedCube();
    auto edgeCv = [](const Mesh& m) {
      double sum = 0.0, sum2 = 0.0, n = 0.0;
      for (auto e : m.edges()) {
        const auto h = m.halfedge(e);
        const Point3& a = m.point(m.source(h));
        const Point3& b = m.point(m.target(h));
        const double dx = toDouble(a.x()) - toDouble(b.x());
        const double dy = toDouble(a.y()) - toDouble(b.y());
        const double dz = toDouble(a.z()) - toDouble(b.z());
        const double l = std::sqrt(dx * dx + dy * dy + dz * dz);
        sum += l;
        sum2 += l * l;
        n += 1.0;
      }
      const double mean = sum / n;
      const double var = sum2 / n - mean * mean;
      return std::sqrt(std::max(0.0, var)) / mean; // coefficient of variation
    };
    ShrinkParams plain;
    plain.iterations = 20;
    plain.step = 0.5;
    plain.smooth = 0.0;
    plain.uniformity = 0.0;
    Mesh wplain = makeEnclosingSphere(cube, 2);
    QString error;
    QVERIFY2(snapShrinkwrap(cube, {}, wplain, plain, error), qPrintable(error));

    ShrinkParams even = plain;
    even.uniformity = 0.8;
    Mesh weven = makeEnclosingSphere(cube, 2);
    QVERIFY2(snapShrinkwrap(cube, {}, weven, even, error), qPrintable(error));

    // Even spacing must not scatter vertices off the surface...
    using Primitive = CGAL::AABB_face_graph_triangle_primitive<Mesh>;
    using Traits = CGAL::AABB_traits<Kernel, Primitive>;
    CGAL::AABB_tree<Traits> tree(cube.faces().begin(), cube.faces().end(), cube);
    double maxDist = 0.0;
    for (auto v : weven.vertices())
      maxDist = std::max(maxDist, std::sqrt(toDouble(
                                            tree.squared_distance(weven.point(v)))));
    QVERIFY2(maxDist < 1e-2, qPrintable(QString::number(maxDist)));
    // ...and should leave edge lengths no less uniform than plain snapping.
    QVERIFY2(edgeCv(weven) <= edgeCv(wplain),
             qPrintable(QStringLiteral("%1 vs %2")
                            .arg(edgeCv(weven))
                            .arg(edgeCv(wplain))));
  }

  void zeroWeightsFreezeWrap() {
    const Mesh cube = testutil::makeClosedCube();
    Mesh wrap = makeEnclosingSphere(cube, 2);
    const MeshStats before = computeStats(wrap);
    // Zero weight on every target face: no attraction anywhere.
    std::unordered_map<std::size_t, double> weights;
    for (auto f : cube.faces())
      weights[f.idx()] = 0.0;
    ShrinkParams p;
    p.iterations = 10;
    p.step = 0.5;
    p.smooth = 0.0; // isolate attraction from smoothing
    QString error;
    QVERIFY2(snapShrinkwrap(cube, weights, wrap, p, error), qPrintable(error));
    const MeshStats after = computeStats(wrap);
    QVERIFY(std::abs(after.volume - before.volume) < 1e-9);
  }
};

QTEST_MAIN(TstShrinkwrap)
#include "tst_shrinkwrap.moc"
