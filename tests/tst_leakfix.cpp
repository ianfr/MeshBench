// Unit tests for leak detection + patching (LeakFix).
//
// 1. Closed cube: watertight, no hole loops, inside/outside probes separated.
// 2. Open cube (top face removed): one hole loop, leak found along a path,
//    fan patch closes it, re-probe shows no leak.

#include "LeakFix.h"
#include "MeshDocument.h"
#include "MeshOps.h"
#include "test_utils.h"

#include <CGAL/Subdivision_method_3/subdivision_methods_3.h>

#include <QtTest>

using namespace meshbench;

class TstLeakFix : public QObject {
  Q_OBJECT
private slots:
  void closedCubeIsSealed() {
    const Mesh cube = testutil::makeClosedCube();
    QVERIFY(isWatertight(cube));
    QVERIFY(findHoleLoops(cube).isEmpty());

    const LeakProbeResult r = detectLeaks(cube, {{0.5, 0.5, 0.5}},
                                          {{5.0, 5.0, 5.0}});
    QVERIFY2(r.warnings.isEmpty(), qPrintable(r.warnings.join("; ")));
    QVERIFY(r.regions >= 2); // interior region distinct from exterior
    QVERIFY(!r.hasLeak);
    QVERIFY(r.paths.isEmpty());
  }

  void openCubeLeaksThenPatches() {
    Mesh open = testutil::makeOpenCube();
    QVERIFY(!isWatertight(open));

    const auto loops = findHoleLoops(open);
    QCOMPARE(loops.size(), 1);
    QCOMPARE(loops.front().edges(), 4);

    LeakProbeResult before =
        detectLeaks(open, {{0.5, 0.5, 0.5}}, {{5.0, 5.0, 5.0}});
    QVERIFY2(before.warnings.isEmpty(), qPrintable(before.warnings.join("; ")));
    QVERIFY(before.hasLeak);
    QCOMPARE(before.paths.size(), 1);
    QVERIFY(before.paths.front().polyline.size() >= 2);

    const PatchResult pr = patchHoles(open, 64);
    QCOMPARE(pr.loopsFound, 1);
    QCOMPARE(pr.loopsFilled, 1);
    QCOMPARE(pr.loopsSkippedTooLarge, 0);
    QVERIFY(isWatertight(open));
    QVERIFY(findHoleLoops(open).isEmpty());

    const LeakProbeResult after =
        detectLeaks(open, {{0.5, 0.5, 0.5}}, {{5.0, 5.0, 5.0}});
    QVERIFY(!after.hasLeak);
  }

  // Probe overlays belong to LeakProbe stages only.
  void probeOverlaysShowOnlyOnLeakProbe() {
    QVERIFY(showsProbeOverlays(QStringLiteral("LeakProbe")));
    QVERIFY(showsProbeOverlays(QStringLiteral("LeakProbe (2 loops, 1 leak(s))")));
    QVERIFY(!showsProbeOverlays(QStringLiteral("Puncture (removed 5 tris)")));
    QVERIFY(!showsProbeOverlays(QStringLiteral("PatchHoles (1 loops filled)")));
    QVERIFY(!showsProbeOverlays(QStringLiteral("Source")));
  }

  // Full user flow: puncture a sealed mesh, detect the leak, patch it,
  // re-probe clean (mirrors Puncture -> LeakProbe -> PatchHoles stages).
  void punctureDetectPatchRoundTrip() {
    Mesh mesh = testutil::makeClosedCube();
    QString error;
    QCOMPARE(removeRandomTriangles(mesh, 5, 1234, error), 5);
    QVERIFY(!isWatertight(mesh));
    QVERIFY(!findHoleLoops(mesh).isEmpty());

    const LeakProbeResult leaked =
        detectLeaks(mesh, {{0.5, 0.5, 0.5}}, {{5.0, 5.0, 5.0}});
    QVERIFY2(leaked.warnings.isEmpty(),
             qPrintable(leaked.warnings.join("; ")));
    QVERIFY(leaked.hasLeak);

    const PatchResult pr = patchHoles(mesh, 64);
    QVERIFY(pr.loopsFilled > 0);
    QCOMPARE(pr.loopsSkippedTooLarge, 0);
    QVERIFY(isWatertight(mesh));

    const LeakProbeResult sealed =
        detectLeaks(mesh, {{0.5, 0.5, 0.5}}, {{5.0, 5.0, 5.0}});
    QVERIFY(!sealed.hasLeak);
  }

  // Bridge analysis implicates the hole on a leak route.
  void bridgingImplicatesHoleOnRoute() {
    const Mesh open = testutil::makeOpenCube(); // single 4-edge top hole
    const LeakProbeResult r =
        detectLeaks(open, {{0.5, 0.5, 0.5}}, {{5.0, 5.0, 5.0}});
    QVERIFY(r.hasLeak);
    QCOMPARE(r.loops.size(), 1);
    QCOMPARE(r.bridgingLoops.size(), 1);
    QVERIFY(r.bridgingLoops.contains(0));
  }

  // Bridge analysis is complete: every hole of a multi-hole leak is
  // implicated, so selective patching seals the mesh fully. Uses a
  // subdivided mesh so the remnant keeps real volume: removing 10 of the
  // 12 faces of a plain cube leaves two disjoint triangles whose fills
  // are zero-volume pillows (closed, but with no interior to separate).
  void bridgingCoversEveryHole() {
    Mesh mesh = testutil::makeClosedCube();
    CGAL::Subdivision_method_3::Loop_subdivision(mesh, 2); // 192 faces
    QVERIFY(isWatertight(mesh));
    QString error;
    QCOMPARE(removeRandomTriangles(mesh, 10, 77, error), 10);
    LeakProbeResult r =
        detectLeaks(mesh, {{0.5, 0.5, 0.5}}, {{5.0, 5.0, 5.0}});
    QVERIFY(r.hasLeak);
    QVERIFY(r.loops.size() >= 2);
    QCOMPARE(r.bridgingLoops.size(), r.loops.size());
    const PatchResult pr = patchHoles(mesh, 64, r.bridgingLoops);
    QCOMPARE(pr.loopsSkippedNotLeaking, 0);
    QVERIFY(isWatertight(mesh));
    const LeakProbeResult sealed =
        detectLeaks(mesh, {{0.5, 0.5, 0.5}}, {{5.0, 5.0, 5.0}});
    QVERIFY(!sealed.hasLeak);
  }

  // A sealed mesh implicates nothing (blind holes excluded).
  void sealedMeshImplicatesNothing() {
    const Mesh cube = testutil::makeClosedCube();
    const LeakProbeResult r =
        detectLeaks(cube, {{0.5, 0.5, 0.5}}, {{5.0, 5.0, 5.0}});
    QVERIFY(!r.hasLeak);
    QVERIFY(r.bridgingLoops.isEmpty());
  }

  // Regression: a few small holes in a fine mesh must still leak.
  // (Cell-blocking the barrier paved such holes over with rim cells and
  // reported "separated"; face-blocking keeps them open.)
  void smallHolesOnFineMeshStillLeak() {
    Mesh mesh = testutil::makeClosedCube();
    CGAL::Subdivision_method_3::Loop_subdivision(mesh, 3); // 768 faces
    QVERIFY(isWatertight(mesh));
    QString error;
    QCOMPARE(removeRandomTriangles(mesh, 4, 99, error), 4);
    const auto loops = findHoleLoops(mesh);
    QVERIFY(!loops.isEmpty());
    const LeakProbeResult r =
        detectLeaks(mesh, {{0.5, 0.5, 0.5}}, {{5.0, 5.0, 5.0}});
    QVERIFY2(r.warnings.isEmpty(), qPrintable(r.warnings.join("; ")));
    QVERIFY2(r.hasLeak, "fine mesh with holes must report a leak");
    QVERIFY(!r.paths.isEmpty() || !r.walkPaths.isEmpty());
  }

  // Barrier predicate: triangle vs axis-aligned voxel-face quad.
  void triOverlapsFaceQuadPiercing() {
    const double t = 1e-9;
    // Triangle edge pierces the quad interior (axis 0, plane x = 0).
    const double a[3] = {-1.0, 0.4, 0.4};
    const double b[3] = {1.0, 0.4, 0.4};
    const double c[3] = {-1.0, 0.6, 0.6};
    QVERIFY(detail::triOverlapsFaceQuad(a, b, c, 0, 0.0, 0.0, 1.0, 0.0, 1.0, t));
    // Same triangle, quad far away: no overlap.
    QVERIFY(!detail::triOverlapsFaceQuad(a, b, c, 0, 5.0, 0.0, 1.0, 0.0, 1.0, t));
    // Coplanar overlap: triangle in plane z = 0 over the quad.
    const double p[3] = {0.2, 0.2, 0.0};
    const double q[3] = {0.8, 0.2, 0.0};
    const double r[3] = {0.2, 0.8, 0.0};
    QVERIFY(detail::triOverlapsFaceQuad(p, q, r, 2, 0.0, 0.0, 1.0, 0.0, 1.0, t));
    // Coplanar but disjoint in 2D: no overlap.
    QVERIFY(!detail::triOverlapsFaceQuad(p, q, r, 2, 0.0, 2.0, 3.0, 2.0, 3.0, t));
    // Triangle strictly on one side, edges not touching the plane: open.
    const double u[3] = {0.2, 0.2, 0.5};
    const double v[3] = {0.8, 0.2, 0.5};
    const double w[3] = {0.2, 0.8, 0.5};
    QVERIFY(!detail::triOverlapsFaceQuad(u, v, w, 2, 0.0, 0.0, 1.0, 0.0, 1.0, t));
  }

  // A closed mesh has no border to leak through, even where the voxel
  // barrier threads a sub-cell gap (Loop-subdivided cube reports a flood
  // path through its rounded caps at res 48).
  void closedSubdividedCubeIsSealed() {
    Mesh mesh = testutil::makeClosedCube();
    CGAL::Subdivision_method_3::Loop_subdivision(mesh, 2);
    QVERIFY(isWatertight(mesh));
    QVERIFY(findHoleLoops(mesh).isEmpty());
    const LeakProbeResult r =
        detectLeaks(mesh, {{0.5, 0.5, 0.5}}, {{5.0, 5.0, 5.0}});
    QVERIFY(!r.hasLeak);
    QVERIFY(r.paths.isEmpty());
    QVERIFY(r.walkPaths.isEmpty());
    QVERIFY(r.bridgingLoops.isEmpty());
  }

  // The probe reports its background grid geometry (for the voxel-grid
  // overlay) plus every route found, BFS and random-walk alike.
  void probeReportsGridAndAllRoutes() {
    Mesh open = testutil::makeOpenCube();
    const LeakProbeResult r =
        detectLeaks(open, {{0.5, 0.5, 0.5}}, {{5.0, 5.0, 5.0}});
    QVERIFY(r.hasLeak);
    QCOMPARE(r.gridRes, 48);
    QVERIFY(r.cellSize > 0.0);
    QVERIFY(r.gridOrigin[0] <= 0.0);
    QVERIFY(r.gridOrigin[1] <= 0.0);
    QVERIFY(r.gridOrigin[2] <= 0.0);
    QVERIFY(r.gridOrigin[0] + r.gridRes * r.cellSize >= 1.0);
    QVERIFY(r.gridOrigin[1] + r.gridRes * r.cellSize >= 1.0);
    QVERIFY(r.gridOrigin[2] + r.gridRes * r.cellSize >= 1.0);
    QVERIFY(!r.paths.isEmpty());
    QVERIFY(!r.walkPaths.isEmpty());
  }

  // Stored-route/grid predicates gate the view toggles: empty reports
  // draw nothing, while populated ones (LeakProbe or PatchHoles stages
  // carrying the routes/grid patching was based on) draw.
  void storedRoutesGateOverlays() {
    LeakReport empty;
    QVERIFY(!empty.hasStoredRoutes());
    QVERIFY(!empty.hasStoredGrid());
    LeakReport routes;
    routes.leakPaths.append(
        QVector<std::array<double, 3>>{{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}});
    QVERIFY(routes.hasStoredRoutes());
    QVERIFY(!routes.hasStoredGrid());
    routes.gridRes = 48;
    routes.gridOrigin = {0.0, 0.0, 0.0};
    routes.cellSize = 0.025;
    QVERIFY(routes.hasStoredGrid());
  }

  // Walk routes span the probe points: each starts exactly at its
  // inside point and ends exactly at an outside point (straight
  // connector from the grid border when the outside point is off-grid).
  void walkRoutesSpanProbePoints() {
    Mesh open = testutil::makeOpenCube();
    const std::array<double, 3> inPt{0.5, 0.5, 0.5};
    const std::array<double, 3> outPt{5.0, 5.0, 5.0};
    const LeakProbeResult r = detectLeaks(open, {inPt}, {outPt});
    QVERIFY(r.hasLeak);
    QVERIFY(!r.walkPaths.isEmpty());
    for (const auto& w : r.walkPaths) {
      QVERIFY(w.polyline.size() >= 2);
      QVERIFY(w.polyline.front() == inPt);
      QVERIFY(w.polyline.back() == outPt);
    }
  }

  // Random walks are seeded: identical params give identical routes.
  void randomWalksAreDeterministic() {
    const Mesh open = testutil::makeOpenCube();
    ProbeParams p;
    p.walksPerInside = 8;
    const LeakProbeResult a =
        detectLeaks(open, {{0.5, 0.5, 0.5}}, {{5.0, 5.0, 5.0}}, p);
    const LeakProbeResult b =
        detectLeaks(open, {{0.5, 0.5, 0.5}}, {{5.0, 5.0, 5.0}}, p);
    QCOMPARE(a.walkPaths.size(), b.walkPaths.size());
    QVERIFY(!a.walkPaths.isEmpty());
    for (int i = 0; i < a.walkPaths.size(); ++i) {
      QCOMPARE(a.walkPaths[i].polyline, b.walkPaths[i].polyline);
    }
  }
};

QTEST_MAIN(TstLeakFix)
#include "tst_leakfix.moc"
