// Unit tests for the pipeline history (MeshDocument).
//
// Successive "Create primitive shape" operations must create parallel
// pipeline objects: adding a new independent mesh must keep every
// existing node instead of overwriting it.

#include "MeshDocument.h"
#include "MeshOps.h"
#include "StageSelection.h"
#include "test_utils.h"

#include <QtTest>

using namespace meshbench;

class TstMeshDocument : public QObject {
  Q_OBJECT
private slots:
  void successiveIndependentMeshesStayParallel() {
    MeshDocument doc;
    Mesh box = testutil::makeClosedCube();
    doc.setSource(std::move(box), QStringLiteral("Box(1,1,1)"));
    QCOMPARE(doc.nodeCount(), 1);

    // A second primitive must extend the pipeline, not replace node 0.
    Mesh sphere = testutil::makeClosedCube();
    const int idx =
        doc.appendSource(QStringLiteral("Sphere(1)"), std::move(sphere));
    QCOMPARE(idx, 1);
    QCOMPARE(doc.nodeCount(), 2);
    QCOMPARE(doc.activeIndex(), 1);
    QCOMPARE(doc.node(0).label, QStringLiteral("Box(1,1,1)"));
    QCOMPARE(doc.node(1).label, QStringLiteral("Sphere(1)"));
    QCOMPARE(computeStats(doc.node(0).mesh).faces,
             static_cast<std::size_t>(12));
    QCOMPARE(computeStats(doc.node(1).mesh).faces,
             static_cast<std::size_t>(12));
  }

  void independentMeshDoesNotCarryProbesForward() {
    MeshDocument doc;
    Mesh box = testutil::makeClosedCube();
    doc.setSource(std::move(box), QStringLiteral("Box(1,1,1)"));
    doc.setProbes({{0.5, 0.5, 0.5, true}});
    QCOMPARE(doc.node(0).probes.size(), 1);

    Mesh sphere = testutil::makeClosedCube();
    doc.appendSource(QStringLiteral("Sphere(1)"), std::move(sphere));
    QVERIFY(doc.node(1).probes.isEmpty());
    // The earlier stage keeps its own probes.
    QCOMPARE(doc.node(0).probes.size(), 1);
  }

  void independentMeshKeepsRedoTail() {
    MeshDocument doc;
    Mesh a = testutil::makeClosedCube();
    doc.setSource(std::move(a), QStringLiteral("Box(1,1,1)"));
    Mesh b = testutil::makeClosedCube();
    doc.appendOperation(QStringLiteral("Center"), QStringLiteral("at volume center"),
                        std::move(b));
    QCOMPARE(doc.nodeCount(), 2);
    // Go back to the older stage: a new independent mesh must still
    // land on the tip instead of dropping the redo tail.
    doc.setActive(0);
    Mesh c = testutil::makeClosedCube();
    doc.appendSource(QStringLiteral("Sphere(1)"), std::move(c));
    QCOMPARE(doc.nodeCount(), 3);
    QCOMPARE(doc.node(1).label, QStringLiteral("Center"));
    QCOMPARE(doc.activeIndex(), 2);
  }

  // The right-click Combine selection is built one stage at a time and
  // must survive Qt clearing the tree selection when the menu opens.
  void rightClickStageSelectionAccumulates() {
    QSet<int> selection;
    selection = toggleStageSelection(selection, 0, true);
    selection = toggleStageSelection(selection, 2, true);
    QCOMPARE(selection.size(), 2);
    QVERIFY(selection.contains(0));
    QVERIFY(selection.contains(2));

    // Re-clicking one stage removes only that stage.
    selection = toggleStageSelection(selection, 0, false);
    QCOMPARE(selection.size(), 1);
    QVERIFY(selection.contains(2));

    // Merge in the tree's current selection (e.g. from Shift-click).
    selection = mergeStageSelections(selection, QSet<int>{1, 2});
    QCOMPARE(selection.size(), 2);
    QVERIFY(selection.contains(1));
    QVERIFY(selection.contains(2));

    // Truncating/reverting the pipeline drops out-of-range stages.
    selection = validStageSelection(QSet<int>{-1, 0, 1, 9}, 2);
    QCOMPARE(selection.size(), 2);
    QVERIFY(selection.contains(0));
    QVERIFY(selection.contains(1));

    // Explicit right-click/checkbox picks are authoritative and are not
    // contaminated by Qt's transient tree selection when the menu opens.
    const QSet<int> explicitPicks = {0, 2};
    const QSet<int> treePicks = {1};
    QVERIFY(resolveStageSelection(explicitPicks, treePicks) == explicitPicks);
    QVERIFY(resolveStageSelection({}, treePicks) == treePicks);
  }
};

QTEST_MAIN(TstMeshDocument)
#include "tst_meshdocument.moc"
