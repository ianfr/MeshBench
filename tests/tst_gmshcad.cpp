// Unit tests for the MCP-only low-level Gmsh CAD session.
//
// These tests exercise the persistent OCC session directly (no GUI): create
// primitives, boolean-cut a hole, extrude a surface, fillet an edge, and
// convert the result to a surface mesh.

#include "GmshCadSession.h"
#include "MeshOps.h"

#include <QtTest>

#include <QTemporaryDir>

#include <vector>

using namespace meshbench;

class TstGmshCad : public QObject {
  Q_OBJECT
private slots:
  void inactiveSessionRejectsOperations() {
    GmshCadSession session;
    int tag = -1;
    QString error;
    QVERIFY(!session.active());
    QVERIFY(!session.addBox({0, 0, 0}, {1, 1, 1}, -1, tag, error));
    QVERIFY(!error.isEmpty());
  }

  void boxMeshesToStl() {
    GmshCadSession session;
    QString error;
    QVERIFY2(session.begin(QString(), 0.5, error), qPrintable(error));
    QVERIFY(session.active());
    int tag = -1;
    QVERIFY2(session.addBox({0, 0, 0}, {2, 2, 2}, -1, tag, error),
             qPrintable(error));
    QVERIFY(tag > 0);
    QCOMPARE(session.meshSize(), 0.5);

    int count = 0;
    QVERIFY2(session.entityCount(3, count, error), qPrintable(error));
    QCOMPARE(count, 1);

    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString path = dir.filePath(QStringLiteral("box.stl"));
    QVERIFY2(session.meshToStl(path, error), qPrintable(error));
    Mesh mesh;
    QVERIFY2(loadMesh(path, mesh, error), qPrintable(error));
    QVERIFY(mesh.num_vertices() > 0);
    QVERIFY(mesh.num_faces() > 0);

    QVERIFY2(session.end(error), qPrintable(error));
    QVERIFY(!session.active());
  }

  void booleanCutCreatesHole() {
    GmshCadSession session;
    QString error;
    QVERIFY2(session.begin(QString(), 0.4, error), qPrintable(error));
    int box = -1;
    int cylinder = -1;
    QVERIFY2(session.addBox({-1, -1, -1}, {2, 2, 2}, -1, box, error),
             qPrintable(error));
    QVERIFY2(session.addCylinder({0, 0, -2}, {0, 0, 1}, 4, 0.5, 0.0, -1,
                                 cylinder, error),
             qPrintable(error));
    std::vector<CadEntity> out;
    QVERIFY2(session.booleanOp(QStringLiteral("cut"), {box}, {cylinder}, true,
                               true, out, error),
             qPrintable(error));
    QVERIFY(!out.empty());

    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString path = dir.filePath(QStringLiteral("cut.stl"));
    QVERIFY2(session.meshToStl(path, error), qPrintable(error));
    Mesh mesh;
    QVERIFY2(loadMesh(path, mesh, error), qPrintable(error));
    QVERIFY(mesh.num_faces() > 12);
    const MeshStats stats = computeStats(mesh);
    QVERIFY(stats.closed);
    QVERIFY2(stats.volume > 0.0,
             qPrintable(QStringLiteral("volume: %1").arg(stats.volume)));
    QVERIFY2(session.end(error), qPrintable(error));
  }

  void extrudedRectangleBecomesSolid() {
    GmshCadSession session;
    QString error;
    QVERIFY2(session.begin(QString(), 0.5, error), qPrintable(error));
    int rectangle = -1;
    QVERIFY2(session.addRectangle({0, 0, 0}, {2.0, 1.0}, -1, rectangle, error),
             qPrintable(error));
    std::vector<CadEntity> created;
    QVERIFY2(session.extrude(2, {rectangle}, {0, 0, 2}, created, error),
             qPrintable(error));
    QVERIFY(!created.empty());
    bool hasVolume = false;
    for (const auto& entity : created)
      hasVolume = hasVolume || entity.dim == 3;
    QVERIFY(hasVolume);

    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString path = dir.filePath(QStringLiteral("extrude.stl"));
    QVERIFY2(session.meshToStl(path, error), qPrintable(error));
    Mesh mesh;
    QVERIFY2(loadMesh(path, mesh, error), qPrintable(error));
    QVERIFY(mesh.num_faces() > 0);
    QVERIFY2(session.end(error), qPrintable(error));
  }

  void coneSeamIsWeldedWatertight() {
    GmshCadSession session;
    QString error;
    QVERIFY2(session.begin(QString(), 0.2, error), qPrintable(error));
    int cone = -1;
    QVERIFY2(session.addCone({0, 0, 0}, {1, 0, 0}, 3, 1.0, 0.01, 0.0, -1,
                             cone, error),
             qPrintable(error));
    QVERIFY(cone > 0);

    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString path = dir.filePath(QStringLiteral("cone.stl"));
    QVERIFY2(session.meshToStl(path, error), qPrintable(error));
    Mesh mesh;
    QVERIFY2(loadMesh(path, mesh, error), qPrintable(error));
    const MeshStats stats = computeStats(mesh);
    QVERIFY(stats.valid);
    QVERIFY2(stats.closed, qPrintable(QStringLiteral("boundary edges: %1")
                                          .arg(stats.boundaryEdges)));
    QCOMPARE(stats.boundaryEdges, std::size_t(0));
    QVERIFY2(session.end(error), qPrintable(error));
  }

  void filletRoundsAnEdge() {
    GmshCadSession session;
    QString error;
    QVERIFY2(session.begin(QString(), 0.5, error), qPrintable(error));
    int box = -1;
    QVERIFY2(session.addBox({-1, -1, -1}, {2, 2, 2}, -1, box, error),
             qPrintable(error));
    QVector<CadEntityInfo> curves;
    QVERIFY2(session.listEntities(1, curves, error), qPrintable(error));
    QVERIFY(!curves.isEmpty());

    std::vector<CadEntity> out;
    QVERIFY2(session.fillet({box}, {curves.front().entity.tag}, 0.15, true, out,
                            error),
             qPrintable(error));
    QVERIFY(!out.empty());

    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString path = dir.filePath(QStringLiteral("fillet.stl"));
    QVERIFY2(session.meshToStl(path, error), qPrintable(error));
    Mesh mesh;
    QVERIFY2(loadMesh(path, mesh, error), qPrintable(error));
    QVERIFY(mesh.num_faces() > 0);
    QVERIFY2(session.end(error), qPrintable(error));
  }

  void invalidPrimitiveIsRejected() {
    GmshCadSession session;
    QString error;
    QVERIFY2(session.begin(QString(), 0.5, error), qPrintable(error));
    int tag = -1;
    QVERIFY(!session.addBox({0, 0, 0}, {0, 1, 1}, -1, tag, error));
    QVERIFY(!error.isEmpty());
    QVERIFY2(session.end(error), qPrintable(error));
  }
};

QTEST_GUILESS_MAIN(TstGmshCad)
#include "tst_gmshcad.moc"
