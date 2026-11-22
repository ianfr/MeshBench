// Unit tests for the scriptable command layer and object-tree JSON.
//
// These tests exercise the parts of the MCP-facing API that do not need a
// GUI: the tool catalog, the argument reader, and the pipeline state
// serializer (scene_get_object_tree).

#include "Commands.h"
#include "MeshDocument.h"
#include "MeshOps.h"
#include "test_utils.h"

#include <QtTest>

#include <QJsonArray>
#include <QJsonObject>

#include <cmath>

using namespace meshbench;

namespace {

bool catalogHas(const QVector<CommandInfo>& tools, const QString& name) {
  for (const auto& tool : tools) {
    if (tool.name == name)
      return true;
  }
  return false;
}

} // namespace

class TstCommands : public QObject {
  Q_OBJECT
private slots:
  void catalogCoversAllMenus() {
    const QVector<CommandInfo> tools = commandCatalog();
    QVERIFY(!tools.isEmpty());

    const QStringList required = {
        // File
        QStringLiteral("file_open_mesh"),
        QStringLiteral("file_import_meshes"),
        QStringLiteral("file_import_feature_edges"),
        QStringLiteral("file_save_active_stage"),
        QStringLiteral("file_export_feature_edges"),
        QStringLiteral("file_export_scene_image"),
        QStringLiteral("file_clear"),
        QStringLiteral("file_quit"),
        // Operations (search + every filter)
        QStringLiteral("operations_search"),
        QStringLiteral("operations_boolean"),
        QStringLiteral("operations_center"),
        QStringLiteral("operations_transform"),
        QStringLiteral("operations_alpha_wrap"),
        QStringLiteral("operations_isotropic_remesh"),
        QStringLiteral("operations_extract_feature_edges"),
        QStringLiteral("operations_snap_to_feature_edges"),
        QStringLiteral("operations_autoclean"),
        QStringLiteral("operations_slice"),
        QStringLiteral("operations_combine"),
        QStringLiteral("operations_detect_leaks"),
        QStringLiteral("operations_patch_holes"),
        QStringLiteral("operations_introduce_random_holes"),
        QStringLiteral("operations_shrinkwrap"),
        QStringLiteral("operations_create_primitive"),
        QStringLiteral("operations_gmsh_remesh"),
        // MCP-only low-level Gmsh CAD session
        QStringLiteral("gmsh_session_begin"),
        QStringLiteral("gmsh_session_end"),
        QStringLiteral("gmsh_session_abort"),
        QStringLiteral("gmsh_session_status"),
        QStringLiteral("gmsh_list_entities"),
        QStringLiteral("gmsh_set_mesh_size"),
        QStringLiteral("gmsh_refresh"),
        QStringLiteral("gmsh_import_cad"),
        QStringLiteral("gmsh_export_cad"),
        QStringLiteral("gmsh_add_box"),
        QStringLiteral("gmsh_add_sphere"),
        QStringLiteral("gmsh_add_ellipsoid"),
        QStringLiteral("gmsh_add_cylinder"),
        QStringLiteral("gmsh_add_cone"),
        QStringLiteral("gmsh_add_torus"),
        QStringLiteral("gmsh_add_rectangle"),
        QStringLiteral("gmsh_add_disk"),
        QStringLiteral("gmsh_extrude"),
        QStringLiteral("gmsh_revolve"),
        QStringLiteral("gmsh_fillet"),
        QStringLiteral("gmsh_chamfer"),
        QStringLiteral("gmsh_boolean"),
        QStringLiteral("gmsh_translate"),
        QStringLiteral("gmsh_rotate"),
        QStringLiteral("gmsh_scale"),
        QStringLiteral("gmsh_mirror"),
        QStringLiteral("gmsh_copy"),
        QStringLiteral("gmsh_delete"),
        // View
        QStringLiteral("view_from_pos_x"),
        QStringLiteral("view_from_neg_x"),
        QStringLiteral("view_from_pos_y"),
        QStringLiteral("view_from_neg_y"),
        QStringLiteral("view_from_pos_z"),
        QStringLiteral("view_from_neg_z"),
        QStringLiteral("view_reset_camera"),
        // State
        QStringLiteral("scene_get_object_tree"),
        QStringLiteral("scene_get_image"),
    };
    for (const QString& name : required)
      QVERIFY2(catalogHas(tools, name), qPrintable(QStringLiteral("missing %1").arg(name)));
  }

  void catalogNamesAreUniqueAndSchemasAreObjects() {
    const QVector<CommandInfo> tools = commandCatalog();
    QSet<QString> seen;
    for (const auto& tool : tools) {
      QVERIFY(!tool.name.isEmpty());
      QVERIFY2(!seen.contains(tool.name), qPrintable(tool.name));
      seen.insert(tool.name);
      QVERIFY(tool.inputSchema.value(QStringLiteral("type")).toString() ==
              QStringLiteral("object"));
      QVERIFY(tool.inputSchema.contains(QStringLiteral("properties")));
    }
  }

  void treeStateReportsPipelineAndActiveStage() {
    MeshDocument doc;
    Mesh cube = testutil::makeClosedCube();
    doc.setSource(std::move(cube), QStringLiteral("cube.stl"));
    Mesh centered = testutil::makeClosedCube();
    doc.appendOperation(QStringLiteral("Center"), QStringLiteral("at volume center"),
                        std::move(centered));
    doc.setProbes({{0.5, 0.5, 0.5, true}, {2.0, 2.0, 2.0, false}});

    const QJsonObject state =
        documentTreeStateJson(doc, QSet<int>{1}, QSet<int>{0, 1}, 1,
                              QSet<int>{3, 1, 2},
                              {{1.0, 2.0, 3.0, false}});

    QCOMPARE(state.value(QStringLiteral("has_mesh")).toBool(), true);
    QCOMPARE(state.value(QStringLiteral("stage_count")).toInt(), 2);
    QCOMPARE(state.value(QStringLiteral("active_stage")).toInt(), 1);

    const QJsonArray visible = state.value(QStringLiteral("visible_stages")).toArray();
    QCOMPARE(visible.size(), 1);
    QCOMPARE(visible.at(0).toInt(), 1);

    const QJsonArray combine =
        state.value(QStringLiteral("combine_selection")).toArray();
    QCOMPARE(combine.size(), 2);

    const QJsonArray stages = state.value(QStringLiteral("stages")).toArray();
    QCOMPARE(stages.size(), 2);
    const QJsonObject first = stages.at(0).toObject();
    QCOMPARE(first.value(QStringLiteral("label")).toString(),
             QStringLiteral("cube.stl"));
    QCOMPARE(first.value(QStringLiteral("active")).toBool(), false);
    QCOMPARE(first.value(QStringLiteral("visible")).toBool(), false);
    QCOMPARE(first.value(QStringLiteral("parent")).toInt(), -1);
    const QJsonObject mesh = first.value(QStringLiteral("mesh")).toObject();
    QCOMPARE(mesh.value(QStringLiteral("faces")).toInt(), 12);
    QCOMPARE(mesh.value(QStringLiteral("vertices")).toInt(), 8);
    QCOMPARE(mesh.value(QStringLiteral("closed")).toBool(), true);

    const QJsonObject second = stages.at(1).toObject();
    QCOMPARE(second.value(QStringLiteral("label")).toString(),
             QStringLiteral("Center"));
    QCOMPARE(second.value(QStringLiteral("active")).toBool(), true);
    QCOMPARE(second.value(QStringLiteral("probes")).toArray().size(), 2);

    const QJsonObject selection = state.value(QStringLiteral("selection")).toObject();
    QCOMPARE(selection.value(QStringLiteral("stage")).toInt(), 1);
    QCOMPARE(selection.value(QStringLiteral("count")).toInt(), 3);
    const QJsonArray faces = selection.value(QStringLiteral("faces")).toArray();
    QCOMPARE(faces.size(), 3);
    QCOMPARE(faces.at(0).toInt(), 1);
    QCOMPARE(faces.at(1).toInt(), 2);
    QCOMPARE(faces.at(2).toInt(), 3);

    const QJsonArray editorProbes =
        state.value(QStringLiteral("editor_probes")).toArray();
    QCOMPARE(editorProbes.size(), 1);
    QCOMPARE(editorProbes.at(0).toObject().value(QStringLiteral("inside")).toBool(),
             false);
  }

  void treeStateHandlesEmptyDocument() {
    MeshDocument doc;
    const QJsonObject state =
        documentTreeStateJson(doc, {}, {}, -1, {}, {});
    QCOMPARE(state.value(QStringLiteral("has_mesh")).toBool(), false);
    QCOMPARE(state.value(QStringLiteral("active_stage")).toInt(), -1);
    QCOMPARE(state.value(QStringLiteral("stage_count")).toInt(), 0);
    QCOMPARE(state.value(QStringLiteral("stages")).toArray().size(), 0);
    QCOMPARE(state.value(QStringLiteral("selection"))
                 .toObject()
                 .value(QStringLiteral("count"))
                 .toInt(),
             0);
  }

  void argumentReaderAcceptsValuesAndDefaults() {
    CommandArgs args(QJsonObject{{QStringLiteral("name"), QStringLiteral("x")},
                                 {QStringLiteral("n"), 2.5},
                                 {QStringLiteral("i"), 4},
                                 {QStringLiteral("b"), true},
                                 {QStringLiteral("v"), QJsonArray{1, 2, 3}}},
                     QStringLiteral("test"));
    QCOMPARE(args.string(QStringLiteral("name")), QStringLiteral("x"));
    QCOMPARE(args.number(QStringLiteral("n"), 0.0), 2.5);
    QCOMPARE(args.integer(QStringLiteral("i"), 0), 4);
    QCOMPARE(args.boolean(QStringLiteral("b"), false), true);
    QCOMPARE(args.number(QStringLiteral("missing"), 7.0), 7.0);
    QCOMPARE(args.doubles(QStringLiteral("v"), {}), (QVector<double>{1, 2, 3}));
    QVERIFY(args.ok());
  }

  void argumentReaderRejectsWrongTypes() {
    CommandArgs args(QJsonObject{{QStringLiteral("n"), QStringLiteral("nope")},
                                 {QStringLiteral("i"), 2.5}},
                     QStringLiteral("test"));
    QCOMPARE(args.number(QStringLiteral("n"), 0.0), 0.0);
    QVERIFY(!args.ok());
    QVERIFY(args.error().contains(QStringLiteral("number")));

    CommandArgs intArgs(QJsonObject{{QStringLiteral("i"), 2.5}},
                        QStringLiteral("test"));
    QCOMPARE(intArgs.integer(QStringLiteral("i"), 0), 0);
    QVERIFY(!intArgs.ok());
  }

  void argumentReaderRequiresKey() {
    CommandArgs args({}, QStringLiteral("test"));
    QVERIFY(args.requiredString(QStringLiteral("path")).isEmpty());
    QVERIFY(!args.ok());
    QVERIFY(args.error().contains(QStringLiteral("path")));
  }
};

QTEST_GUILESS_MAIN(TstCommands)
#include "tst_commands.moc"
