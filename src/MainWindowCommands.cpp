#include "MainWindow.h"

#include "Commands.h"
#include "GmshCadSession.h"
#include "LeakFix.h"
#include "McpHttpServer.h"
#include "McpProtocol.h"

#include <QDir>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QMetaObject>
#include <QPlainTextEdit>
#include <QSet>
#include <QStringList>
#include <QThread>
#include <QTimer>
#include <QUrl>
#include <QUrlQuery>

#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <vector>

namespace meshbench {
namespace {

QJsonArray stringsToJson(const QStringList& strings) {
  QJsonArray out;
  for (const QString& s : strings)
    out.append(s);
  return out;
}

bool jsonIntegerArray(const QJsonValue& value, QVector<int>& out,
                      QString& error) {
  out.clear();
  if (!value.isArray()) {
    error = QStringLiteral("expected an array of stage indices.");
    return false;
  }
  for (const QJsonValue& item : value.toArray()) {
    if (!item.isDouble() || std::floor(item.toDouble()) != item.toDouble()) {
      error = QStringLiteral("stage indices must be integers.");
      out.clear();
      return false;
    }
    out.append(static_cast<int>(item.toDouble()));
  }
  return true;
}

McpHttpResponse jsonResponse(int status, const QJsonDocument& doc) {
  McpHttpResponse response;
  response.status = status;
  response.contentType = "application/json";
  response.body = doc.toJson(QJsonDocument::Compact).toStdString();
  return response;
}

McpHttpResponse jsonErrorResponse(int status, const QString& message) {
  QJsonObject error{{QStringLiteral("error"), message}};
  return jsonResponse(status, QJsonDocument(error));
}

QJsonObject jsonRpcError(int code, const QString& message) {
  return QJsonObject{
      {QStringLiteral("jsonrpc"), QStringLiteral("2.0")},
      {QStringLiteral("id"), QJsonValue(QJsonValue::Null)},
      {QStringLiteral("error"),
       QJsonObject{{QStringLiteral("code"), code},
                   {QStringLiteral("message"), message}}}};
}

QJsonArray entitiesToJson(const std::vector<CadEntity>& entities) {
  QJsonArray out;
  for (const CadEntity& entity : entities)
    out.append(QJsonObject{{QStringLiteral("dim"), entity.dim},
                           {QStringLiteral("tag"), entity.tag}});
  return out;
}

bool readVec3(CommandArgs& args, const QString& key,
              const std::array<double, 3>& fallback,
              std::array<double, 3>& out, QString& error) {
  const QVector<double> values =
      args.doubles(key, {fallback[0], fallback[1], fallback[2]});
  if (!args.ok()) {
    error = args.error();
    return false;
  }
  if (values.size() != 3) {
    error = QStringLiteral("argument '%1' needs exactly three numbers.")
                .arg(key);
    return false;
  }
  out = {values[0], values[1], values[2]};
  return true;
}

bool readVec4(CommandArgs& args, const QString& key,
              const std::array<double, 4>& fallback,
              std::array<double, 4>& out, QString& error) {
  const QVector<double> values =
      args.doubles(key, {fallback[0], fallback[1], fallback[2], fallback[3]});
  if (!args.ok()) {
    error = args.error();
    return false;
  }
  if (values.size() != 4) {
    error = QStringLiteral("argument '%1' needs exactly four numbers.")
                .arg(key);
    return false;
  }
  out = {values[0], values[1], values[2], values[3]};
  return true;
}

bool readIntTags(CommandArgs& args, const QString& key, bool required,
                 std::vector<int>& out, QString& error) {
  const QJsonArray array = args.array(key, required);
  if (!args.ok()) {
    error = args.error();
    return false;
  }
  if (array.isEmpty()) {
    out.clear();
    if (required) {
      error = QStringLiteral("argument '%1' must not be empty.").arg(key);
      return false;
    }
    return true;
  }
  QVector<int> parsed;
  if (!jsonIntegerArray(array, parsed, error)) {
    error = QStringLiteral("argument '%1': %2").arg(key, error);
    return false;
  }
  out.assign(parsed.begin(), parsed.end());
  return true;
}

} // namespace

QJsonObject MainWindow::objectTreeState() const {
  const QVector<ProbePoint> probes = m_probeTable ? probesFromTable()
                                                  : QVector<ProbePoint>{};
  return documentTreeStateJson(m_doc, m_visible, m_stageSelection,
                               m_selectionStage, m_selection, probes);
}

CommandReply MainWindow::executeCommand(const QString& name,
                                        const QJsonObject& arguments) {
  CommandReply reply;
  const QString command = name.trimmed();
  if (command.isEmpty()) {
    reply.error = QStringLiteral("No command name given.");
    return reply;
  }

  const auto fail = [&reply](const QString& error) {
    reply.ok = false;
    reply.error = error;
    reply.data = QJsonObject{};
    reply.image.clear();
    return reply;
  };
  const auto succeed = [&reply, this](const QString& message) {
    reply.ok = true;
    reply.error.clear();
    reply.message = message;
    reply.data.insert(QStringLiteral("active_stage"),
                      m_doc.hasMesh() ? m_doc.activeIndex() : -1);
    reply.data.insert(QStringLiteral("stage_count"), m_doc.nodeCount());
    return reply;
  };
  // Common small result payload identifying the current active stage.
  const auto activeInfo = [this]() {
    QJsonObject info;
    info.insert(QStringLiteral("active_stage"),
                m_doc.hasMesh() ? m_doc.activeIndex() : -1);
    info.insert(QStringLiteral("stage_count"), m_doc.nodeCount());
    if (m_doc.hasMesh()) {
      const auto& node = m_doc.node(m_doc.activeIndex());
      const MeshStats stats = computeStats(node.mesh);
      info.insert(QStringLiteral("active_label"), node.label);
      info.insert(QStringLiteral("faces"), static_cast<qint64>(stats.faces));
      info.insert(QStringLiteral("vertices"),
                  static_cast<qint64>(stats.vertices));
      info.insert(QStringLiteral("closed"), stats.closed);
    }
    return info;
  };
  const auto withActive = [&activeInfo](CommandReply value) {
    QJsonObject data = activeInfo();
    for (auto it = value.data.begin(); it != value.data.end(); ++it)
      data.insert(it.key(), it.value());
    value.data = data;
    return value;
  };

  // --- File -------------------------------------------------------------
  if (command == QStringLiteral("file_open_mesh")) {
    CommandArgs args(arguments, command);
    const QString path = args.requiredString(QStringLiteral("path"));
    if (!args.ok())
      return fail(args.error());
    QString error;
    if (!loadMeshFile(path, error))
      return fail(error);
    reply.image.clear();
    return withActive(succeed(QStringLiteral("Opened %1.").arg(path)));
  }
  if (command == QStringLiteral("file_import_meshes")) {
    CommandArgs args(arguments, command);
    const QStringList paths = args.strings(QStringLiteral("paths"));
    if (!args.ok())
      return fail(args.error());
    if (paths.isEmpty())
      return fail(QStringLiteral("file_import_meshes: paths is empty."));
    QString error;
    if (!importMeshFiles(paths, error))
      return fail(error);
    return withActive(
        succeed(QStringLiteral("Imported %1 mesh file(s).").arg(paths.size())));
  }
  if (command == QStringLiteral("file_import_feature_edges")) {
    CommandArgs args(arguments, command);
    const QString path = args.requiredString(QStringLiteral("path"));
    if (!args.ok())
      return fail(args.error());
    QString error;
    if (!importFeatureEdgesFile(path, error))
      return fail(error);
    return withActive(succeed(QStringLiteral("Imported feature edges from %1.")
                                  .arg(path)));
  }
  if (command == QStringLiteral("file_save_active_stage")) {
    CommandArgs args(arguments, command);
    const QString path = args.requiredString(QStringLiteral("path"));
    if (!args.ok())
      return fail(args.error());
    QString error;
    if (!saveActiveMeshTo(path, error))
      return fail(error);
    return withActive(succeed(QStringLiteral("Saved the active stage to %1.")
                                  .arg(QFileInfo(path).absoluteFilePath())));
  }
  if (command == QStringLiteral("file_export_feature_edges")) {
    CommandArgs args(arguments, command);
    const QString path = args.requiredString(QStringLiteral("path"));
    if (!args.ok())
      return fail(args.error());
    QString error;
    if (!saveActiveFeatureEdgesTo(path, error))
      return fail(error);
    return withActive(succeed(
        QStringLiteral("Exported feature edges to %1.").arg(path)));
  }
  if (command == QStringLiteral("file_export_scene_image")) {
    CommandArgs args(arguments, command);
    const QString path = args.requiredString(QStringLiteral("path"));
    const int width = args.integer(QStringLiteral("width"), 0);
    const int height = args.integer(QStringLiteral("height"), 0);
    if (!args.ok())
      return fail(args.error());
    if (width < 0 || height < 0 || (width == 0) != (height == 0))
      return fail(QStringLiteral("width and height must both be 0 (current "
                                 "size) or both be positive."));
    QString error;
    if (!saveSceneImageTo(path, width, height, error))
      return fail(error);
    reply.data.insert(QStringLiteral("path"),
                      QFileInfo(path).absoluteFilePath());
    return succeed(QStringLiteral("Exported the scene image to %1.").arg(path));
  }
  if (command == QStringLiteral("file_clear")) {
    clearAll();
    const int stages = m_doc.nodeCount();
    reply.data.insert(QStringLiteral("stage_count"), stages);
    return succeed(QStringLiteral("Cleared the pipeline."));
  }
  if (command == QStringLiteral("file_quit")) {
    // Reply first; closing immediately would tear down the HTTP connection
    // before the client receives its result. A short delay lets the server
    // thread flush the JSON-RPC response.
    QTimer::singleShot(500, this, [this] { close(); });
    return succeed(QStringLiteral("MeshBench is closing."));
  }

  // --- Operations -------------------------------------------------------
  if (command == QStringLiteral("operations_search")) {
    QJsonArray operations;
    for (const CommandInfo& info : commandCatalog()) {
      if (!info.name.startsWith(QStringLiteral("operations_")))
        continue;
      operations.append(QJsonObject{
          {QStringLiteral("name"), info.name},
          {QStringLiteral("description"), info.description}});
    }
    reply.data.insert(QStringLiteral("operations"), operations);
    return succeed(
        QStringLiteral("MeshBench exposes %1 operations.").arg(operations.size()));
  }
  if (command == QStringLiteral("operations_boolean")) {
    CommandArgs args(arguments, command);
    const QString path = args.requiredString(QStringLiteral("path"));
    const QString opName = args.requiredString(QStringLiteral("operation")).toLower();
    if (!args.ok())
      return fail(args.error());
    BooleanOp op;
    if (opName == QStringLiteral("union"))
      op = BooleanOp::Union;
    else if (opName == QStringLiteral("intersection"))
      op = BooleanOp::Intersection;
    else if (opName == QStringLiteral("difference"))
      op = BooleanOp::Difference;
    else
      return fail(QStringLiteral("operation must be union, intersection or "
                                 "difference."));
    QString error;
    if (!applyBoolean(path, op, error))
      return fail(error);
    return withActive(succeed(QStringLiteral("Applied %1 with %2.")
                                  .arg(opName, QFileInfo(path).fileName())));
  }
  if (command == QStringLiteral("operations_center")) {
    CommandArgs args(arguments, command);
    if (!args.ok())
      return fail(args.error());
    QString error;
    if (!applyCenter(error))
      return fail(error);
    return withActive(
        succeed(QStringLiteral("Centered the mesh at its volume centroid.")));
  }
  if (command == QStringLiteral("operations_transform")) {
    CommandArgs args(arguments, command);
    const QVector<double> t =
        args.doubles(QStringLiteral("translate"), {0.0, 0.0, 0.0});
    const QVector<double> r =
        args.doubles(QStringLiteral("rotate_degrees"), {0.0, 0.0, 0.0});
    const bool aboutCenter =
        args.boolean(QStringLiteral("rotate_about_center"), true);
    if (!args.ok())
      return fail(args.error());
    if (t.size() != 3 || r.size() != 3)
      return fail(QStringLiteral("translate and rotate_degrees need exactly "
                                 "three numbers."));
    TransformParams params;
    params.translate = {t[0], t[1], t[2]};
    params.rotateDeg = {r[0], r[1], r[2]};
    params.rotateAboutCenter = aboutCenter;
    QString error;
    if (!applyTransform(params, error))
      return fail(error);
    return withActive(succeed(QStringLiteral("Transformed the active stage.")));
  }
  if (command == QStringLiteral("operations_alpha_wrap")) {
    CommandArgs args(arguments, command);
    const double alpha =
        args.number(QStringLiteral("alpha_denominator"), 500.0);
    const double offset =
        args.number(QStringLiteral("offset_denominator"), 1000.0);
    if (!args.ok())
      return fail(args.error());
    QString error;
    if (!applyAlphaWrap(alpha, offset, error))
      return fail(error);
    return withActive(succeed(QStringLiteral("Applied CGAL alpha wrap.")));
  }
  if (command == QStringLiteral("operations_isotropic_remesh")) {
    CommandArgs args(arguments, command);
    // A default edge length of 0 means "derive from the bbox", exactly like
    // the dialog's default. The algorithm rejects 0, so compute it here.
    double edge = args.number(QStringLiteral("target_edge_length"), 0.0);
    const int iterations = args.integer(QStringLiteral("iterations"), 3);
    const double angle =
        args.number(QStringLiteral("feature_angle_degrees"), 30.0);
    if (!args.ok())
      return fail(args.error());
    if (edge <= 0.0) {
      if (!ensureActiveMesh(reply.error))
        return fail(reply.error);
      const MeshStats stats = computeStats(m_doc.activeMesh());
      const double dx = stats.bboxMax[0] - stats.bboxMin[0];
      const double dy = stats.bboxMax[1] - stats.bboxMin[1];
      const double dz = stats.bboxMax[2] - stats.bboxMin[2];
      const double diag = std::sqrt(dx * dx + dy * dy + dz * dz);
      edge = diag > 0.0 ? diag / 50.0 : 0.1;
    }
    QString error;
    if (!applyIsotropicRemesh(edge, iterations, angle, error))
      return fail(error);
    reply.data.insert(QStringLiteral("target_edge_length"), edge);
    return withActive(succeed(QStringLiteral("Applied isotropic remeshing.")));
  }
  if (command == QStringLiteral("operations_extract_feature_edges")) {
    CommandArgs args(arguments, command);
    const double angle =
        args.number(QStringLiteral("feature_angle_degrees"), 30.0);
    const bool boundary =
        args.boolean(QStringLiteral("include_boundary_edges"), true);
    if (!args.ok())
      return fail(args.error());
    QString error;
    if (!applyExtractFeatureEdges(angle, boundary, error))
      return fail(error);
    if (m_doc.hasMesh()) {
      reply.data.insert(
          QStringLiteral("feature_edges"),
          static_cast<qint64>(m_doc.node(m_doc.activeIndex()).featureEdges.edges.size()));
    }
    return withActive(succeed(QStringLiteral("Extracted feature edges.")));
  }
  if (command == QStringLiteral("operations_snap_to_feature_edges")) {
    CommandArgs args(arguments, command);
    const int source = args.integer(QStringLiteral("feature_edge_stage"), -1);
    double edge = args.number(QStringLiteral("target_edge_length"), 0.0);
    const double angle =
        args.number(QStringLiteral("min_facet_angle_degrees"), 20.0);
    if (!args.ok())
      return fail(args.error());
    if (edge <= 0.0) {
      if (!ensureActiveMesh(reply.error))
        return fail(reply.error);
      const MeshStats stats = computeStats(m_doc.activeMesh());
      const double dx = stats.bboxMax[0] - stats.bboxMin[0];
      const double dy = stats.bboxMax[1] - stats.bboxMin[1];
      const double dz = stats.bboxMax[2] - stats.bboxMin[2];
      const double diag = std::sqrt(dx * dx + dy * dy + dz * dz);
      edge = diag > 0.0 ? diag / 50.0 : 0.1;
    }
    QString error;
    if (!applySnapToFeatureEdges(source, edge, angle, error))
      return fail(error);
    return withActive(
        succeed(QStringLiteral("Snapped the active stage to feature edges.")));
  }
  if (command == QStringLiteral("operations_autoclean")) {
    CommandArgs args(arguments, command);
    AutoCleanParams params;
    params.detailSizeRatio =
        args.number(QStringLiteral("detail_size_ratio"), params.detailSizeRatio);
    params.maxRegionFraction =
        args.number(QStringLiteral("max_region_fraction"), params.maxRegionFraction);
    params.minSmallFaces =
        args.integer(QStringLiteral("min_small_faces"), params.minSmallFaces);
    params.growRings =
        args.integer(QStringLiteral("grow_rings"), params.growRings);
    params.maxHoleEdges =
        args.integer(QStringLiteral("max_hole_edges"), params.maxHoleEdges);
    if (!args.ok())
      return fail(args.error());
    AutoCleanResult result;
    QString error;
    if (!applyAutoClean(params, result, error))
      return fail(error);
    reply.data.insert(QStringLiteral("clusters_removed"), result.clustersRemoved);
    reply.data.insert(QStringLiteral("faces_removed"), result.facesRemoved);
    reply.data.insert(QStringLiteral("holes_patched"), result.holesPatched);
    return withActive(succeed(QStringLiteral("Autoclean removed %1 cluster(s).")
                                  .arg(result.clustersRemoved)));
  }
  if (command == QStringLiteral("operations_slice")) {
    CommandArgs args(arguments, command);
    QVector<double> origin =
        args.doubles(QStringLiteral("origin"), {});
    QVector<double> normal =
        args.doubles(QStringLiteral("normal"), {0.0, 0.0, 1.0});
    if (!args.ok())
      return fail(args.error());
    if (origin.isEmpty()) {
      if (!ensureActiveMesh(reply.error))
        return fail(reply.error);
      const MeshStats stats = computeStats(m_doc.activeMesh());
      origin = {(stats.bboxMin[0] + stats.bboxMax[0]) / 2,
                (stats.bboxMin[1] + stats.bboxMax[1]) / 2,
                (stats.bboxMin[2] + stats.bboxMax[2]) / 2};
    }
    if (origin.size() != 3 || normal.size() != 3)
      return fail(QStringLiteral("origin and normal need exactly three "
                                 "numbers."));
    if (std::abs(normal[0]) + std::abs(normal[1]) + std::abs(normal[2]) <
        1e-15)
      return fail(QStringLiteral("slice normal must be non-zero."));
    QString error;
    if (!applyClip(makePoint(origin[0], origin[1], origin[2]),
                   Vector3(normal[0], normal[1], normal[2]), error))
      return fail(error);
    return withActive(succeed(QStringLiteral("Sliced the active stage.")));
  }
  if (command == QStringLiteral("operations_combine")) {
    CommandArgs args(arguments, command);
    const QJsonValue stagesValue = arguments.value(QStringLiteral("stages"));
    const QString mode =
        args.string(QStringLiteral("mode"), QStringLiteral("append")).toLower();
    if (!args.ok())
      return fail(args.error());
    QVector<int> stages;
    if (stagesValue.isUndefined() || stagesValue.isNull()) {
      for (int i : selectedStageIndices())
        stages.append(i);
    } else {
      QString parseError;
      if (!jsonIntegerArray(stagesValue, stages, parseError))
        return fail(QStringLiteral("operations_combine: %1").arg(parseError));
    }
    if (mode != QStringLiteral("append") && mode != QStringLiteral("union"))
      return fail(QStringLiteral("mode must be append or union."));
    QList<int> stageList;
    for (int i : stages)
      stageList.append(i);
    QString error;
    if (!applyCombine(stageList, mode == QStringLiteral("union"), error))
      return fail(error);
    return withActive(succeed(QStringLiteral("Combined %1 stages (%2).")
                                  .arg(stages.size())
                                  .arg(mode)));
  }
  if (command == QStringLiteral("operations_detect_leaks")) {
    CommandArgs args(arguments, command);
    const int grid = args.integer(QStringLiteral("voxels_per_axis"),
                                  ProbeParams{}.gridRes);
    if (!args.ok())
      return fail(args.error());
    QString error;
    if (!probeMeshAndAppend(m_doc.hasMesh() ? m_doc.activeMesh() : Mesh(),
                            QStringLiteral("LeakProbe"), grid, error))
      return fail(error);
    const auto& leak = m_doc.node(m_doc.activeIndex()).leaks;
    reply.data.insert(QStringLiteral("has_leak"), leak.hasLeak);
    reply.data.insert(QStringLiteral("hole_loops"), leak.holeLoops);
    reply.data.insert(QStringLiteral("leak_paths"),
                      static_cast<qint64>(leak.leakPaths.size()));
    return withActive(succeed(leak.hasLeak
                                  ? QStringLiteral("LEAK detected.")
                                  : QStringLiteral("No leak detected.")));
  }
  if (command == QStringLiteral("operations_patch_holes")) {
    CommandArgs args(arguments, command);
    const int maxEdges = args.integer(QStringLiteral("max_loop_edges"), 64);
    const bool patchAll = args.boolean(QStringLiteral("patch_all"), false);
    if (!args.ok())
      return fail(args.error());
    QString error;
    int loopsFilled = 0;
    if (!applyPatchHoles(maxEdges, patchAll, error, &loopsFilled))
      return fail(error);
    reply.data.insert(QStringLiteral("loops_filled"), loopsFilled);
    return withActive(
        succeed(loopsFilled > 0
                    ? QStringLiteral("Patched %1 leaking hole loop(s).")
                          .arg(loopsFilled)
                    : QStringLiteral("No leaking boundary loop needed "
                                     "patching.")));
  }
  if (command == QStringLiteral("operations_introduce_random_holes")) {
    CommandArgs args(arguments, command);
    const int triangles = args.integer(QStringLiteral("triangles"), 10);
    const int seed = args.integer(QStringLiteral("seed"), 42);
    if (!args.ok())
      return fail(args.error());
    QString error;
    if (!applyIntroduceHoles(triangles, static_cast<unsigned>(seed), error))
      return fail(error);
    return withActive(
        succeed(QStringLiteral("Removed %1 random triangles.").arg(triangles)));
  }
  if (command == QStringLiteral("operations_shrinkwrap")) {
    CommandArgs args(arguments, command);
    ShrinkParams params = m_shrinkParams;
    params.subdivisions =
        args.integer(QStringLiteral("subdivisions"), params.subdivisions);
    params.iterations = args.integer(QStringLiteral("iterations"), params.iterations);
    params.step = args.number(QStringLiteral("step"), params.step);
    params.smooth = args.number(QStringLiteral("smoothing"), params.smooth);
    params.uniformity =
        args.number(QStringLiteral("even_spacing"), params.uniformity);
    if (!args.ok())
      return fail(args.error());
    QString error;
    if (!applyShrinkwrap(params, error))
      return fail(error);
    return withActive(succeed(QStringLiteral("Created a shrinkwrap stage.")));
  }
  if (command == QStringLiteral("operations_create_primitive")) {
    CommandArgs args(arguments, command);
    const QString shapeName =
        args.requiredString(QStringLiteral("shape")).toLower();
    const QJsonArray dimArray =
        args.array(QStringLiteral("dimensions"), true);
    const double meshSize = args.number(QStringLiteral("mesh_size"), 0.1);
    if (!args.ok())
      return fail(args.error());
    PrimitiveParams params;
    if (shapeName == QStringLiteral("box"))
      params.shape = PrimitiveShape::Box;
    else if (shapeName == QStringLiteral("sphere"))
      params.shape = PrimitiveShape::Sphere;
    else if (shapeName == QStringLiteral("ellipsoid"))
      params.shape = PrimitiveShape::Ellipsoid;
    else if (shapeName == QStringLiteral("cylinder"))
      params.shape = PrimitiveShape::Cylinder;
    else if (shapeName == QStringLiteral("cone"))
      params.shape = PrimitiveShape::Cone;
    else if (shapeName == QStringLiteral("torus"))
      params.shape = PrimitiveShape::Torus;
    else
      return fail(QStringLiteral("Unknown primitive shape '%1'.").arg(shapeName));
    params.dims.clear();
    for (const QJsonValue& value : dimArray) {
      if (!value.isDouble() || !std::isfinite(value.toDouble()))
        return fail(QStringLiteral("dimensions must contain only finite numbers."));
      params.dims.append(value.toDouble());
    }
    const QStringList expected = primitiveDimNames(params.shape);
    if (params.dims.size() != expected.size())
      return fail(QStringLiteral("%1 needs %2 dimensions (%3).")
                      .arg(shapeName)
                      .arg(expected.size())
                      .arg(expected.join(QStringLiteral(", "))));
    params.meshSize = meshSize;
    QString error;
    if (!applyCreatePrimitive(params, error))
      return fail(error);
    return withActive(
        succeed(QStringLiteral("Created a %1 primitive.").arg(shapeName)));
  }
  if (command == QStringLiteral("operations_gmsh_remesh")) {
    CommandArgs args(arguments, command);
    const double size =
        args.number(QStringLiteral("target_element_size"), 0.1);
    if (!args.ok())
      return fail(args.error());
    QString error;
    if (!applyGmshRemesh(size, error))
      return fail(error);
    return withActive(succeed(QStringLiteral("Remeshed with Gmsh (size=%1).")
                                  .arg(size)));
  }

  // --- MCP-only low-level Gmsh CAD session ------------------------------
  // Every geometry operation below applies to the single pipeline stage
  // created by gmsh_session_begin, then refreshes that stage in place.
  const auto finishGmshOp = [&](const QString& message,
                                const std::vector<CadEntity>& created) {
    QString refreshError;
    const bool refreshed = updateGmshCadStage(refreshError);
    CommandReply value;
    value.ok = true;
    value.message = message;
    value.data.insert(QStringLiteral("stage"), m_gmshCadStage);
    value.data.insert(QStringLiteral("created"), entitiesToJson(created));
    int count = 0;
    QString countError;
    if (m_gmshCad && m_gmshCad->active() &&
        m_gmshCad->entityCount(0, count, countError))
      value.data.insert(QStringLiteral("entity_count"), count);
    if (!refreshed) {
      value.data.insert(QStringLiteral("mesh_error"), refreshError);
      value.message +=
          QStringLiteral(" Mesh refresh failed: %1").arg(refreshError);
    }
    return withActive(value);
  };

  if (command == QStringLiteral("gmsh_session_begin")) {
    CommandArgs args(arguments, command);
    QString label = args.string(QStringLiteral("label"),
                                QStringLiteral("GmshCAD"));
    const QString importPath = args.string(QStringLiteral("import_path"), {});
    const double meshSize = args.number(QStringLiteral("mesh_size"), 0.2);
    if (!args.ok())
      return fail(args.error());
    if (m_gmshCad && m_gmshCad->active())
      return fail(QStringLiteral("A Gmsh CAD session is already active. Call "
                                 "gmsh_session_end or gmsh_session_abort "
                                 "first."));
    if (!label.startsWith(QStringLiteral("GmshCAD")))
      label = QStringLiteral("GmshCAD: %1").arg(label);

    auto session = std::make_unique<GmshCadSession>();
    QString error;
    if (!session->begin(importPath, meshSize, error))
      return fail(error);
    m_gmshCad = std::move(session);

    // Predict the new stage index: appendSource always appends at the tip.
    m_gmshCadStage = m_doc.nodeCount();
    m_doc.appendSource(label, Mesh());
    m_visible.insert(m_gmshCadStage);
    refreshView();

    if (!importPath.isEmpty()) {
      QString refreshError;
      if (!updateGmshCadStage(refreshError)) {
        const int stage = m_gmshCadStage;
        endGmshCadSession();
        if (stage >= 0 && stage < m_doc.nodeCount())
          deleteStages(QSet<int>{stage});
        return fail(QStringLiteral("Imported %1 but could not mesh it: %2")
                        .arg(importPath, refreshError));
      }
    }

    reply.data.insert(QStringLiteral("stage"), m_gmshCadStage);
    reply.data.insert(QStringLiteral("mesh_size"), meshSize);
    reply.data.insert(QStringLiteral("session_active"), true);
    reply.data.insert(QStringLiteral("import_path"), importPath);
    return withActive(succeed(QStringLiteral("Started a Gmsh CAD session on "
                                             "stage %1 (mesh_size=%2).")
                                  .arg(m_gmshCadStage)
                                  .arg(meshSize)));
  }
  if (command == QStringLiteral("gmsh_session_end")) {
    QString error;
    if (!gmshCadRequireActive(error))
      return fail(error);
    QString refreshError;
    const bool refreshed = updateGmshCadStage(refreshError);
    const int stage = m_gmshCadStage;
    endGmshCadSession();
    reply.data.insert(QStringLiteral("stage"), stage);
    reply.data.insert(QStringLiteral("session_active"), false);
    if (!refreshed)
      reply.data.insert(QStringLiteral("mesh_error"), refreshError);
    return withActive(succeed(
        refreshed
            ? QStringLiteral("Ended the Gmsh CAD session; stage %1 keeps the "
                             "final mesh.")
                  .arg(stage)
            : QStringLiteral("Ended the Gmsh CAD session; final mesh refresh "
                             "failed: %1")
                  .arg(refreshError)));
  }
  if (command == QStringLiteral("gmsh_session_abort")) {
    CommandArgs args(arguments, command);
    const bool removeStage =
        args.boolean(QStringLiteral("remove_stage"), true);
    if (!args.ok())
      return fail(args.error());
    if (!m_gmshCad || !m_gmshCad->active())
      return succeed(QStringLiteral("No Gmsh CAD session is active."));
    const int stage = m_gmshCadStage;
    endGmshCadSession();
    if (removeStage && stage >= 0 && stage < m_doc.nodeCount())
      deleteStages(QSet<int>{stage});
    reply.data.insert(QStringLiteral("stage"), stage);
    reply.data.insert(QStringLiteral("removed"), removeStage);
    return succeed(QStringLiteral("Aborted the Gmsh CAD session%1.")
                       .arg(removeStage ? QStringLiteral(" and removed its "
                                                        "stage")
                                        : QString()));
  }
  if (command == QStringLiteral("gmsh_session_status")) {
    const bool active = m_gmshCad && m_gmshCad->active();
    reply.data.insert(QStringLiteral("active"), active);
    reply.data.insert(QStringLiteral("stage"), active ? m_gmshCadStage : -1);
    reply.data.insert(QStringLiteral("mesh_size"),
                      active ? m_gmshCad->meshSize() : 0.0);
    if (active) {
      int count = 0;
      QString error;
      if (m_gmshCad->entityCount(0, count, error))
        reply.data.insert(QStringLiteral("entity_count"), count);
      else
        reply.data.insert(QStringLiteral("entity_count_error"), error);
    }
    return succeed(active ? QStringLiteral("A Gmsh CAD session is active on "
                                           "stage %1.")
                                .arg(m_gmshCadStage)
                          : QStringLiteral("No Gmsh CAD session is active."));
  }
  if (command == QStringLiteral("gmsh_list_entities")) {
    CommandArgs args(arguments, command);
    const int dim = args.integer(QStringLiteral("dim"), -1);
    if (!args.ok())
      return fail(args.error());
    QString error;
    if (!gmshCadRequireActive(error))
      return fail(error);
    QVector<CadEntityInfo> entities;
    if (!m_gmshCad->listEntities(dim, entities, error))
      return fail(error);
    QJsonArray array;
    for (const CadEntityInfo& info : entities) {
      QJsonArray bbox;
      for (double v : info.bbox)
        bbox.append(v);
      array.append(QJsonObject{{QStringLiteral("dim"), info.entity.dim},
                               {QStringLiteral("tag"), info.entity.tag},
                               {QStringLiteral("type"), info.type},
                               {QStringLiteral("bbox"), bbox}});
    }
    reply.data.insert(QStringLiteral("entities"), array);
    return succeed(QStringLiteral("Listed %1 CAD entity/entities.")
                       .arg(entities.size()));
  }
  if (command == QStringLiteral("gmsh_set_mesh_size")) {
    CommandArgs args(arguments, command);
    const double meshSize =
        args.requiredNumber(QStringLiteral("mesh_size"));
    const bool refresh = args.boolean(QStringLiteral("refresh"), true);
    if (!args.ok())
      return fail(args.error());
    QString error;
    if (!gmshCadRequireActive(error))
      return fail(error);
    if (!m_gmshCad->setMeshSize(meshSize, error))
      return fail(error);
    if (!refresh) {
      reply.data.insert(QStringLiteral("stage"), m_gmshCadStage);
      reply.data.insert(QStringLiteral("mesh_size"), meshSize);
      reply.data.insert(QStringLiteral("refreshed"), false);
      return withActive(succeed(QStringLiteral("Set CAD mesh_size to %1 "
                                               "(refresh deferred).")
                                    .arg(meshSize)));
    }
    return finishGmshOp(QStringLiteral("Set CAD mesh_size to %1.")
                            .arg(meshSize),
                        {});
  }
  if (command == QStringLiteral("gmsh_refresh")) {
    QString error;
    if (!gmshCadRequireActive(error))
      return fail(error);
    if (!updateGmshCadStage(error))
      return fail(error);
    return withActive(succeed(QStringLiteral("Refreshed the CAD stage mesh.")));
  }
  if (command == QStringLiteral("gmsh_import_cad")) {
    CommandArgs args(arguments, command);
    const QString path = args.requiredString(QStringLiteral("path"));
    const bool highestDimOnly =
        args.boolean(QStringLiteral("highest_dim_only"), true);
    if (!args.ok())
      return fail(args.error());
    QString error;
    if (!gmshCadRequireActive(error))
      return fail(error);
    std::vector<CadEntity> created;
    if (!m_gmshCad->importShapes(path, highestDimOnly, created, error))
      return fail(error);
    return finishGmshOp(QStringLiteral("Imported CAD shapes from %1.").arg(path),
                        created);
  }
  if (command == QStringLiteral("gmsh_export_cad")) {
    CommandArgs args(arguments, command);
    const QString path = args.requiredString(QStringLiteral("path"));
    if (!args.ok())
      return fail(args.error());
    QString error;
    if (!gmshCadRequireActive(error))
      return fail(error);
    if (!m_gmshCad->exportModel(path, error))
      return fail(error);
    reply.data.insert(QStringLiteral("path"),
                      QFileInfo(path).absoluteFilePath());
    return succeed(QStringLiteral("Exported the CAD model to %1.").arg(path));
  }
  if (command == QStringLiteral("gmsh_add_box")) {
    CommandArgs args(arguments, command);
    std::array<double, 3> origin{0.0, 0.0, 0.0};
    std::array<double, 3> size{0.0, 0.0, 0.0};
    const int tag = args.integer(QStringLiteral("tag"), -1);
    QString error;
    if (!readVec3(args, QStringLiteral("origin"), origin, origin, error) ||
        !readVec3(args, QStringLiteral("size"), size, size, error))
      return fail(error);
    if (!args.ok())
      return fail(args.error());
    if (!gmshCadRequireActive(error))
      return fail(error);
    int outTag = -1;
    if (!m_gmshCad->addBox(origin, size, tag, outTag, error))
      return fail(error);
    return finishGmshOp(QStringLiteral("Added box volume %1.").arg(outTag),
                        {CadEntity{3, outTag}});
  }
  if (command == QStringLiteral("gmsh_add_sphere")) {
    CommandArgs args(arguments, command);
    std::array<double, 3> center{0.0, 0.0, 0.0};
    const double radius = args.requiredNumber(QStringLiteral("radius"));
    const int tag = args.integer(QStringLiteral("tag"), -1);
    QString error;
    if (!readVec3(args, QStringLiteral("center"), center, center, error) ||
        !args.ok())
      return fail(error.isEmpty() ? args.error() : error);
    if (!gmshCadRequireActive(error))
      return fail(error);
    int outTag = -1;
    if (!m_gmshCad->addSphere(center, radius, tag, outTag, error))
      return fail(error);
    return finishGmshOp(QStringLiteral("Added sphere volume %1.").arg(outTag),
                        {CadEntity{3, outTag}});
  }
  if (command == QStringLiteral("gmsh_add_ellipsoid")) {
    CommandArgs args(arguments, command);
    std::array<double, 3> center{0.0, 0.0, 0.0};
    std::array<double, 3> radii{0.0, 0.0, 0.0};
    const int tag = args.integer(QStringLiteral("tag"), -1);
    QString error;
    if (!readVec3(args, QStringLiteral("center"), center, center, error) ||
        !readVec3(args, QStringLiteral("radii"), radii, radii, error))
      return fail(error);
    if (!args.ok())
      return fail(args.error());
    if (!gmshCadRequireActive(error))
      return fail(error);
    int outTag = -1;
    if (!m_gmshCad->addEllipsoid(center, radii, tag, outTag, error))
      return fail(error);
    return finishGmshOp(QStringLiteral("Added ellipsoid volume %1.").arg(outTag),
                        {CadEntity{3, outTag}});
  }
  if (command == QStringLiteral("gmsh_add_cylinder")) {
    CommandArgs args(arguments, command);
    std::array<double, 3> base{0.0, 0.0, 0.0};
    std::array<double, 3> axis{0.0, 0.0, 1.0};
    const double height = args.requiredNumber(QStringLiteral("height"));
    const double radius = args.requiredNumber(QStringLiteral("radius"));
    const double angle = args.number(QStringLiteral("angle_degrees"), 0.0);
    const int tag = args.integer(QStringLiteral("tag"), -1);
    QString error;
    if (!readVec3(args, QStringLiteral("base"), base, base, error) ||
        !readVec3(args, QStringLiteral("axis"), axis, axis, error))
      return fail(error);
    if (!args.ok())
      return fail(args.error());
    if (!gmshCadRequireActive(error))
      return fail(error);
    int outTag = -1;
    if (!m_gmshCad->addCylinder(base, axis, height, radius, angle, tag, outTag,
                                error))
      return fail(error);
    return finishGmshOp(QStringLiteral("Added cylinder volume %1.").arg(outTag),
                        {CadEntity{3, outTag}});
  }
  if (command == QStringLiteral("gmsh_add_cone")) {
    CommandArgs args(arguments, command);
    std::array<double, 3> base{0.0, 0.0, 0.0};
    std::array<double, 3> axis{0.0, 0.0, 1.0};
    const double height = args.requiredNumber(QStringLiteral("height"));
    const double radius1 = args.requiredNumber(QStringLiteral("radius1"));
    const double radius2 = args.number(QStringLiteral("radius2"), 0.0);
    const double angle = args.number(QStringLiteral("angle_degrees"), 0.0);
    const int tag = args.integer(QStringLiteral("tag"), -1);
    QString error;
    if (!readVec3(args, QStringLiteral("base"), base, base, error) ||
        !readVec3(args, QStringLiteral("axis"), axis, axis, error))
      return fail(error);
    if (!args.ok())
      return fail(args.error());
    if (!gmshCadRequireActive(error))
      return fail(error);
    int outTag = -1;
    if (!m_gmshCad->addCone(base, axis, height, radius1, radius2, angle, tag,
                            outTag, error))
      return fail(error);
    return finishGmshOp(QStringLiteral("Added cone volume %1.").arg(outTag),
                        {CadEntity{3, outTag}});
  }
  if (command == QStringLiteral("gmsh_add_torus")) {
    CommandArgs args(arguments, command);
    std::array<double, 3> center{0.0, 0.0, 0.0};
    const double major =
        args.requiredNumber(QStringLiteral("major_radius"));
    const double minor =
        args.requiredNumber(QStringLiteral("minor_radius"));
    const double angle = args.number(QStringLiteral("angle_degrees"), 0.0);
    const int tag = args.integer(QStringLiteral("tag"), -1);
    QString error;
    if (!readVec3(args, QStringLiteral("center"), center, center, error))
      return fail(error);
    if (!args.ok())
      return fail(args.error());
    if (!gmshCadRequireActive(error))
      return fail(error);
    int outTag = -1;
    if (!m_gmshCad->addTorus(center, major, minor, angle, tag, outTag, error))
      return fail(error);
    return finishGmshOp(QStringLiteral("Added torus volume %1.").arg(outTag),
                        {CadEntity{3, outTag}});
  }
  if (command == QStringLiteral("gmsh_add_rectangle")) {
    CommandArgs args(arguments, command);
    std::array<double, 3> origin{0.0, 0.0, 0.0};
    const QVector<double> size =
        args.doubles(QStringLiteral("size"), {});
    const int tag = args.integer(QStringLiteral("tag"), -1);
    QString error;
    if (!readVec3(args, QStringLiteral("origin"), origin, origin, error))
      return fail(error);
    if (size.size() != 2) {
      error = QStringLiteral("argument 'size' needs exactly two numbers "
                             "(dx, dy).");
      return fail(error);
    }
    if (!args.ok())
      return fail(args.error());
    if (!gmshCadRequireActive(error))
      return fail(error);
    int outTag = -1;
    if (!m_gmshCad->addRectangle(origin, {size[0], size[1]}, tag, outTag,
                                 error))
      return fail(error);
    return finishGmshOp(QStringLiteral("Added rectangle surface %1.").arg(outTag),
                        {CadEntity{2, outTag}});
  }
  if (command == QStringLiteral("gmsh_add_disk")) {
    CommandArgs args(arguments, command);
    std::array<double, 3> center{0.0, 0.0, 0.0};
    std::array<double, 3> zAxis{0.0, 0.0, 1.0};
    std::array<double, 3> xAxis{1.0, 0.0, 0.0};
    const double radiusX = args.requiredNumber(QStringLiteral("radius_x"));
    const double radiusY =
        args.number(QStringLiteral("radius_y"), radiusX);
    const int tag = args.integer(QStringLiteral("tag"), -1);
    QString error;
    if (!readVec3(args, QStringLiteral("center"), center, center, error) ||
        !readVec3(args, QStringLiteral("z_axis"), zAxis, zAxis, error) ||
        !readVec3(args, QStringLiteral("x_axis"), xAxis, xAxis, error))
      return fail(error);
    if (!args.ok())
      return fail(args.error());
    if (!gmshCadRequireActive(error))
      return fail(error);
    int outTag = -1;
    if (!m_gmshCad->addDisk(center, radiusX, radiusY, zAxis, xAxis, tag,
                            outTag, error))
      return fail(error);
    return finishGmshOp(QStringLiteral("Added disk surface %1.").arg(outTag),
                        {CadEntity{2, outTag}});
  }
  if (command == QStringLiteral("gmsh_extrude") ||
      command == QStringLiteral("gmsh_revolve")) {
    CommandArgs args(arguments, command);
    const int dim = args.integer(QStringLiteral("dim"), 2);
    std::vector<int> tags;
    QString error;
    if (!readIntTags(args, QStringLiteral("tags"), true, tags, error))
      return fail(error);
    if (!args.ok())
      return fail(args.error());
    if (!gmshCadRequireActive(error))
      return fail(error);
    std::vector<CadEntity> created;
    if (command == QStringLiteral("gmsh_extrude")) {
      std::array<double, 3> delta{0.0, 0.0, 0.0};
      if (!readVec3(args, QStringLiteral("translate"),
                    {0.0, 0.0, 0.0}, delta, error))
        return fail(error);
      if (!args.ok())
        return fail(args.error());
      if (!m_gmshCad->extrude(dim, tags, delta, created, error))
        return fail(error);
      return finishGmshOp(QStringLiteral("Extruded %1 entities.").arg(tags.size()),
                          created);
    }
    std::array<double, 3> axisPoint{0.0, 0.0, 0.0};
    std::array<double, 3> axisDirection{0.0, 0.0, 1.0};
    const double angle =
        args.requiredNumber(QStringLiteral("angle_degrees"));
    if (!readVec3(args, QStringLiteral("axis_point"), axisPoint, axisPoint,
                  error) ||
        !readVec3(args, QStringLiteral("axis_direction"), axisDirection,
                  axisDirection, error) ||
        !args.ok())
      return fail(error.isEmpty() ? args.error() : error);
    if (!m_gmshCad->revolve(dim, tags, axisPoint, axisDirection, angle,
                            created, error))
      return fail(error);
    return finishGmshOp(
        QStringLiteral("Revolved %1 entities by %2 degrees.")
            .arg(tags.size())
            .arg(angle),
        created);
  }
  if (command == QStringLiteral("gmsh_fillet")) {
    CommandArgs args(arguments, command);
    std::vector<int> volumes;
    std::vector<int> curves;
    QString error;
    if (!readIntTags(args, QStringLiteral("volumes"), true, volumes, error) ||
        !readIntTags(args, QStringLiteral("curves"), true, curves, error))
      return fail(error);
    const double radius = args.requiredNumber(QStringLiteral("radius"));
    const bool removeVolume =
        args.boolean(QStringLiteral("remove_volume"), true);
    if (!args.ok())
      return fail(args.error());
    if (!gmshCadRequireActive(error))
      return fail(error);
    std::vector<CadEntity> created;
    if (!m_gmshCad->fillet(volumes, curves, radius, removeVolume, created,
                           error))
      return fail(error);
    return finishGmshOp(QStringLiteral("Filleted %1 curve(s) at radius %2.")
                            .arg(curves.size())
                            .arg(radius),
                        created);
  }
  if (command == QStringLiteral("gmsh_chamfer")) {
    CommandArgs args(arguments, command);
    std::vector<int> volumes;
    std::vector<int> curves;
    std::vector<int> surfaces;
    QString error;
    if (!readIntTags(args, QStringLiteral("volumes"), true, volumes, error) ||
        !readIntTags(args, QStringLiteral("curves"), true, curves, error) ||
        !readIntTags(args, QStringLiteral("surfaces"), true, surfaces, error))
      return fail(error);
    const double distance =
        args.requiredNumber(QStringLiteral("distance"));
    const bool removeVolume =
        args.boolean(QStringLiteral("remove_volume"), true);
    if (!args.ok())
      return fail(args.error());
    if (!gmshCadRequireActive(error))
      return fail(error);
    std::vector<CadEntity> created;
    if (!m_gmshCad->chamfer(volumes, curves, surfaces, distance, removeVolume,
                            created, error))
      return fail(error);
    return finishGmshOp(QStringLiteral("Chamfered %1 curve(s) at distance %2.")
                            .arg(curves.size())
                            .arg(distance),
                        created);
  }
  if (command == QStringLiteral("gmsh_boolean")) {
    CommandArgs args(arguments, command);
    const QString operation =
        args.requiredString(QStringLiteral("operation"));
    std::vector<int> objects;
    std::vector<int> tools;
    QString error;
    if (!readIntTags(args, QStringLiteral("objects"), true, objects, error) ||
        !readIntTags(args, QStringLiteral("tools"), true, tools, error))
      return fail(error);
    const bool removeObject =
        args.boolean(QStringLiteral("remove_object"), true);
    const bool removeTool =
        args.boolean(QStringLiteral("remove_tool"), true);
    if (!args.ok())
      return fail(args.error());
    if (!gmshCadRequireActive(error))
      return fail(error);
    std::vector<CadEntity> created;
    if (!m_gmshCad->booleanOp(operation, objects, tools, removeObject,
                              removeTool, created, error))
      return fail(error);
    return finishGmshOp(QStringLiteral("Applied CAD boolean %1.")
                            .arg(operation.toLower()),
                        created);
  }
  if (command == QStringLiteral("gmsh_translate") ||
      command == QStringLiteral("gmsh_rotate") ||
      command == QStringLiteral("gmsh_scale") ||
      command == QStringLiteral("gmsh_mirror")) {
    CommandArgs args(arguments, command);
    const int dim = args.integer(QStringLiteral("dim"), 3);
    std::vector<int> tags;
    QString error;
    if (!readIntTags(args, QStringLiteral("tags"), true, tags, error))
      return fail(error);
    if (!args.ok())
      return fail(args.error());
    if (!gmshCadRequireActive(error))
      return fail(error);

    if (command == QStringLiteral("gmsh_translate")) {
      std::array<double, 3> delta{0.0, 0.0, 0.0};
      if (!readVec3(args, QStringLiteral("translate"),
                    {0.0, 0.0, 0.0}, delta, error))
        return fail(error);
      if (!args.ok())
        return fail(args.error());
      if (!m_gmshCad->translate(dim, tags, delta, error))
        return fail(error);
      return finishGmshOp(QStringLiteral("Translated %1 entities.")
                              .arg(tags.size()),
                          {});
    }
    if (command == QStringLiteral("gmsh_rotate")) {
      std::array<double, 3> point{0.0, 0.0, 0.0};
      std::array<double, 3> direction{0.0, 0.0, 1.0};
      const double angle =
          args.requiredNumber(QStringLiteral("angle_degrees"));
      if (!readVec3(args, QStringLiteral("axis_point"), point, point, error) ||
          !readVec3(args, QStringLiteral("axis_direction"), direction,
                    direction, error) ||
          !args.ok())
        return fail(error.isEmpty() ? args.error() : error);
      if (!m_gmshCad->rotate(dim, tags, point, direction, angle, error))
        return fail(error);
      return finishGmshOp(
          QStringLiteral("Rotated %1 entities by %2 degrees.")
              .arg(tags.size())
              .arg(angle),
          {});
    }
    if (command == QStringLiteral("gmsh_scale")) {
      std::array<double, 3> center{0.0, 0.0, 0.0};
      std::array<double, 3> factors{1.0, 1.0, 1.0};
      if (!readVec3(args, QStringLiteral("center"), center, center, error) ||
          !readVec3(args, QStringLiteral("factors"), factors, factors, error))
        return fail(error);
      if (!args.ok())
        return fail(args.error());
      if (!m_gmshCad->scale(dim, tags, center, factors, error))
        return fail(error);
      return finishGmshOp(QStringLiteral("Scaled %1 entities.").arg(tags.size()),
                          {});
    }
    std::array<double, 4> plane{0.0, 0.0, 1.0, 0.0};
    if (!readVec4(args, QStringLiteral("plane"), plane, plane, error))
      return fail(error);
    if (!args.ok())
      return fail(args.error());
    if (!m_gmshCad->mirror(dim, tags, plane, error))
      return fail(error);
    return finishGmshOp(QStringLiteral("Mirrored %1 entities.").arg(tags.size()),
                        {});
  }
  if (command == QStringLiteral("gmsh_copy")) {
    CommandArgs args(arguments, command);
    const int dim = args.integer(QStringLiteral("dim"), 3);
    std::vector<int> tags;
    QString error;
    if (!readIntTags(args, QStringLiteral("tags"), true, tags, error))
      return fail(error);
    if (!args.ok())
      return fail(args.error());
    if (!gmshCadRequireActive(error))
      return fail(error);
    std::vector<CadEntity> created;
    if (!m_gmshCad->copy(dim, tags, created, error))
      return fail(error);
    return finishGmshOp(QStringLiteral("Copied %1 entities.").arg(tags.size()),
                        created);
  }
  if (command == QStringLiteral("gmsh_delete")) {
    CommandArgs args(arguments, command);
    const int dim = args.integer(QStringLiteral("dim"), 3);
    std::vector<int> tags;
    QString error;
    if (!readIntTags(args, QStringLiteral("tags"), true, tags, error))
      return fail(error);
    const bool recursive = args.boolean(QStringLiteral("recursive"), true);
    if (!args.ok())
      return fail(args.error());
    if (!gmshCadRequireActive(error))
      return fail(error);
    if (!m_gmshCad->remove(dim, tags, recursive, error))
      return fail(error);
    return finishGmshOp(QStringLiteral("Deleted %1 entities.").arg(tags.size()),
                        {});
  }

  // --- View -------------------------------------------------------------
  if (command.startsWith(QStringLiteral("view_from_"))) {
    StandardView view;
    if (command == QStringLiteral("view_from_pos_x"))
      view = StandardView::PosX;
    else if (command == QStringLiteral("view_from_neg_x"))
      view = StandardView::NegX;
    else if (command == QStringLiteral("view_from_pos_y"))
      view = StandardView::PosY;
    else if (command == QStringLiteral("view_from_neg_y"))
      view = StandardView::NegY;
    else if (command == QStringLiteral("view_from_pos_z"))
      view = StandardView::PosZ;
    else if (command == QStringLiteral("view_from_neg_z"))
      view = StandardView::NegZ;
    else
      return fail(QStringLiteral("Unknown view direction in '%1'.").arg(command));
    m_view->setStandardView(view);
    return succeed(QStringLiteral("Changed the camera to %1.")
                       .arg(command.mid(QStringLiteral("view_from_").size())));
  }
  if (command == QStringLiteral("view_reset_camera")) {
    m_view->resetCamera();
    return succeed(QStringLiteral("Reset the camera to fit the scene."));
  }

  // --- Scene state ------------------------------------------------------
  if (command == QStringLiteral("scene_get_object_tree")) {
    reply.data = objectTreeState();
    return succeed(QStringLiteral("Object tree: %1 stage(s), active stage %2.")
                       .arg(m_doc.nodeCount())
                       .arg(m_doc.hasMesh() ? m_doc.activeIndex() : -1));
  }
  if (command == QStringLiteral("scene_get_image")) {
    CommandArgs args(arguments, command);
    const int width = args.integer(QStringLiteral("width"), 0);
    const int height = args.integer(QStringLiteral("height"), 0);
    if (!args.ok())
      return fail(args.error());
    if (width < 0 || height < 0 || (width == 0) != (height == 0))
      return fail(QStringLiteral("width and height must both be 0 (current "
                                 "size) or both be positive."));
    QByteArray png;
    QString error;
    if (!m_view->sceneImagePng(width, height, png, error))
      return fail(error);
    reply.image = std::move(png);
    reply.imageMimeType = QStringLiteral("image/png");
    reply.data.insert(QStringLiteral("width"), width);
    reply.data.insert(QStringLiteral("height"), height);
    return succeed(QStringLiteral("Rendered the current scene as a PNG "
                                  "(%1 bytes).")
                       .arg(reply.image.size()));
  }
  if (command == QStringLiteral("scene_set_active_stage")) {
    CommandArgs args(arguments, command);
    const int index = args.requiredInteger(QStringLiteral("index"));
    if (!args.ok())
      return fail(args.error());
    if (index < 0 || index >= m_doc.nodeCount())
      return fail(QStringLiteral("Stage index %1 is out of range (0..%2).")
                      .arg(index)
                      .arg(m_doc.nodeCount() - 1));
    m_doc.setActive(index);
    return succeed(QStringLiteral("Activated stage %1.").arg(index));
  }
  if (command == QStringLiteral("scene_set_stage_visible")) {
    CommandArgs args(arguments, command);
    const int index = args.requiredInteger(QStringLiteral("index"));
    const bool visible = args.boolean(QStringLiteral("visible"), true);
    if (!args.ok())
      return fail(args.error());
    if (index < 0 || index >= m_doc.nodeCount())
      return fail(QStringLiteral("Stage index %1 is out of range.").arg(index));
    if (visible)
      m_visible.insert(index);
    else
      m_visible.remove(index);
    refreshAll();
    return succeed(QStringLiteral("%1 stage %2.")
                       .arg(visible ? QStringLiteral("Showing")
                                    : QStringLiteral("Hiding"))
                       .arg(index));
  }
  if (command == QStringLiteral("scene_set_stage_opacity")) {
    CommandArgs args(arguments, command);
    const int index = args.requiredInteger(QStringLiteral("index"));
    const double opacity = args.requiredNumber(QStringLiteral("opacity"));
    if (!args.ok())
      return fail(args.error());
    if (index < 0 || index >= m_doc.nodeCount())
      return fail(QStringLiteral("Stage index %1 is out of range.").arg(index));
    if (opacity < -1.0 || opacity > 1.0)
      return fail(QStringLiteral("opacity must be between 0 and 1, or -1 for "
                                 "automatic."));
    m_doc.node(index).opacity = opacity;
    updateStagePanel();
    refreshView();
    return succeed(opacity < 0.0
                       ? QStringLiteral("Stage %1 opacity is automatic.").arg(index)
                       : QStringLiteral("Stage %1 opacity pinned to %2.")
                             .arg(index)
                             .arg(opacity));
  }
  if (command == QStringLiteral("scene_set_combine_selection")) {
    CommandArgs args(arguments, command);
    const QJsonArray stages = args.array(QStringLiteral("stages"), true);
    if (!args.ok())
      return fail(args.error());
    QVector<int> parsed;
    QString parseError;
    if (!jsonIntegerArray(stages, parsed, parseError))
      return fail(parseError);
    QSet<int> selection;
    for (int i : parsed) {
      if (i < 0 || i >= m_doc.nodeCount())
        return fail(QStringLiteral("Stage index %1 is out of range.").arg(i));
      selection.insert(i);
    }
    setStageSelection(selection);
    reply.data.insert(QStringLiteral("combine_selection"), parsed.size());
    return succeed(
        QStringLiteral("Combine selection now has %1 stage(s).").arg(selection.size()));
  }
  if (command == QStringLiteral("scene_set_probes")) {
    CommandArgs args(arguments, command);
    const QJsonArray probesArray = args.array(QStringLiteral("probes"), true);
    if (!args.ok())
      return fail(args.error());
    QVector<ProbePoint> probes;
    for (const QJsonValue& value : probesArray) {
      if (!value.isObject())
        return fail(QStringLiteral("each probe must be an object."));
      const QJsonObject obj = value.toObject();
      for (const QString& key :
           {QStringLiteral("x"), QStringLiteral("y"), QStringLiteral("z")}) {
        if (obj.contains(key) && !obj.value(key).isDouble())
          return fail(QStringLiteral("probe.%1 must be a number.").arg(key));
      }
      ProbePoint probe;
      probe.x = obj.value(QStringLiteral("x")).toDouble(0.0);
      probe.y = obj.value(QStringLiteral("y")).toDouble(0.0);
      probe.z = obj.value(QStringLiteral("z")).toDouble(0.0);
      probe.inside = obj.value(QStringLiteral("inside")).toBool(true);
      probes.append(probe);
    }
    probesToTable(probes);
    if (m_doc.hasMesh())
      m_doc.setProbes(probes);
    reply.data.insert(QStringLiteral("probe_count"), probes.size());
    return succeed(QStringLiteral("Set %1 probe(s).").arg(probes.size()));
  }
  if (command == QStringLiteral("scene_set_default_probes")) {
    QString error;
    if (!fillDefaultProbes(error))
      return fail(error);
    reply.data.insert(QStringLiteral("probe_count"), probesFromTable().size());
    return succeed(QStringLiteral("Set default inside/outside probes."));
  }
  if (command == QStringLiteral("scene_get_log")) {
    CommandArgs args(arguments, command);
    const int limit = args.integer(QStringLiteral("limit"), 200);
    if (!args.ok())
      return fail(args.error());
    const QStringList lines =
        m_log ? m_log->toPlainText().split(QLatin1Char('\n')) : QStringList{};
    const int lineCount = static_cast<int>(lines.size());
    const int start = std::max(0, lineCount - std::max(0, limit));
    reply.data.insert(QStringLiteral("lines"),
                      stringsToJson(lines.mid(start)));
    return succeed(QStringLiteral("Returned the last %1 log line(s).")
                       .arg(lineCount - start));
  }

  return fail(QStringLiteral("Unknown command '%1'. Use tools/list for the "
                             "available commands.")
                  .arg(command));
}

bool MainWindow::gmshCadRequireActive(QString& error) const {
  error.clear();
  if (!m_gmshCad || !m_gmshCad->active()) {
    error = QStringLiteral("No Gmsh CAD session is active. Call "
                           "gmsh_session_begin first.");
    return false;
  }
  if (m_gmshCadStage < 0 || m_gmshCadStage >= m_doc.nodeCount()) {
    error = QStringLiteral("The Gmsh CAD session stage no longer exists.");
    return false;
  }
  return true;
}

void MainWindow::endGmshCadSession() {
  if (!m_gmshCad)
    return;
  QString error;
  if (m_gmshCad->active() && !m_gmshCad->end(error) && !error.isEmpty())
    log(QStringLiteral("Gmsh CAD session warning: %1").arg(error));
  m_gmshCad.reset();
  m_gmshCadStage = -1;
}

void MainWindow::validateGmshCadState() {
  if (!m_gmshCad || !m_gmshCad->active())
    return;
  const bool valid =
      m_gmshCadStage >= 0 && m_gmshCadStage < m_doc.nodeCount() &&
      m_doc.node(m_gmshCadStage).label.startsWith(QStringLiteral("GmshCAD"));
  if (valid)
    return;
  log(QStringLiteral("Gmsh CAD session ended because its stage was removed "
                     "or replaced."));
  m_gmshCad->abort();
  m_gmshCad.reset();
  m_gmshCadStage = -1;
}

bool MainWindow::updateGmshCadStage(QString& error) {
  error.clear();
  if (!gmshCadRequireActive(error))
    return false;

  bool hasGeometry = false;
  if (!m_gmshCad->hasSurfaceGeometry(hasGeometry, error))
    return false;

  Mesh mesh;
  if (hasGeometry) {
    if (!m_gmshCad->meshToMesh(mesh, error))
      return false;
    if (mesh.num_vertices() == 0) {
      error = QStringLiteral("Gmsh produced an empty surface mesh. Try a "
                             "smaller mesh_size.");
      return false;
    }
  }

  int entityCount = 0;
  if (!m_gmshCad->entityCount(0, entityCount, error))
    return false;

  auto& node = m_doc.node(m_gmshCadStage);
  node.details = QStringLiteral("CAD session: %1 entities, mesh_size=%2")
                     .arg(entityCount)
                     .arg(m_gmshCad->meshSize());
  m_doc.replaceMesh(m_gmshCadStage, std::move(mesh));
  m_visible.insert(m_gmshCadStage);
  refreshView();

  // Exit check: the in-place stage update must not have removed the node.
  if (m_gmshCadStage < 0 || m_gmshCadStage >= m_doc.nodeCount()) {
    error = QStringLiteral("The Gmsh CAD stage disappeared while refreshing.");
    return false;
  }
  return true;
}

void MainWindow::startMcpServer() {
  // MESHBENCH_MCP_PORT: unset -> 8765, "off"/"none"/"-1" -> disabled,
  // 0 -> ephemeral port, other values -> that TCP port.
  const QString portSetting = qEnvironmentVariable("MESHBENCH_MCP_PORT");
  const QString hostSetting =
      qEnvironmentVariable("MESHBENCH_MCP_HOST", QStringLiteral("127.0.0.1"));
  if (portSetting.compare(QStringLiteral("off"), Qt::CaseInsensitive) == 0 ||
      portSetting.compare(QStringLiteral("none"), Qt::CaseInsensitive) == 0 ||
      portSetting == QStringLiteral("-1")) {
    log(QStringLiteral("MCP server disabled by MESHBENCH_MCP_PORT."));
    return;
  }
  bool ok = true;
  const int port =
      portSetting.isEmpty() ? 8765 : portSetting.toInt(&ok);
  if (!ok || port < 0 || port > 65535) {
    log(QStringLiteral("MCP server: invalid MESHBENCH_MCP_PORT '%1'.")
            .arg(portSetting));
    return;
  }

  m_mcpProtocol = std::make_unique<McpProtocol>(
      [] { return commandCatalog(); },
      [this](const QString& name, const QJsonObject& args) {
        return executeCommand(name, args);
      });
  m_mcpServer = std::make_unique<McpHttpServer>();
  std::string bindError;
  const bool started = m_mcpServer->start(
      hostSetting.toStdString(), static_cast<std::uint16_t>(port),
      [this](const McpHttpRequest& request) {
        return handleHttpRequest(request);
      },
      bindError);
  if (!started) {
    log(QStringLiteral("MCP server failed to start: %1")
            .arg(QString::fromStdString(bindError)));
    m_mcpServer.reset();
    m_mcpProtocol.reset();
    return;
  }
  const quint16 actualPort = m_mcpServer->port();
  log(QStringLiteral("MCP server listening on http://%1:%2/mcp "
                     "(unauthenticated, loopback only).")
          .arg(hostSetting)
          .arg(actualPort));
  log(QStringLiteral("MCP endpoints: POST /mcp, GET /api/tree, "
                     "GET /api/scene.png, GET /api/commands, POST /api/tool."));
}

McpHttpResponse MainWindow::handleHttpRequest(const McpHttpRequest& request) {
  // The HTTP server runs on its own thread; all commands touch the document,
  // the VTK viewport and Qt widgets, so they must run on the GUI thread.
  // Use a shared state + queued invocation instead of BlockingQueuedConnection
  // so a shutting-down event loop can time out without deadlocking.
  struct Pending {
    std::mutex mutex;
    std::condition_variable cv;
    bool done = false;
    McpHttpResponse response;
  };
  auto pending = std::make_shared<Pending>();

  QMetaObject::invokeMethod(
      this,
      [this, pending, request] {
        McpHttpResponse response;
        try {
          response = [this, &request]() {
            const QUrl url = QUrl::fromEncoded(
                QByteArray::fromStdString(request.target));
            const QString path = url.path();
            const QString method =
                QString::fromStdString(request.method).toUpper();
            const auto cors = [](McpHttpResponse& r) {
              r.headers["Access-Control-Allow-Origin"] = "*";
              r.headers["Access-Control-Allow-Methods"] =
                  "GET, POST, OPTIONS";
              r.headers["Access-Control-Allow-Headers"] =
                  "Content-Type, Mcp-Session-Id, Mcp-Protocol-Version";
            };

            if (method == QStringLiteral("OPTIONS")) {
              McpHttpResponse r;
              r.status = 204;
              cors(r);
              return r;
            }

            // MCP Streamable HTTP transport: JSON-RPC over POST.
            if (method == QStringLiteral("POST") &&
                (path == QStringLiteral("/mcp") ||
                 path == QStringLiteral("/") ||
                 path == QStringLiteral("/message"))) {
              QJsonParseError parseError;
              const QJsonDocument doc = QJsonDocument::fromJson(
                  QByteArray::fromStdString(request.body), &parseError);
              if (parseError.error != QJsonParseError::NoError) {
                McpHttpResponse r = jsonResponse(
                    400, QJsonDocument(jsonRpcError(-32700,
                                                    QStringLiteral("Parse error: %1")
                                                        .arg(parseError.errorString()))));
                cors(r);
                return r;
              }
              if (!m_mcpProtocol) {
                McpHttpResponse r = jsonErrorResponse(
                    503, QStringLiteral("MCP protocol is not initialized."));
                cors(r);
                return r;
              }
              if (doc.isObject()) {
                const QJsonObject reply =
                    m_mcpProtocol->handle(doc.object());
                if (reply.isEmpty()) {
                  McpHttpResponse r;
                  r.status = 202; // notification: no response body
                  cors(r);
                  return r;
                }
                McpHttpResponse r = jsonResponse(200, QJsonDocument(reply));
                cors(r);
                return r;
              }
              if (doc.isArray()) {
                QJsonArray replies;
                for (const QJsonValue& item : doc.array()) {
                  if (!item.isObject()) {
                    replies.append(jsonRpcError(
                        -32600, QStringLiteral("Invalid JSON-RPC message.")));
                    continue;
                  }
                  const QJsonObject reply =
                      m_mcpProtocol->handle(item.toObject());
                  if (!reply.isEmpty())
                    replies.append(reply);
                }
                if (replies.isEmpty()) {
                  McpHttpResponse r;
                  r.status = 202;
                  cors(r);
                  return r;
                }
                McpHttpResponse r = jsonResponse(200, QJsonDocument(replies));
                cors(r);
                return r;
              }
              McpHttpResponse r = jsonResponse(
                  400, QJsonDocument(jsonRpcError(
                           -32600, QStringLiteral("Request must be a JSON "
                                                  "object or array."))));
              cors(r);
              return r;
            }

            // Convenience REST endpoints for curl / non-MCP clients.
            if (method == QStringLiteral("GET") &&
                (path == QStringLiteral("/api/tree") ||
                 path == QStringLiteral("/tree"))) {
              const CommandReply reply =
                  executeCommand(QStringLiteral("scene_get_object_tree"), {});
              McpHttpResponse r =
                  reply.ok ? jsonResponse(200, QJsonDocument(reply.data))
                           : jsonErrorResponse(500, reply.error);
              cors(r);
              return r;
            }
            if (method == QStringLiteral("GET") &&
                (path == QStringLiteral("/api/commands") ||
                 path == QStringLiteral("/commands"))) {
              QJsonArray tools;
              for (const CommandInfo& info : commandCatalog()) {
                tools.append(QJsonObject{
                    {QStringLiteral("name"), info.name},
                    {QStringLiteral("description"), info.description},
                    {QStringLiteral("inputSchema"), info.inputSchema}});
              }
              McpHttpResponse r =
                  jsonResponse(200, QJsonDocument(QJsonObject{
                                        {QStringLiteral("tools"), tools}}));
              cors(r);
              return r;
            }
            if (method == QStringLiteral("GET") &&
                (path == QStringLiteral("/api/scene.png") ||
                 path == QStringLiteral("/scene.png"))) {
              const QUrlQuery query(url);
              bool wOk = true, hOk = true;
              const int width =
                  query.queryItemValue(QStringLiteral("width")).isEmpty()
                      ? 0
                      : query.queryItemValue(QStringLiteral("width"))
                            .toInt(&wOk);
              const int height =
                  query.queryItemValue(QStringLiteral("height")).isEmpty()
                      ? 0
                      : query.queryItemValue(QStringLiteral("height"))
                            .toInt(&hOk);
              if (!wOk || !hOk || width < 0 || height < 0 ||
                  (width == 0) != (height == 0)) {
                McpHttpResponse r = jsonErrorResponse(
                    400, QStringLiteral("width and height must both be 0 "
                                        "or both be positive."));
                cors(r);
                return r;
              }
              QByteArray png;
              QString error;
              if (!m_view->sceneImagePng(width, height, png, error)) {
                McpHttpResponse r = jsonErrorResponse(500, error);
                cors(r);
                return r;
              }
              McpHttpResponse r;
              r.status = 200;
              r.contentType = "image/png";
              r.body.assign(png.constData(),
                            static_cast<std::size_t>(png.size()));
              cors(r);
              return r;
            }
            if (method == QStringLiteral("POST") &&
                (path == QStringLiteral("/api/tool") ||
                 path.startsWith(QStringLiteral("/api/commands/")))) {
              const QJsonDocument doc = QJsonDocument::fromJson(
                  QByteArray::fromStdString(request.body));
              QString name;
              QJsonObject args;
              if (path == QStringLiteral("/api/tool")) {
                if (!doc.isObject()) {
                  McpHttpResponse r =
                      jsonErrorResponse(400, QStringLiteral("Body must be a JSON "
                                                            "object."));
                  cors(r);
                  return r;
                }
                const QJsonObject obj = doc.object();
                name = obj.value(QStringLiteral("name")).toString();
                args = obj.value(QStringLiteral("arguments")).toObject();
              } else {
                name = path.mid(QStringLiteral("/api/commands/").size());
                if (!doc.isNull() && !doc.isObject()) {
                  McpHttpResponse r = jsonErrorResponse(
                      400, QStringLiteral("Arguments must be a JSON object or "
                                          "empty."));
                  cors(r);
                  return r;
                }
                args = doc.object();
              }
              if (name.isEmpty()) {
                McpHttpResponse r =
                    jsonErrorResponse(400, QStringLiteral("Missing tool name."));
                cors(r);
                return r;
              }
              const CommandReply reply = executeCommand(name, args);
              QJsonObject out{
                  {QStringLiteral("ok"), reply.ok},
                  {QStringLiteral("message"), reply.message},
                  {QStringLiteral("error"), reply.error},
                  {QStringLiteral("data"), reply.data}};
              if (!reply.image.isEmpty())
                out.insert(QStringLiteral("image_base64"),
                           QString::fromLatin1(reply.image.toBase64()));
              McpHttpResponse r =
                  jsonResponse(reply.ok ? 200 : 400, QJsonDocument(out));
              cors(r);
              return r;
            }
            if (method == QStringLiteral("GET") &&
                (path == QStringLiteral("/") ||
                 path == QStringLiteral("/health"))) {
              QJsonObject info{
                  {QStringLiteral("name"), QStringLiteral("meshbench")},
                  {QStringLiteral("status"),
                   QStringLiteral("ok")},
                  {QStringLiteral("mcp"), QStringLiteral("/mcp")},
                  {QStringLiteral("tree"), QStringLiteral("/api/tree")},
                  {QStringLiteral("image"),
                   QStringLiteral("/api/scene.png")},
                  {QStringLiteral("commands"),
                   QStringLiteral("/api/commands")},
                  {QStringLiteral("command"),
                   QStringLiteral("/api/commands/<tool>")},
                  {QStringLiteral("tool"), QStringLiteral("/api/tool")}};
              McpHttpResponse r =
                  jsonResponse(200, QJsonDocument(info));
              cors(r);
              return r;
            }

            if ((method == QStringLiteral("GET") ||
                 method == QStringLiteral("DELETE")) &&
                path == QStringLiteral("/mcp")) {
              McpHttpResponse r = jsonErrorResponse(
                  405, QStringLiteral("The MCP endpoint accepts POST only."));
              r.headers["Allow"] = "POST";
              cors(r);
              return r;
            }

            McpHttpResponse r = jsonErrorResponse(
                404, QStringLiteral("No endpoint at %1 %2.").arg(method, path));
            cors(r);
            return r;
          }();
        } catch (const std::exception& e) {
          response = jsonErrorResponse(
              500, QStringLiteral("Internal error: %1")
                       .arg(QString::fromUtf8(e.what())));
        } catch (...) {
          response = jsonErrorResponse(
              500, QStringLiteral("Internal error while handling the request."));
        }

        {
          std::lock_guard<std::mutex> lock(pending->mutex);
          if (pending->done)
            return; // requester already timed out
          pending->response = std::move(response);
          pending->done = true;
        }
        pending->cv.notify_all();
      },
      Qt::QueuedConnection);

  std::unique_lock<std::mutex> lock(pending->mutex);
  // CAD booleans and fine surface meshing can take several minutes on large
  // models. Do not turn a healthy long-running command into a 503.
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::seconds(3600);
  while (!pending->done) {
    if (m_mcpShuttingDown.load())
      break;
    if (std::chrono::steady_clock::now() >= deadline)
      break;
    pending->cv.wait_for(lock, std::chrono::milliseconds(250));
  }
  if (!pending->done) {
    McpHttpResponse response;
    response.status = 503;
    response.contentType = "application/json";
    response.body =
        "{\"error\":\"MeshBench did not process the request in time.\"}";
    return response;
  }
  return pending->response;
}

} // namespace meshbench
