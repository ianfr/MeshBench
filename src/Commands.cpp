#include "Commands.h"
#include "MeshOps.h"

#include <QFileInfo>
#include <QJsonDocument>

#include <algorithm>
#include <cmath>
#include <limits>

namespace meshbench {
namespace {

QJsonObject prop(const QString& type, const QString& description) {
  return QJsonObject{{QStringLiteral("type"), type},
                     {QStringLiteral("description"), description}};
}

QJsonObject stringProp(const QString& description) {
  return prop(QStringLiteral("string"), description);
}

QJsonObject numberProp(const QString& description) {
  return prop(QStringLiteral("number"), description);
}

QJsonObject intProp(const QString& description) {
  return prop(QStringLiteral("integer"), description);
}

QJsonObject boolProp(const QString& description) {
  return prop(QStringLiteral("boolean"), description);
}

QJsonObject vec3Prop(const QString& description) {
  return QJsonObject{
      {QStringLiteral("type"), QStringLiteral("array")},
      {QStringLiteral("description"), description},
      {QStringLiteral("items"), QJsonObject{{QStringLiteral("type"), QStringLiteral("number")}}},
      {QStringLiteral("minItems"), 3},
      {QStringLiteral("maxItems"), 3}};
}

QJsonObject vec4Prop(const QString& description) {
  return QJsonObject{
      {QStringLiteral("type"), QStringLiteral("array")},
      {QStringLiteral("description"), description},
      {QStringLiteral("items"), QJsonObject{{QStringLiteral("type"), QStringLiteral("number")}}},
      {QStringLiteral("minItems"), 4},
      {QStringLiteral("maxItems"), 4}};
}

QJsonObject vec2Prop(const QString& description) {
  return QJsonObject{
      {QStringLiteral("type"), QStringLiteral("array")},
      {QStringLiteral("description"), description},
      {QStringLiteral("items"), QJsonObject{{QStringLiteral("type"), QStringLiteral("number")}}},
      {QStringLiteral("minItems"), 2},
      {QStringLiteral("maxItems"), 2}};
}

QJsonObject stringsProp(const QString& description) {
  return QJsonObject{
      {QStringLiteral("type"), QStringLiteral("array")},
      {QStringLiteral("description"), description},
      {QStringLiteral("items"), QJsonObject{{QStringLiteral("type"), QStringLiteral("string")}}}};
}

QJsonObject intsProp(const QString& description) {
  return QJsonObject{
      {QStringLiteral("type"), QStringLiteral("array")},
      {QStringLiteral("description"), description},
      {QStringLiteral("items"), QJsonObject{{QStringLiteral("type"), QStringLiteral("integer")}}}};
}

QJsonObject objectSchema(QJsonObject properties,
                         const QStringList& required = {}) {
  QJsonObject schema{
      {QStringLiteral("type"), QStringLiteral("object")},
      {QStringLiteral("properties"), std::move(properties)},
      {QStringLiteral("additionalProperties"), false}};
  if (!required.isEmpty())
    schema.insert(QStringLiteral("required"), QJsonArray::fromStringList(required));
  return schema;
}

CommandInfo cmd(const QString& name, const QString& description,
                QJsonObject properties, const QStringList& required = {}) {
  return CommandInfo{name, description,
                     objectSchema(std::move(properties), required)};
}

QJsonArray bboxMinJson(const MeshStats& s) {
  return QJsonArray{s.bboxMin[0], s.bboxMin[1], s.bboxMin[2]};
}

QJsonArray bboxMaxJson(const MeshStats& s) {
  return QJsonArray{s.bboxMax[0], s.bboxMax[1], s.bboxMax[2]};
}

} // namespace

QJsonArray probesJson(const QVector<ProbePoint>& probes) {
  QJsonArray out;
  for (const auto& p : probes) {
    out.append(QJsonObject{{QStringLiteral("x"), p.x},
                           {QStringLiteral("y"), p.y},
                           {QStringLiteral("z"), p.z},
                           {QStringLiteral("inside"), p.inside}});
  }
  return out;
}

QJsonObject documentTreeStateJson(const MeshDocument& doc,
                                  const QSet<int>& visibleStages,
                                  const QSet<int>& combineSelected,
                                  int selectionStage, const QSet<int>& selection,
                                  const QVector<ProbePoint>& editorProbes) {
  QJsonObject root;
  root.insert(QStringLiteral("format_version"), 1);
  root.insert(QStringLiteral("has_mesh"), doc.hasMesh());
  root.insert(QStringLiteral("stage_count"), doc.nodeCount());
  root.insert(QStringLiteral("active_stage"), doc.hasMesh() ? doc.activeIndex() : -1);

  QJsonArray visible;
  QList<int> sortedVisible(visibleStages.begin(), visibleStages.end());
  std::sort(sortedVisible.begin(), sortedVisible.end());
  for (int i : sortedVisible)
    visible.append(i);
  root.insert(QStringLiteral("visible_stages"), visible);

  QJsonArray combine;
  QList<int> sortedCombine(combineSelected.begin(), combineSelected.end());
  std::sort(sortedCombine.begin(), sortedCombine.end());
  for (int i : sortedCombine)
    combine.append(i);
  root.insert(QStringLiteral("combine_selection"), combine);

  QJsonArray stages;
  for (int i = 0; i < doc.nodeCount(); ++i) {
    const PipelineNode& node = doc.node(i);
    const MeshStats stats = computeStats(node.mesh);
    QJsonObject stage;
    stage.insert(QStringLiteral("index"), i);
    stage.insert(QStringLiteral("label"), node.label);
    stage.insert(QStringLiteral("details"), node.details);
    stage.insert(QStringLiteral("timestamp"),
                 node.timestamp.isValid() ? node.timestamp.toString(Qt::ISODate)
                                          : QString());
    stage.insert(QStringLiteral("parent"), node.parent);
    stage.insert(QStringLiteral("active"), i == doc.activeIndex());
    stage.insert(QStringLiteral("visible"), visibleStages.contains(i));
    stage.insert(QStringLiteral("combine_selected"), combineSelected.contains(i));

    QJsonObject mesh;
    mesh.insert(QStringLiteral("valid"), stats.valid);
    mesh.insert(QStringLiteral("vertices"), static_cast<qint64>(stats.vertices));
    mesh.insert(QStringLiteral("faces"), static_cast<qint64>(stats.faces));
    mesh.insert(QStringLiteral("edges"), static_cast<qint64>(stats.edges));
    mesh.insert(QStringLiteral("boundary_edges"),
                static_cast<qint64>(stats.boundaryEdges));
    mesh.insert(QStringLiteral("closed"), stats.closed);
    mesh.insert(QStringLiteral("area"), stats.area);
    mesh.insert(QStringLiteral("volume"), stats.volume);
    mesh.insert(QStringLiteral("bbox_min"), bboxMinJson(stats));
    mesh.insert(QStringLiteral("bbox_max"), bboxMaxJson(stats));
    stage.insert(QStringLiteral("mesh"), mesh);

    stage.insert(QStringLiteral("opacity"),
                 node.opacity >= 0.0 ? QJsonValue(node.opacity)
                                     : QJsonValue(QJsonValue::Null));

    QJsonObject edges;
    edges.insert(QStringLiteral("count"),
                 static_cast<qint64>(node.featureEdges.edges.size()));
    edges.insert(QStringLiteral("points"),
                 static_cast<qint64>(node.featureEdges.points.size()));
    stage.insert(QStringLiteral("feature_edges"), edges);

    QJsonObject leak;
    leak.insert(QStringLiteral("probed"), node.leaks.probed);
    leak.insert(QStringLiteral("has_leak"), node.leaks.hasLeak);
    leak.insert(QStringLiteral("hole_loops"), node.leaks.holeLoops);
    leak.insert(QStringLiteral("regions"), node.leaks.regions);
    leak.insert(QStringLiteral("leak_paths"),
                static_cast<qint64>(node.leaks.leakPaths.size()));
    leak.insert(QStringLiteral("walk_paths"),
                static_cast<qint64>(node.leaks.walkPaths.size()));
    leak.insert(QStringLiteral("grid_res"), node.leaks.gridRes);
    leak.insert(QStringLiteral("cell_size"), node.leaks.cellSize);
    leak.insert(QStringLiteral("has_stored_routes"), node.leaks.hasStoredRoutes());
    leak.insert(QStringLiteral("has_stored_grid"), node.leaks.hasStoredGrid());
    leak.insert(QStringLiteral("warnings"),
                QJsonArray::fromStringList(node.leaks.warnings));
    stage.insert(QStringLiteral("leak"), leak);
    stage.insert(QStringLiteral("probes"), probesJson(node.probes));
    stage.insert(QStringLiteral("face_count"), static_cast<qint64>(node.mesh.num_faces()));
    stage.insert(QStringLiteral("vertex_count"),
                 static_cast<qint64>(node.mesh.num_vertices()));

    stages.append(stage);
  }
  root.insert(QStringLiteral("stages"), stages);

  root.insert(QStringLiteral("editor_probes"), probesJson(editorProbes));

  QJsonObject selectionJson;
  selectionJson.insert(QStringLiteral("stage"), selectionStage);
  selectionJson.insert(QStringLiteral("count"), selection.size());
  QJsonArray faces;
  QList<int> sortedFaces(selection.begin(), selection.end());
  std::sort(sortedFaces.begin(), sortedFaces.end());
  for (int f : sortedFaces)
    faces.append(f);
  selectionJson.insert(QStringLiteral("faces"), faces);
  root.insert(QStringLiteral("selection"), selectionJson);

  // Exit check: keep the documented keys present even for an empty doc.
  Q_ASSERT(root.contains(QStringLiteral("stages")));
  return root;
}

QVector<CommandInfo> commandCatalog() {
  QVector<CommandInfo> tools;

  // --- File menu --------------------------------------------------------
  tools.append(cmd(
      QStringLiteral("file_open_mesh"),
      QStringLiteral("File > Open mesh: replace the document with a mesh "
                     "loaded from disk (STL, OFF or OBJ). Clears the current "
                     "pipeline."),
      {{QStringLiteral("path"), stringProp(QStringLiteral("Absolute or relative path to the mesh."))}},
      {QStringLiteral("path")}));

  tools.append(cmd(
      QStringLiteral("file_import_meshes"),
      QStringLiteral("File > Import meshes: append one or more mesh files as "
                     "new independent source stages without clearing history."),
      {{QStringLiteral("paths"), stringsProp(QStringLiteral("Mesh file paths to import."))}},
      {QStringLiteral("paths")}));

  tools.append(cmd(
      QStringLiteral("file_import_feature_edges"),
      QStringLiteral("File > Import feature edges (eMesh): load an OpenFOAM "
                     "featureEdgeMesh and overlay it on the active stage (or "
                     "add it as an independent edge object when no mesh exists)."),
      {{QStringLiteral("path"), stringProp(QStringLiteral("Path to a .eMesh file."))}},
      {QStringLiteral("path")}));

  tools.append(cmd(
      QStringLiteral("file_save_active_stage"),
      QStringLiteral("File > Save active stage: write the active mesh to STL, "
                     "OFF or OBJ (chosen by the path suffix)."),
      {{QStringLiteral("path"), stringProp(QStringLiteral("Output mesh path."))}},
      {QStringLiteral("path")}));

  tools.append(cmd(
      QStringLiteral("file_export_feature_edges"),
      QStringLiteral("File > Export feature edges as eMesh: write the active "
                     "stage's feature edges as an ASCII OpenFOAM featureEdgeMesh."),
      {{QStringLiteral("path"), stringProp(QStringLiteral("Output .eMesh path."))}},
      {QStringLiteral("path")}));

  tools.append(cmd(
      QStringLiteral("file_export_scene_image"),
      QStringLiteral("File > Export scene image: render the current viewport "
                     "to a PNG file. Equivalent to scene_get_image but writes "
                     "the bytes to disk."),
      {{QStringLiteral("path"), stringProp(QStringLiteral("Output .png path."))},
       {QStringLiteral("width"), intProp(QStringLiteral("Image width in pixels (0 = window width)."))},
       {QStringLiteral("height"), intProp(QStringLiteral("Image height in pixels (0 = window height)."))}},
      {QStringLiteral("path")}));

  tools.append(cmd(
      QStringLiteral("file_clear"),
      QStringLiteral("File > Clear: remove every pipeline stage and reset the viewport."),
      {}));

  tools.append(cmd(
      QStringLiteral("file_quit"),
      QStringLiteral("File > Quit: close the MeshBench window (and therefore "
                     "stop this MCP server)."),
      {}));

  // --- Operations menu --------------------------------------------------
  tools.append(cmd(
      QStringLiteral("operations_search"),
      QStringLiteral("Operations > Search operations: return the operation "
                     "command catalog. Use this instead of opening the GUI "
                     "search dialog."),
      {}));

  tools.append(cmd(
      QStringLiteral("operations_boolean"),
      QStringLiteral("Operations > Boolean with another mesh: corefined union, "
                     "intersection or difference between the active stage and "
                     "a mesh file."),
      {{QStringLiteral("path"), stringProp(QStringLiteral("Second mesh file path."))},
       {QStringLiteral("operation"),
        QJsonObject{{QStringLiteral("type"), QStringLiteral("string")},
                    {QStringLiteral("description"),
                     QStringLiteral("union, intersection or difference "
                                    "(difference = active minus other).")},
                    {QStringLiteral("enum"),
                     QJsonArray{QStringLiteral("union"), QStringLiteral("intersection"),
                                QStringLiteral("difference")}}}}},
      {QStringLiteral("path"), QStringLiteral("operation")}));

  tools.append(cmd(
      QStringLiteral("operations_center"),
      QStringLiteral("Operations > Center at volume center: translate the active "
                     "mesh so its volume centroid is at the origin."),
      {}));

  tools.append(cmd(
      QStringLiteral("operations_transform"),
      QStringLiteral("Operations > Transform: translate and/or rotate the active "
                     "mesh. Rotation is XYZ Euler degrees (X then Y then Z), "
                     "then translation."),
      {{QStringLiteral("translate"), vec3Prop(QStringLiteral("Translation (dx, dy, dz). Default 0,0,0."))},
       {QStringLiteral("rotate_degrees"), vec3Prop(QStringLiteral("XYZ rotation in degrees. Default 0,0,0."))},
       {QStringLiteral("rotate_about_center"), boolProp(QStringLiteral("Rotate about the mesh center (default true) or the world origin."))}}));

  tools.append(cmd(
      QStringLiteral("operations_alpha_wrap"),
      QStringLiteral("Operations > Alpha wrap: CGAL alpha wrap into a closed "
                     "watertight approximation. Parameters are relative to the "
                     "active mesh bounding-box diagonal."),
      {{QStringLiteral("alpha_denominator"), numberProp(QStringLiteral("alpha = bbox diagonal / this. Default 500."))},
       {QStringLiteral("offset_denominator"), numberProp(QStringLiteral("offset = bbox diagonal / this. Default 1000."))}}));

  tools.append(cmd(
      QStringLiteral("operations_isotropic_remesh"),
      QStringLiteral("Operations > Isotropic remesh: CGAL feature-preserving "
                     "isotropic remeshing of the active stage."),
      {{QStringLiteral("target_edge_length"), numberProp(QStringLiteral("Target edge length. Default: bbox diagonal / 50."))},
       {QStringLiteral("iterations"), intProp(QStringLiteral("Remeshing iterations. Default 3."))},
       {QStringLiteral("feature_angle_degrees"), numberProp(QStringLiteral("Feature angle in degrees. Default 30."))}}));

  tools.append(cmd(
      QStringLiteral("operations_extract_feature_edges"),
      QStringLiteral("Operations > Extract feature edges: append a stage "
                     "carrying the active surface plus sharp feature edges."),
      {{QStringLiteral("feature_angle_degrees"), numberProp(QStringLiteral("Feature angle in degrees. Default 30."))},
       {QStringLiteral("include_boundary_edges"), boolProp(QStringLiteral("Include open boundary edges. Default true."))}}));

  tools.append(cmd(
      QStringLiteral("operations_snap_to_feature_edges"),
      QStringLiteral("Operations > Snap to feature edges: constrained Delaunay "
                     "remeshing of the active stage using a feature-edge stage "
                     "as protected polylines."),
      {{QStringLiteral("feature_edge_stage"), intProp(QStringLiteral("Stage index holding the feature edges. Default: nearest earlier stage."))},
       {QStringLiteral("target_edge_length"), numberProp(QStringLiteral("Target edge length. Default: bbox diagonal / 50."))},
       {QStringLiteral("min_facet_angle_degrees"), numberProp(QStringLiteral("Minimum facet angle in degrees (0-30). Default 20."))}}));

  tools.append(cmd(
      QStringLiteral("operations_autoclean"),
      QStringLiteral("Operations > Autoclean (experimental): find clusters of "
                     "small triangles, remove them and refine-fill the holes."),
      {{QStringLiteral("detail_size_ratio"), numberProp(QStringLiteral("Small-triangle threshold vs median triangle size (0-1). Default 0.35."))},
       {QStringLiteral("max_region_fraction"), numberProp(QStringLiteral("Maximum cluster bbox as a fraction of the mesh bbox diagonal. Default 0.05."))},
       {QStringLiteral("min_small_faces"), intProp(QStringLiteral("Minimum triangles in a cluster. Default 8."))},
       {QStringLiteral("grow_rings"), intProp(QStringLiteral("Rings to grow around each cluster. Default 1."))},
       {QStringLiteral("max_hole_edges"), intProp(QStringLiteral("Maximum hole boundary edges to fill. Default 256."))}}));

  tools.append(cmd(
      QStringLiteral("operations_slice"),
      QStringLiteral("Operations > Slice with plane: clip the active stage "
                     "with a plane, keeping the negative side."),
      {{QStringLiteral("origin"), vec3Prop(QStringLiteral("Plane origin (x, y, z). Default: bbox center."))},
       {QStringLiteral("normal"), vec3Prop(QStringLiteral("Plane normal (x, y, z). Default 0,0,1."))}}));

  tools.append(cmd(
      QStringLiteral("operations_combine"),
      QStringLiteral("Operations > Combine selected stages: concatenate stages, "
                     "or sequentially boolean-union them. Omit stages to use "
                     "the current GUI Combine selection."),
      {{QStringLiteral("stages"), intsProp(QStringLiteral("Stage indices to combine (at least 2)."))},
       {QStringLiteral("mode"),
        QJsonObject{{QStringLiteral("type"), QStringLiteral("string")},
                    {QStringLiteral("description"), QStringLiteral("append (default) or union.")},
                    {QStringLiteral("enum"), QJsonArray{QStringLiteral("append"), QStringLiteral("union")}}}}}));

  tools.append(cmd(
      QStringLiteral("operations_detect_leaks"),
      QStringLiteral("Operations > Detect leaks: voxel flood-fill leak probe of "
                     "the active stage using the inside/outside probes in the "
                     "probe editor. Appends a LeakProbe stage."),
      {{QStringLiteral("voxels_per_axis"), intProp(QStringLiteral("Voxel grid resolution per axis. Default 48."))}}));

  tools.append(cmd(
      QStringLiteral("operations_patch_holes"),
      QStringLiteral("Operations > Patch holes: fill leaking boundary loops. By "
                     "default only loops traversed by leak routes are patched."),
      {{QStringLiteral("max_loop_edges"), intProp(QStringLiteral("Maximum boundary-loop edge count to fill. Default 64."))},
       {QStringLiteral("patch_all"), boolProp(QStringLiteral("Patch every boundary loop, ignoring leak paths. Default false."))}}));

  tools.append(cmd(
      QStringLiteral("operations_introduce_random_holes"),
      QStringLiteral("Operations > Introduce random holes: delete N triangles "
                     "deterministically (experiment helper for leak tests)."),
      {{QStringLiteral("triangles"), intProp(QStringLiteral("Triangles to remove. Default 10."))},
       {QStringLiteral("seed"), intProp(QStringLiteral("Random seed. Default 42."))}}));

  tools.append(cmd(
      QStringLiteral("operations_shrinkwrap"),
      QStringLiteral("Operations > Shrinkwrap: snap a fresh enclosing icosphere "
                     "onto the active stage. Appends a translucent Shrinkwrap stage."),
      {{QStringLiteral("subdivisions"), intProp(QStringLiteral("Icosphere subdivisions (0-6). Default 4."))},
       {QStringLiteral("iterations"), intProp(QStringLiteral("Snap iterations. Default 25."))},
       {QStringLiteral("step"), numberProp(QStringLiteral("Attraction step (0-1). Default 0.3."))},
       {QStringLiteral("smoothing"), numberProp(QStringLiteral("Laplacian smoothing (0-0.9). Default 0.2."))},
       {QStringLiteral("even_spacing"), numberProp(QStringLiteral("Tangential even-spacing relaxation (0-1). Default 0."))}}));

  tools.append(cmd(
      QStringLiteral("operations_create_primitive"),
      QStringLiteral("Operations > Create primitive shape: build a Gmsh "
                     "OpenCASCADE primitive as a new source stage."),
      {{QStringLiteral("shape"),
        QJsonObject{{QStringLiteral("type"), QStringLiteral("string")},
                    {QStringLiteral("description"), QStringLiteral("box, sphere, ellipsoid, cylinder, cone or torus.")},
                    {QStringLiteral("enum"),
                     QJsonArray{QStringLiteral("box"), QStringLiteral("sphere"),
                                QStringLiteral("ellipsoid"), QStringLiteral("cylinder"),
                                QStringLiteral("cone"), QStringLiteral("torus")}}}},
       {QStringLiteral("dimensions"), QJsonObject{
            {QStringLiteral("type"), QStringLiteral("array")},
            {QStringLiteral("description"),
             QStringLiteral("Shape dimensions in order: box dx,dy,dz; sphere r; "
                            "ellipsoid a,b,c; cylinder r,h; cone rBase,h; torus R,r.")},
            {QStringLiteral("items"), QJsonObject{{QStringLiteral("type"), QStringLiteral("number")}}}}},
       {QStringLiteral("mesh_size"), numberProp(QStringLiteral("Target element size. Default 0.1."))}},
      {QStringLiteral("shape"), QStringLiteral("dimensions")}));

  tools.append(cmd(
      QStringLiteral("operations_gmsh_remesh"),
      QStringLiteral("Operations > Remesh with Gmsh: reclassify and regenerate "
                     "the active mesh at a target element size."),
      {{QStringLiteral("target_element_size"), numberProp(QStringLiteral("Target element size. Default 0.1."))}}));

  // --- MCP-only low-level Gmsh CAD session ------------------------------
  // These tools are not exposed in the GUI. A session owns exactly one
  // pipeline stage that is refreshed in place after every operation.
  tools.append(cmd(
      QStringLiteral("gmsh_session_begin"),
      QStringLiteral("MCP-only: start a low-level Gmsh/OpenCASCADE CAD session "
                     "and create one pipeline stage for it. Every gmsh_* "
                     "geometry operation updates that same stage in place until "
                     "gmsh_session_end. Optionally import a STEP/IGES/BREP file."),
      {{QStringLiteral("label"), stringProp(QStringLiteral("Stage label suffix. Default GmshCAD."))},
       {QStringLiteral("import_path"), stringProp(QStringLiteral("Optional STEP/IGES/BREP file to import."))},
       {QStringLiteral("mesh_size"), numberProp(QStringLiteral("Surface mesh element size. Default 0.2."))}}));

  tools.append(cmd(
      QStringLiteral("gmsh_session_end"),
      QStringLiteral("MCP-only: finish the CAD session. The single stage keeps "
                     "the final mesh; subsequent gmsh_* operations require a "
                     "new gmsh_session_begin."),
      {}));

  tools.append(cmd(
      QStringLiteral("gmsh_session_abort"),
      QStringLiteral("MCP-only: abandon the CAD session. Optionally removes "
                     "the session's pipeline stage (default true)."),
      {{QStringLiteral("remove_stage"), boolProp(QStringLiteral("Remove the session stage. Default true."))}}));

  tools.append(cmd(
      QStringLiteral("gmsh_session_status"),
      QStringLiteral("MCP-only: report whether a CAD session is active, its "
                     "stage index, mesh size and entity count."),
      {}));

  tools.append(cmd(
      QStringLiteral("gmsh_list_entities"),
      QStringLiteral("MCP-only: list OpenCASCADE entities in the session with "
                     "dimension, tag, type and bounding box. Use the tags for "
                     "fillet/chamfer/extrude/boolean/transform tools."),
      {{QStringLiteral("dim"), intProp(QStringLiteral("Only list this dimension (0=point, 1=curve, 2=surface, 3=volume); -1 lists all. Default -1."))}}));

  tools.append(cmd(
      QStringLiteral("gmsh_set_mesh_size"),
      QStringLiteral("MCP-only: change the session's surface mesh size and "
                     "refresh its stage (set refresh=false to defer the "
                     "refresh until gmsh_session_end)."),
      {{QStringLiteral("mesh_size"), numberProp(QStringLiteral("New positive element size."))},
       {QStringLiteral("refresh"), boolProp(QStringLiteral("Refresh the stage immediately. Default true."))}},
      {QStringLiteral("mesh_size")}));

  tools.append(cmd(
      QStringLiteral("gmsh_refresh"),
      QStringLiteral("MCP-only: regenerate and refresh the session stage's "
                     "surface mesh from the current CAD geometry."),
      {}));

  tools.append(cmd(
      QStringLiteral("gmsh_import_cad"),
      QStringLiteral("MCP-only: import additional STEP/IGES/BREP shapes into "
                     "the active CAD session."),
      {{QStringLiteral("path"), stringProp(QStringLiteral("CAD file path."))},
       {QStringLiteral("highest_dim_only"), boolProp(QStringLiteral("Import only highest-dimensional shapes. Default true."))}},
      {QStringLiteral("path")}));

  tools.append(cmd(
      QStringLiteral("gmsh_export_cad"),
      QStringLiteral("MCP-only: export the current CAD model to a file whose "
                     "extension selects the format (e.g. .step, .brep)."),
      {{QStringLiteral("path"), stringProp(QStringLiteral("Output CAD file path."))}},
      {QStringLiteral("path")}));

  const auto cadTagProp = intProp(QStringLiteral("Optional explicit tag; -1 picks a new one."));
  const auto cadOriginProp = vec3Prop(QStringLiteral("Origin/base point [x,y,z]. Default [0,0,0]."));
  const auto cadCenterProp = vec3Prop(QStringLiteral("Center [x,y,z]. Default [0,0,0]."));
  const auto cadAxisProp = vec3Prop(QStringLiteral("Axis direction [x,y,z]. Default [0,0,1]."));
  const auto dimProp = intProp(QStringLiteral("Entity dimension: 1=curve, 2=surface, 3=volume. Default 3 (2 for extrude/revolve)."));
  const auto tagsProp = intsProp(QStringLiteral("Entity tags (from gmsh_list_entities)."));

  tools.append(cmd(
      QStringLiteral("gmsh_add_box"),
      QStringLiteral("MCP-only: add an OCC box volume to the active CAD session."),
      {{QStringLiteral("origin"), cadOriginProp},
       {QStringLiteral("size"), vec3Prop(QStringLiteral("Box extents [dx,dy,dz]."))},
       {QStringLiteral("tag"), cadTagProp}},
      {QStringLiteral("size")}));

  tools.append(cmd(
      QStringLiteral("gmsh_add_sphere"),
      QStringLiteral("MCP-only: add an OCC sphere volume to the active CAD session."),
      {{QStringLiteral("center"), cadCenterProp},
       {QStringLiteral("radius"), numberProp(QStringLiteral("Sphere radius."))},
       {QStringLiteral("tag"), cadTagProp}},
      {QStringLiteral("radius")}));

  tools.append(cmd(
      QStringLiteral("gmsh_add_ellipsoid"),
      QStringLiteral("MCP-only: add an OCC ellipsoid volume (a scaled sphere) to "
                     "the active CAD session."),
      {{QStringLiteral("center"), cadCenterProp},
       {QStringLiteral("radii"), vec3Prop(QStringLiteral("Semi-axis lengths [a,b,c]."))},
       {QStringLiteral("tag"), cadTagProp}},
      {QStringLiteral("radii")}));

  tools.append(cmd(
      QStringLiteral("gmsh_add_cylinder"),
      QStringLiteral("MCP-only: add an OCC cylinder volume to the active CAD "
                     "session."),
      {{QStringLiteral("base"), cadOriginProp},
       {QStringLiteral("axis"), cadAxisProp},
       {QStringLiteral("height"), numberProp(QStringLiteral("Cylinder height."))},
       {QStringLiteral("radius"), numberProp(QStringLiteral("Cylinder radius."))},
       {QStringLiteral("angle_degrees"), numberProp(QStringLiteral("Optional angular opening in degrees. Default 360."))},
       {QStringLiteral("tag"), cadTagProp}},
      {QStringLiteral("height"), QStringLiteral("radius")}));

  tools.append(cmd(
      QStringLiteral("gmsh_add_cone"),
      QStringLiteral("MCP-only: add an OCC cone/frustum volume to the active CAD "
                     "session."),
      {{QStringLiteral("base"), cadOriginProp},
       {QStringLiteral("axis"), cadAxisProp},
       {QStringLiteral("height"), numberProp(QStringLiteral("Cone height."))},
       {QStringLiteral("radius1"), numberProp(QStringLiteral("Base radius."))},
       {QStringLiteral("radius2"), numberProp(QStringLiteral("Top radius. Default 0 (a point)."))},
       {QStringLiteral("angle_degrees"), numberProp(QStringLiteral("Optional angular opening in degrees. Default 360."))},
       {QStringLiteral("tag"), cadTagProp}},
      {QStringLiteral("height"), QStringLiteral("radius1")}));

  tools.append(cmd(
      QStringLiteral("gmsh_add_torus"),
      QStringLiteral("MCP-only: add an OCC torus volume to the active CAD session."),
      {{QStringLiteral("center"), cadCenterProp},
       {QStringLiteral("major_radius"), numberProp(QStringLiteral("Major radius R."))},
       {QStringLiteral("minor_radius"), numberProp(QStringLiteral("Minor radius r."))},
       {QStringLiteral("angle_degrees"), numberProp(QStringLiteral("Optional angular opening in degrees. Default 360."))},
       {QStringLiteral("tag"), cadTagProp}},
      {QStringLiteral("major_radius"), QStringLiteral("minor_radius")}));

  tools.append(cmd(
      QStringLiteral("gmsh_add_rectangle"),
      QStringLiteral("MCP-only: add a planar rectangle surface in the XY plane. "
                     "Extrude it to create a solid."),
      {{QStringLiteral("origin"), cadOriginProp},
       {QStringLiteral("size"), vec2Prop(QStringLiteral("Rectangle extents [dx,dy]."))},
       {QStringLiteral("tag"), cadTagProp}},
      {QStringLiteral("size")}));

  tools.append(cmd(
      QStringLiteral("gmsh_add_disk"),
      QStringLiteral("MCP-only: add a planar disk/ellipse surface. Extrude or "
                     "revolve it to create a solid."),
      {{QStringLiteral("center"), cadCenterProp},
       {QStringLiteral("radius_x"), numberProp(QStringLiteral("Radius along the local x-axis."))},
       {QStringLiteral("radius_y"), numberProp(QStringLiteral("Radius along the local y-axis. Default radius_x."))},
       {QStringLiteral("z_axis"), vec3Prop(QStringLiteral("Disk normal. Default [0,0,1]."))},
       {QStringLiteral("x_axis"), vec3Prop(QStringLiteral("Local x-axis. Default [1,0,0]."))},
       {QStringLiteral("tag"), cadTagProp}},
      {QStringLiteral("radius_x")}));

  tools.append(cmd(
      QStringLiteral("gmsh_extrude"),
      QStringLiteral("MCP-only: extrude CAD surfaces/curves by a translation "
                     "vector, creating solids/surfaces."),
      {{QStringLiteral("dim"), intProp(QStringLiteral("Source dimension: 1=curve, 2=surface. Default 2."))},
       {QStringLiteral("tags"), tagsProp},
       {QStringLiteral("translate"), vec3Prop(QStringLiteral("Extrusion vector [dx,dy,dz]."))}},
      {QStringLiteral("tags"), QStringLiteral("translate")}));

  tools.append(cmd(
      QStringLiteral("gmsh_revolve"),
      QStringLiteral("MCP-only: revolve CAD surfaces/curves around an axis, "
                     "creating solids/surfaces."),
      {{QStringLiteral("dim"), intProp(QStringLiteral("Source dimension: 1=curve, 2=surface. Default 2."))},
       {QStringLiteral("tags"), tagsProp},
       {QStringLiteral("axis_point"), vec3Prop(QStringLiteral("Point on the revolution axis. Default [0,0,0]."))},
       {QStringLiteral("axis_direction"), vec3Prop(QStringLiteral("Revolution axis direction. Default [0,0,1]."))},
       {QStringLiteral("angle_degrees"), numberProp(QStringLiteral("Revolution angle in degrees."))}},
      {QStringLiteral("tags"), QStringLiteral("angle_degrees")}));

  tools.append(cmd(
      QStringLiteral("gmsh_fillet"),
      QStringLiteral("MCP-only: fillet volume edges (curves) with a radius. "
                     "Use gmsh_list_entities with dim=1 to find curve tags."),
      {{QStringLiteral("volumes"), intsProp(QStringLiteral("Volume tags to fillet."))},
       {QStringLiteral("curves"), intsProp(QStringLiteral("Curve/edge tags to round."))},
       {QStringLiteral("radius"), numberProp(QStringLiteral("Fillet radius."))},
       {QStringLiteral("remove_volume"), boolProp(QStringLiteral("Remove the original volume. Default true."))}},
      {QStringLiteral("volumes"), QStringLiteral("curves"), QStringLiteral("radius")}));

  tools.append(cmd(
      QStringLiteral("gmsh_chamfer"),
      QStringLiteral("MCP-only: chamfer volume edges with a distance. Needs the "
                     "adjacent surfaces as well as the curves."),
      {{QStringLiteral("volumes"), intsProp(QStringLiteral("Volume tags."))},
       {QStringLiteral("curves"), intsProp(QStringLiteral("Curve/edge tags."))},
       {QStringLiteral("surfaces"), intsProp(QStringLiteral("Adjacent surface tags."))},
       {QStringLiteral("distance"), numberProp(QStringLiteral("Chamfer distance."))},
       {QStringLiteral("remove_volume"), boolProp(QStringLiteral("Remove the original volume. Default true."))}},
      {QStringLiteral("volumes"), QStringLiteral("curves"), QStringLiteral("surfaces"), QStringLiteral("distance")}));

  tools.append(cmd(
      QStringLiteral("gmsh_boolean"),
      QStringLiteral("MCP-only: boolean cut/fuse/intersect between volume sets "
                     "in the CAD session. Cutting a cylinder out of a box is "
                     "the standard hole workflow."),
      {{QStringLiteral("operation"),
        QJsonObject{{QStringLiteral("type"), QStringLiteral("string")},
                    {QStringLiteral("description"), QStringLiteral("cut, fuse or intersect.")},
                    {QStringLiteral("enum"), QJsonArray{QStringLiteral("cut"), QStringLiteral("fuse"), QStringLiteral("intersect")}}}},
       {QStringLiteral("objects"), intsProp(QStringLiteral("Object volume tags."))},
       {QStringLiteral("tools"), intsProp(QStringLiteral("Tool volume tags."))},
       {QStringLiteral("remove_object"), boolProp(QStringLiteral("Remove object inputs. Default true."))},
       {QStringLiteral("remove_tool"), boolProp(QStringLiteral("Remove tool inputs. Default true."))}},
      {QStringLiteral("operation"), QStringLiteral("objects"), QStringLiteral("tools")}));

  tools.append(cmd(
      QStringLiteral("gmsh_translate"),
      QStringLiteral("MCP-only: translate CAD entities in the active session."),
      {{QStringLiteral("dim"), dimProp},
       {QStringLiteral("tags"), tagsProp},
       {QStringLiteral("translate"), vec3Prop(QStringLiteral("Translation [dx,dy,dz]."))}},
      {QStringLiteral("tags"), QStringLiteral("translate")}));

  tools.append(cmd(
      QStringLiteral("gmsh_rotate"),
      QStringLiteral("MCP-only: rotate CAD entities around an axis."),
      {{QStringLiteral("dim"), dimProp},
       {QStringLiteral("tags"), tagsProp},
       {QStringLiteral("axis_point"), vec3Prop(QStringLiteral("Point on the rotation axis. Default [0,0,0]."))},
       {QStringLiteral("axis_direction"), vec3Prop(QStringLiteral("Rotation axis direction. Default [0,0,1]."))},
       {QStringLiteral("angle_degrees"), numberProp(QStringLiteral("Rotation angle in degrees."))}},
      {QStringLiteral("tags"), QStringLiteral("angle_degrees")}));

  tools.append(cmd(
      QStringLiteral("gmsh_scale"),
      QStringLiteral("MCP-only: scale CAD entities about a center point."),
      {{QStringLiteral("dim"), dimProp},
       {QStringLiteral("tags"), tagsProp},
       {QStringLiteral("center"), vec3Prop(QStringLiteral("Scale center. Default [0,0,0]."))},
       {QStringLiteral("factors"), vec3Prop(QStringLiteral("Scale factors [a,b,c]."))}},
      {QStringLiteral("tags"), QStringLiteral("factors")}));

  tools.append(cmd(
      QStringLiteral("gmsh_mirror"),
      QStringLiteral("MCP-only: mirror CAD entities across a plane "
                     "a*x+b*y+c*z+d=0."),
      {{QStringLiteral("dim"), dimProp},
       {QStringLiteral("tags"), tagsProp},
       {QStringLiteral("plane"), vec4Prop(QStringLiteral("Plane coefficients [a,b,c,d]."))}},
      {QStringLiteral("tags"), QStringLiteral("plane")}));

  tools.append(cmd(
      QStringLiteral("gmsh_copy"),
      QStringLiteral("MCP-only: copy CAD entities, returning the new tags."),
      {{QStringLiteral("dim"), dimProp},
       {QStringLiteral("tags"), tagsProp}},
      {QStringLiteral("tags")}));

  tools.append(cmd(
      QStringLiteral("gmsh_delete"),
      QStringLiteral("MCP-only: delete CAD entities from the session."),
      {{QStringLiteral("dim"), dimProp},
       {QStringLiteral("tags"), tagsProp},
       {QStringLiteral("recursive"), boolProp(QStringLiteral("Recursively delete boundary entities. Default true."))}},
      {QStringLiteral("tags")}));

  // --- View menu --------------------------------------------------------
  const auto viewCmd = [](const QString& name, const QString& description) {
    return cmd(name, description, {});
  };
  tools.append(viewCmd(QStringLiteral("view_from_pos_x"),
                       QStringLiteral("View > View from +X: snap the camera to look along +X.")));
  tools.append(viewCmd(QStringLiteral("view_from_neg_x"),
                       QStringLiteral("View > View from -X: snap the camera to look along -X.")));
  tools.append(viewCmd(QStringLiteral("view_from_pos_y"),
                       QStringLiteral("View > View from +Y: snap the camera to look along +Y.")));
  tools.append(viewCmd(QStringLiteral("view_from_neg_y"),
                       QStringLiteral("View > View from -Y: snap the camera to look along -Y.")));
  tools.append(viewCmd(QStringLiteral("view_from_pos_z"),
                       QStringLiteral("View > View from +Z: snap the camera to look along +Z.")));
  tools.append(viewCmd(QStringLiteral("view_from_neg_z"),
                       QStringLiteral("View > View from -Z: snap the camera to look along -Z.")));
  tools.append(viewCmd(QStringLiteral("view_reset_camera"),
                       QStringLiteral("View > Reset camera: fit the whole scene in the viewport.")));

  // --- Scene inspection / state helpers ---------------------------------
  tools.append(cmd(
      QStringLiteral("scene_get_object_tree"),
      QStringLiteral("Return the current object tree (pipeline) state as JSON: "
                     "stages, labels, parent links, visibility, active stage, "
                     "Combine selection, mesh statistics, feature edges, leak "
                     "reports, probes and the viewport face selection."),
      {}));

  tools.append(cmd(
      QStringLiteral("scene_get_image"),
      QStringLiteral("Return a PNG image of the current scene as an MCP image "
                     "content block. Use this to see the viewport after "
                     "running operations or changing the view."),
      {{QStringLiteral("width"), intProp(QStringLiteral("Image width in pixels (0 = window width)."))},
       {QStringLiteral("height"), intProp(QStringLiteral("Image height in pixels (0 = window height)."))}}));

  tools.append(cmd(
      QStringLiteral("scene_set_active_stage"),
      QStringLiteral("Make a pipeline stage active (the stage operations act on "
                     "and the stage whose overlays render)."),
      {{QStringLiteral("index"), intProp(QStringLiteral("Stage index from scene_get_object_tree."))}},
      {QStringLiteral("index")}));

  tools.append(cmd(
      QStringLiteral("scene_set_stage_visible"),
      QStringLiteral("Show or hide a pipeline stage in the viewport (the eye "
                     "checkbox in the object tree)."),
      {{QStringLiteral("index"), intProp(QStringLiteral("Stage index."))},
       {QStringLiteral("visible"), boolProp(QStringLiteral("true to draw the stage, false to hide it."))}},
      {QStringLiteral("index"), QStringLiteral("visible")}));

  tools.append(cmd(
      QStringLiteral("scene_set_stage_opacity"),
      QStringLiteral("Pin a stage's viewport opacity, or pass -1 to return to "
                     "automatic opacity (opaque when active, faint otherwise)."),
      {{QStringLiteral("index"), intProp(QStringLiteral("Stage index."))},
       {QStringLiteral("opacity"), numberProp(QStringLiteral("Opacity from 0 to 1, or -1 for automatic."))}},
      {QStringLiteral("index"), QStringLiteral("opacity")}));

  tools.append(cmd(
      QStringLiteral("scene_set_combine_selection"),
      QStringLiteral("Replace the stages marked for Combine (the Combine column "
                     "checkboxes). Pass an empty array to clear it."),
      {{QStringLiteral("stages"), intsProp(QStringLiteral("Stage indices to mark for Combine."))}},
      {QStringLiteral("stages")}));

  tools.append(cmd(
      QStringLiteral("scene_set_probes"),
      QStringLiteral("Replace the leak-probe editor contents. Probes are used by "
                     "operations_detect_leaks and path-driven patch_holes."),
      {{QStringLiteral("probes"), QJsonObject{
            {QStringLiteral("type"), QStringLiteral("array")},
            {QStringLiteral("description"), QStringLiteral("Probe points.")},
            {QStringLiteral("items"),
             QJsonObject{
                 {QStringLiteral("type"), QStringLiteral("object")},
                 {QStringLiteral("properties"),
                  QJsonObject{{QStringLiteral("x"), numberProp(QStringLiteral("X coordinate."))},
                              {QStringLiteral("y"), numberProp(QStringLiteral("Y coordinate."))},
                              {QStringLiteral("z"), numberProp(QStringLiteral("Z coordinate."))},
                              {QStringLiteral("inside"), boolProp(QStringLiteral("Inside (true) or outside (false)."))}}}}}}}},
      {QStringLiteral("probes")}));

  tools.append(cmd(
      QStringLiteral("scene_set_default_probes"),
      QStringLiteral("Set default probes for the active stage: bbox center "
                     "(inside) and a point beyond the bbox corner (outside)."),
      {}));

  tools.append(cmd(
      QStringLiteral("scene_get_log"),
      QStringLiteral("Return the tail of the MeshBench log."),
      {{QStringLiteral("limit"), intProp(QStringLiteral("Maximum number of lines. Default 200."))}}));

  // Exit check: names are unique (MCP clients key on them).
  QSet<QString> seen;
  for (const auto& tool : tools) {
    if (seen.contains(tool.name))
      qWarning("duplicate command name: %s", qPrintable(tool.name));
    seen.insert(tool.name);
  }
  return tools;
}

CommandArgs::CommandArgs(const QJsonObject& args, QString commandName)
    : m_args(args), m_command(std::move(commandName)) {
  if (!m_error.isEmpty())
    m_error.clear();
}

void CommandArgs::fail(const QString& message) {
  if (m_error.isEmpty())
    m_error = QStringLiteral("%1: %2").arg(m_command, message);
}

QString CommandArgs::string(const QString& key, const QString& fallback) {
  const QJsonValue value = m_args.value(key);
  if (value.isUndefined() || value.isNull())
    return fallback;
  if (!value.isString()) {
    fail(QStringLiteral("argument '%1' must be a string.").arg(key));
    return fallback;
  }
  return value.toString();
}

QString CommandArgs::requiredString(const QString& key) {
  if (!has(key)) {
    fail(QStringLiteral("missing required string argument '%1'.").arg(key));
    return {};
  }
  return string(key);
}

double CommandArgs::number(const QString& key, double fallback) {
  const QJsonValue value = m_args.value(key);
  if (value.isUndefined() || value.isNull())
    return fallback;
  if (!value.isDouble()) {
    fail(QStringLiteral("argument '%1' must be a number.").arg(key));
    return fallback;
  }
  const double v = value.toDouble();
  if (!std::isfinite(v)) {
    fail(QStringLiteral("argument '%1' must be finite.").arg(key));
    return fallback;
  }
  return v;
}

double CommandArgs::requiredNumber(const QString& key) {
  if (!has(key)) {
    fail(QStringLiteral("missing required number argument '%1'.").arg(key));
    return 0.0;
  }
  return number(key, 0.0);
}

int CommandArgs::integer(const QString& key, int fallback) {
  const QJsonValue value = m_args.value(key);
  if (value.isUndefined() || value.isNull())
    return fallback;
  if (!value.isDouble()) {
    fail(QStringLiteral("argument '%1' must be an integer.").arg(key));
    return fallback;
  }
  const double v = value.toDouble();
  if (!std::isfinite(v) || v < std::numeric_limits<int>::min() ||
      v > std::numeric_limits<int>::max() || std::floor(v) != v) {
    fail(QStringLiteral("argument '%1' must be an integer in range.").arg(key));
    return fallback;
  }
  return static_cast<int>(v);
}

int CommandArgs::requiredInteger(const QString& key) {
  if (!has(key)) {
    fail(QStringLiteral("missing required integer argument '%1'.").arg(key));
    return 0;
  }
  return integer(key, 0);
}

bool CommandArgs::boolean(const QString& key, bool fallback) {
  const QJsonValue value = m_args.value(key);
  if (value.isUndefined() || value.isNull())
    return fallback;
  if (!value.isBool()) {
    fail(QStringLiteral("argument '%1' must be a boolean.").arg(key));
    return fallback;
  }
  return value.toBool();
}

QVector<double> CommandArgs::doubles(const QString& key,
                                     const QVector<double>& fallback) {
  const QJsonValue value = m_args.value(key);
  if (value.isUndefined() || value.isNull())
    return fallback;
  if (!value.isArray()) {
    fail(QStringLiteral("argument '%1' must be an array of numbers.").arg(key));
    return fallback;
  }
  const QJsonArray array = value.toArray();
  QVector<double> out;
  out.reserve(array.size());
  for (const QJsonValue& item : array) {
    if (!item.isDouble()) {
      fail(QStringLiteral("argument '%1' must contain only numbers.").arg(key));
      return fallback;
    }
    const double v = item.toDouble();
    if (!std::isfinite(v)) {
      fail(QStringLiteral("argument '%1' must contain only finite numbers.").arg(key));
      return fallback;
    }
    out.append(v);
  }
  return out;
}

QStringList CommandArgs::strings(const QString& key,
                                 const QStringList& fallback) {
  const QJsonValue value = m_args.value(key);
  if (value.isUndefined() || value.isNull())
    return fallback;
  if (!value.isArray()) {
    fail(QStringLiteral("argument '%1' must be an array of strings.").arg(key));
    return fallback;
  }
  QStringList out;
  for (const QJsonValue& item : value.toArray()) {
    if (!item.isString()) {
      fail(QStringLiteral("argument '%1' must contain only strings.").arg(key));
      return fallback;
    }
    out.append(item.toString());
  }
  return out;
}

QJsonObject CommandArgs::object(const QString& key) {
  const QJsonValue value = m_args.value(key);
  if (value.isUndefined() || value.isNull())
    return {};
  if (!value.isObject()) {
    fail(QStringLiteral("argument '%1' must be an object.").arg(key));
    return {};
  }
  return value.toObject();
}

QJsonArray CommandArgs::array(const QString& key, bool required) {
  const QJsonValue value = m_args.value(key);
  if (value.isUndefined() || value.isNull()) {
    if (required)
      fail(QStringLiteral("missing required array argument '%1'.").arg(key));
    return {};
  }
  if (!value.isArray()) {
    fail(QStringLiteral("argument '%1' must be an array.").arg(key));
    return {};
  }
  return value.toArray();
}

} // namespace meshbench
