#pragma once

// Persistent low-level Gmsh/OpenCASCADE CAD session.
//
// The GUI does not expose these operations. They back the MCP-only
// `gmsh_*` tools: an LLM starts a session (which creates one pipeline
// stage), runs any number of low-level OCC operations (add primitives,
// extrude, revolve, fillet, boolean cut, ...), and ends the session. The
// pipeline stage is refreshed in place after each operation, so the object
// tree always contains exactly one node for the whole CAD sequence.
//
// Gmsh's C++ API keeps process-global model state, so only one session may
// be active at a time. All methods must be called from one thread (the GUI
// thread in MeshBench).

#include "MeshTypes.h"

#include <QString>
#include <QVector>

#include <array>
#include <memory>
#include <vector>

namespace meshbench {

struct CadEntity {
  int dim = -1;
  int tag = -1;
};

struct CadEntityInfo {
  CadEntity entity;
  QString type;
  std::array<double, 6> bbox = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

class GmshCadSession {
public:
  GmshCadSession();
  ~GmshCadSession();

  GmshCadSession(const GmshCadSession&) = delete;
  GmshCadSession& operator=(const GmshCadSession&) = delete;

  // Start a new empty OCC model, optionally importing a STEP/IGES/BREP file.
  bool begin(const QString& importPath, double meshSize, QString& error);
  // Finalize the Gmsh session. Geometry is discarded; the caller keeps the
  // last meshed stage.
  bool end(QString& error);
  // Finalize without reporting errors (destruction / abandoned session).
  void abort();
  bool active() const;
  double meshSize() const;
  bool setMeshSize(double meshSize, QString& error);

  // Primitive creation. tag < 0 picks a new tag automatically. outTag is a
  // volume tag (dim 3). Angles are in degrees.
  bool addBox(const std::array<double, 3>& origin,
              const std::array<double, 3>& size, int tag, int& outTag,
              QString& error);
  bool addSphere(const std::array<double, 3>& center, double radius, int tag,
                 int& outTag, QString& error);
  bool addEllipsoid(const std::array<double, 3>& center,
                    const std::array<double, 3>& radii, int tag, int& outTag,
                    QString& error);
  bool addCylinder(const std::array<double, 3>& base,
                   const std::array<double, 3>& axisDirection, double height,
                   double radius, double angleDegrees, int tag, int& outTag,
                   QString& error);
  bool addCone(const std::array<double, 3>& base,
               const std::array<double, 3>& axisDirection, double height,
               double radius1, double radius2, double angleDegrees, int tag,
               int& outTag, QString& error);
  bool addTorus(const std::array<double, 3>& center, double majorRadius,
                double minorRadius, double angleDegrees, int tag, int& outTag,
                QString& error);
  bool addRectangle(const std::array<double, 3>& origin,
                    std::array<double, 2> size, int tag, int& outTag,
                    QString& error);
  bool addDisk(const std::array<double, 3>& center, double radiusX,
               double radiusY, const std::array<double, 3>& zAxis,
               const std::array<double, 3>& xAxis, int tag, int& outTag,
               QString& error);

  // Sweep/extrude existing entities (dim 2 surfaces by default).
  bool extrude(int dim, const std::vector<int>& tags,
               const std::array<double, 3>& delta,
               std::vector<CadEntity>& out, QString& error);
  bool revolve(int dim, const std::vector<int>& tags,
               const std::array<double, 3>& axisPoint,
               const std::array<double, 3>& axisDirection, double angleDegrees,
               std::vector<CadEntity>& out, QString& error);

  // CAD editing.
  bool fillet(const std::vector<int>& volumes, const std::vector<int>& curves,
              double radius, bool removeVolume, std::vector<CadEntity>& out,
              QString& error);
  bool chamfer(const std::vector<int>& volumes,
               const std::vector<int>& curves,
               const std::vector<int>& surfaces, double distance,
               bool removeVolume, std::vector<CadEntity>& out, QString& error);
  bool booleanOp(const QString& operation, const std::vector<int>& objects,
                 const std::vector<int>& tools, bool removeObject,
                 bool removeTool, std::vector<CadEntity>& out, QString& error);

  bool translate(int dim, const std::vector<int>& tags,
                 const std::array<double, 3>& delta, QString& error);
  bool rotate(int dim, const std::vector<int>& tags,
              const std::array<double, 3>& axisPoint,
              const std::array<double, 3>& axisDirection, double angleDegrees,
              QString& error);
  bool scale(int dim, const std::vector<int>& tags,
             const std::array<double, 3>& center,
             const std::array<double, 3>& factors, QString& error);
  bool mirror(int dim, const std::vector<int>& tags,
              const std::array<double, 4>& plane, QString& error);
  bool copy(int dim, const std::vector<int>& tags,
            std::vector<CadEntity>& out, QString& error);
  bool remove(int dim, const std::vector<int>& tags, bool recursive,
              QString& error);

  // CAD interchange and inspection.
  bool importShapes(const QString& path, bool highestDimOnly,
                    std::vector<CadEntity>& out, QString& error);
  bool exportModel(const QString& path, QString& error);
  bool listEntities(int dim, QVector<CadEntityInfo>& out, QString& error);
  bool entityCount(int minDim, int& count, QString& error);
  bool hasSurfaceGeometry(bool& hasGeometry, QString& error);

  // Regenerate the surface mesh and write it as STL for the pipeline stage.
  bool meshToStl(const QString& path, QString& error);
  // Regenerate the surface mesh directly as a CGAL mesh (orientation is
  // repaired with CGAL's polygon-soup helpers). The pipeline uses this path:
  // Gmsh's STL output can be locally inconsistently oriented, which CGAL's
  // STL reader rejects.
  bool meshToMesh(Mesh& mesh, QString& error);

private:
  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

} // namespace meshbench
