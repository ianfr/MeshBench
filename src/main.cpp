#include "MainWindow.h"

#include <QApplication>

int main(int argc, char** argv) {
  QApplication app(argc, argv);
  app.setApplicationName(QStringLiteral("MeshBench"));
  app.setOrganizationName(QStringLiteral("MeshBench"));
  meshbench::MainWindow window;
  window.show();
  return app.exec();
}
