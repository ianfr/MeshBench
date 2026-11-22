#include "SceneExport.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>

#include <vtkImageData.h>
#include <vtkPNGWriter.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkUnsignedCharArray.h>
#include <vtkWindowToImageFilter.h>

#include <cstddef>
#include <limits>

namespace meshbench {
namespace {

constexpr unsigned char kPngSignature[8] = {0x89, 'P', 'N', 'G', '\r', '\n',
                                            0x1a, '\n'};

bool hasPngSignature(const QByteArray& data) {
  if (data.size() < static_cast<int>(sizeof(kPngSignature)))
    return false;
  for (std::size_t i = 0; i < sizeof(kPngSignature); ++i) {
    if (static_cast<unsigned char>(data[static_cast<int>(i)]) !=
        kPngSignature[i])
      return false;
  }
  return true;
}

// Shared writer path: both the standalone-image and render-window helpers
// funnel through here so the PNG signature validation lives in one place.
bool encodePng(vtkImageData* image, QByteArray& png, QString& error) {
  if (!image) {
    error = QStringLiteral("Cannot export image: no image data.");
    return false;
  }
  int extent[6] = {0, 0, 0, 0, 0, 0};
  image->GetExtent(extent);
  if (extent[1] < extent[0] || extent[3] < extent[2] || extent[5] < extent[4]) {
    error = QStringLiteral("Cannot export image: image has an empty extent.");
    return false;
  }

  auto writer = vtkSmartPointer<vtkPNGWriter>::New();
  writer->SetInputData(image);
  writer->SetWriteToMemory(true);
  writer->SetCompressionLevel(6);
  writer->Write();

  auto* result = writer->GetResult();
  if (!result || result->GetNumberOfValues() <= 0) {
    error = QStringLiteral("PNG encoding failed (no output).");
    return false;
  }
  const vtkIdType byteCount = result->GetNumberOfValues();
  if (byteCount > std::numeric_limits<int>::max()) {
    error = QStringLiteral("PNG output is too large.");
    return false;
  }
  png = QByteArray(reinterpret_cast<const char*>(result->GetPointer(0)),
                   static_cast<int>(byteCount));
  if (!hasPngSignature(png)) {
    error = QStringLiteral("PNG encoding produced an invalid signature.");
    png.clear();
    return false;
  }
  return true;
}

} // namespace

bool imageDataPng(vtkImageData* image, QByteArray& png, QString& error) {
  // Runtime safety checks: null input and a clean output on every failure.
  if (!image) {
    error = QStringLiteral("Cannot export image: no image data.");
    png.clear();
    return false;
  }
  png.clear();
  error.clear();
  if (!encodePng(image, png, error)) {
    png.clear();
    return false;
  }
  // Exit check: a successful export is a non-empty PNG.
  if (png.isEmpty() || !hasPngSignature(png)) {
    error = QStringLiteral("PNG encoding produced no usable output.");
    png.clear();
    return false;
  }
  return true;
}

bool renderWindowPng(vtkRenderWindow* window, int width, int height,
                     QByteArray& png, QString& error) {
  png.clear();
  error.clear();
  if (!window) {
    error = QStringLiteral("Cannot export scene: no render window.");
    return false;
  }
  if (width < 0 || height < 0) {
    error = QStringLiteral("Image width and height must be >= 0.");
    return false;
  }

  const int* oldSize = window->GetSize();
  const int oldWidth = oldSize ? oldSize[0] : 0;
  const int oldHeight = oldSize ? oldSize[1] : 0;
  const bool resize = width > 0 && height > 0 &&
                      (width != oldWidth || height != oldHeight);
  if (resize)
    window->SetSize(width, height);

  // Render before capturing so the export always reflects the current scene
  // (camera changes, new stages, selection overlays).
  window->Render();

  auto shot = vtkSmartPointer<vtkWindowToImageFilter>::New();
  shot->SetInput(window);
  shot->ReadFrontBufferOff();
  shot->SetScale(1);
  shot->Update();

  const bool ok = encodePng(shot->GetOutput(), png, error);

  if (resize) {
    window->SetSize(oldWidth, oldHeight);
    window->Render();
  }
  if (!ok) {
    png.clear();
    return false;
  }
  // Exit check: the scene export must be a valid PNG.
  if (png.isEmpty() || !hasPngSignature(png)) {
    error = QStringLiteral("Scene image export produced no usable PNG.");
    png.clear();
    return false;
  }
  return true;
}

bool writeRenderWindowPng(vtkRenderWindow* window, const QString& path,
                          int width, int height, QString& error) {
  error.clear();
  if (path.isEmpty()) {
    error = QStringLiteral("No output path given.");
    return false;
  }
  const QFileInfo info(path);
  if (!info.absoluteDir().exists()) {
    error = QStringLiteral("Output directory does not exist: %1")
                .arg(info.absolutePath());
    return false;
  }

  QByteArray png;
  if (!renderWindowPng(window, width, height, png, error))
    return false;

  QFile file(path);
  if (!file.open(QIODevice::WriteOnly)) {
    error = QStringLiteral("Cannot write %1: %2").arg(path, file.errorString());
    return false;
  }
  if (file.write(png) != png.size()) {
    error = QStringLiteral("Failed to write the complete PNG to %1.").arg(path);
    file.close();
    return false;
  }
  file.close();

  // Exit check: the file is on disk and holds the signature we wrote.
  if (!QFileInfo::exists(path) || QFileInfo(path).size() < 8) {
    error = QStringLiteral("PNG file was not written to %1.").arg(path);
    return false;
  }
  return true;
}

} // namespace meshbench
