#pragma once

// Scene image export: render a VTK render window to PNG. Used by the
// File > Export scene image menu item and by the MCP scene_get_image tool.
//
// The render-window helper is deliberately kept free of Qt and of
// VtkView so the PNG encoding path can be unit-tested without an OpenGL
// context (see imageDataPng + tests/tst_sceneexport.cpp).

#include <QByteArray>
#include <QString>

class vtkImageData;
class vtkRenderWindow;

namespace meshbench {

// Encode an RGB/RGBA vtkImageData as PNG. Returns false with an error for a
// null image, an empty extent, or a writer failure. The output starts with
// the PNG signature on success.
bool imageDataPng(vtkImageData* image, QByteArray& png, QString& error);

// Render `window` and encode the result as PNG. width/height <= 0 keep the
// window's current size. The window size is restored before returning.
bool renderWindowPng(vtkRenderWindow* window, int width, int height,
                     QByteArray& png, QString& error);

// Render + write to disk. Creates parent directories? No: the path's parent
// must already exist (matching normal file-save behavior).
bool writeRenderWindowPng(vtkRenderWindow* window, const QString& path,
                          int width, int height, QString& error);

} // namespace meshbench
