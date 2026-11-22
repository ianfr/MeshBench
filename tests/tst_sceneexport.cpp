// Unit tests for scene image export (PNG encoding path).
//
// The render-window path needs a live OpenGL context, but the PNG encoder
// is shared with imageDataPng(), which can be exercised with a tiny
// in-memory vtkImageData on a headless machine.

#include "SceneExport.h"

#include <QtTest>

#include <QFile>
#include <QTemporaryDir>

#include <vtkActor.h>
#include <vtkImageData.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkType.h>

using namespace meshbench;

class TstSceneExport : public QObject {
  Q_OBJECT
private slots:
  void nullImageFailsCleanly() {
    QByteArray png("stale");
    QString error;
    QVERIFY(!imageDataPng(nullptr, png, error));
    QVERIFY(!error.isEmpty());
    QVERIFY(png.isEmpty());
  }

  void encodesImageDataAsPng() {
    auto image = vtkSmartPointer<vtkImageData>::New();
    image->SetDimensions(4, 3, 1);
    image->AllocateScalars(VTK_UNSIGNED_CHAR, 3);
    auto* pixels = static_cast<unsigned char*>(image->GetScalarPointer());
    const int count = 4 * 3 * 3;
    for (int i = 0; i < count; ++i)
      pixels[i] = static_cast<unsigned char>((i * 37) % 256);

    QByteArray png;
    QString error;
    QVERIFY2(imageDataPng(image, png, error), qPrintable(error));
    QVERIFY(error.isEmpty());
    QVERIFY(png.size() > 8);
    QCOMPARE(static_cast<unsigned char>(png[0]), 0x89u);
    QCOMPARE(png.mid(1, 3), QByteArray("PNG"));
  }

  void emptyImageFails() {
    auto image = vtkSmartPointer<vtkImageData>::New();
    // No dimensions/scalars: the writer must fail instead of producing junk.
    QByteArray png;
    QString error;
    QVERIFY(!imageDataPng(image, png, error));
    QVERIFY(!error.isEmpty());
    QVERIFY(png.isEmpty());
  }

  void nullRenderWindowFailsCleanly() {
    QByteArray png("stale");
    QString error;
    QVERIFY(!renderWindowPng(nullptr, 0, 0, png, error));
    QVERIFY(!error.isEmpty());
    QVERIFY(png.isEmpty());

    QString writeError;
    QVERIFY(!writeRenderWindowPng(nullptr, QStringLiteral("/tmp/x.png"), 0, 0,
                                  writeError));
    QVERIFY(!writeError.isEmpty());
  }

  void rendersOffscreenWindowToPng() {
    auto window = vtkSmartPointer<vtkRenderWindow>::New();
    window->SetOffScreenRendering(1);
    window->SetSize(160, 120);
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->SetBackground(0.1, 0.2, 0.3);
    window->AddRenderer(renderer);
    auto source = vtkSmartPointer<vtkSphereSource>::New();
    source->SetThetaResolution(12);
    source->SetPhiResolution(12);
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(source->GetOutputPort());
    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    renderer->AddActor(actor);

    QByteArray png;
    QString error;
    if (!renderWindowPng(window, 160, 120, png, error))
      QSKIP(qPrintable(error)); // some headless machines lack any GL context
    QVERIFY(png.size() > 8);
    QCOMPARE(png.mid(1, 3), QByteArray("PNG"));
    // Size is restored after the export.
    QCOMPARE(window->GetSize()[0], 160);
    QCOMPARE(window->GetSize()[1], 120);

    // File path: same renderer, explicit output.
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString path = dir.filePath(QStringLiteral("scene.png"));
    QString writeError;
    QVERIFY2(writeRenderWindowPng(window, path, 80, 60, writeError),
             qPrintable(writeError));
    QFile file(path);
    QVERIFY(file.open(QIODevice::ReadOnly));
    const QByteArray written = file.readAll();
    QVERIFY(written.size() > 8);
    QCOMPARE(written.mid(1, 3), QByteArray("PNG"));
  }
};

QTEST_GUILESS_MAIN(TstSceneExport)
#include "tst_sceneexport.moc"
