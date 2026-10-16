// Standalone pixel regression test; compile with Qt5Gui and Qt5Core.
#include <cassert>
#include <iostream>

#include <QDir>
#include <QElapsedTimer>
#include <QGuiApplication>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QOpenGLFramebufferObject>
#include <QOpenGLFunctions>
#include <QOpenGLPaintDevice>

#include "selfdrive/ui/qt/onroad/halo.h"

QImage render(const QSize &size, const OnroadHaloState &state, double seconds = 0) {
  QImage image(size, QImage::Format_ARGB32_Premultiplied);
  image.fill(Qt::transparent);
  QPainter painter(&image);
  paint_onroad_halo(painter, image.rect(), state, seconds);
  return image;
}

int main(int argc, char **argv) {
  const QString output = argc > 1 ? argv[1] : "/tmp/onroad-halo";
  QDir().mkpath(output);
  const OnroadHaloState both{true, true}, lateral{true}, cruise{false, true};
  const OnroadHaloState braking{true, true, true};
  const OnroadHaloState turn{true, true, false, false, true};
  const OnroadHaloState combined{true, true, true, false, true};
  const OnroadHaloState hazard{false, false, false, true, true};
  for (const QSize size : {QSize(2160, 1080), QSize(1206, 597), QSize(1100, 1080)}) {
    const int cx = size.width() / 2, cy = size.height() / 2;
    const auto off = render(size, {});
    assert(off.pixelColor(cx, 5).alpha() == 0);
    const auto green = render(size, both);
    const QColor edge = green.pixelColor(cx, 5);
    assert(edge.alpha() == 255 && edge.green() > 190 && edge.red() == 0);
    assert(green.pixelColor(cx, cy).alpha() == 0);
    const double opaque = std::min(size.width(), size.height()) / 1080.0 * 27;
    for (int y = 2; y < static_cast<int>(opaque) - 1; ++y) assert(green.pixelColor(cx, y).alpha() == 255);
    const int half_fade = static_cast<int>(opaque * 2);
    assert(green.pixelColor(cx, half_fade).alpha() > 90 && green.pixelColor(cx, half_fade).alpha() < 170);
    assert(green.pixelColor(cx, static_cast<int>(opaque * 3 + 8)).alpha() == 0);
    assert(render(size, lateral).pixelColor(cx, 5).blue() > 190);
    assert(render(size, cruise).pixelColor(cx, 5).red() == 25);
    const auto red = render(size, braking);
    assert(red.pixelColor(cx, 5).red() > 240 && red.pixelColor(cx, size.height() - 5).green() > 190);
    assert(render(size, turn).pixelColor(size.width() - 5, cy).red() > 240);
    assert(render(size, turn, 0.5).pixelColor(size.width() - 5, cy).red() == 0);
    assert(render(size, turn).pixelColor(5, cy).red() == 0);
    assert(render(size, hazard).pixelColor(5, cy).red() > 240);
    assert(render(size, hazard).pixelColor(size.width() - 5, cy).red() > 240);
    assert(render(size, combined).pixelColor(cx, 5).red() > 240);
    assert(render(size, combined).pixelColor(size.width() - 5, cy).red() > 240);
    assert(render(size, both, 1.75) != render(size, both, 5.25));

    // Translated Qt rasterization may round coverage by a few alpha levels.
    // The camera inset must still match the parent's geometry, not a second border.
    QImage inset(size - QSize(60, 60), QImage::Format_ARGB32_Premultiplied);
    inset.fill(Qt::transparent);
    QPainter p(&inset);
    paint_onroad_halo(p, QRectF(-30, -30, size.width(), size.height()), combined, 0);
    p.end();
    const auto crop = render(size, combined).copy(30, 30, inset.width(), inset.height());
    int max_delta = 0;
    for (int y = 0; y < inset.height(); ++y) {
      for (int x = 0; x < inset.width(); ++x) {
        const QRgb a = inset.pixel(x, y), b = crop.pixel(x, y);
        max_delta = std::max({max_delta, std::abs(qAlpha(a) - qAlpha(b)), std::abs(qRed(a) - qRed(b)),
                              std::abs(qGreen(a) - qGreen(b)), std::abs(qBlue(a) - qBlue(b))});
      }
    }
    std::cout << "Inset max channel delta: " << max_delta << std::endl;
    if (max_delta > 4) {
      inset.save(output + "/inset.png");
      crop.save(output + "/crop.png");
    }
    assert(max_delta <= 4);
  }

  int index = 0;
  for (const auto state : {both, lateral, cruise, combined, hazard, OnroadHaloState{}}) {
    assert(render(QSize(2160, 1080), state).save(output + QString("/halo-%1.png").arg(index++)));
  }
  QElapsedTimer timer;
  timer.start();
  for (int i = 0; i < 60; ++i) render(QSize(2160, 1080), combined, i / 20.0);
  std::cout << "Halo pixel checks passed; CPU image rendering: " << timer.elapsed() / 60.0 << " ms/frame\n";
  if (argc > 2 && QString(argv[2]) == "--gpu") {
    QSurfaceFormat format;
    format.setRenderableType(QSurfaceFormat::OpenGLES);
    format.setVersion(3, 0);
    QSurfaceFormat::setDefaultFormat(format);
    QGuiApplication app(argc, argv);
    QOpenGLContext context;
    context.setFormat(format);
    if (!context.create()) return 2;
    QOffscreenSurface surface;
    surface.setFormat(context.format());
    surface.create();
    if (!context.makeCurrent(&surface)) return 2;
    std::cout << "Renderer: " << context.functions()->glGetString(GL_RENDERER) << std::endl;
    QOpenGLFramebufferObject frame(2160, 1080, QOpenGLFramebufferObject::CombinedDepthStencil);
    frame.bind();
    QOpenGLPaintDevice device(2160, 1080);
    timer.restart();
    for (int i = 0; i < 60; ++i) {
      QPainter painter(&device);
      painter.fillRect(QRect(0, 0, 2160, 1080), Qt::black);
      paint_onroad_halo(painter, QRect(0, 0, 2160, 1080), combined, i / 20.0);
      painter.end();
      context.functions()->glFinish();
    }
    std::cout << "GPU rendering: " << timer.elapsed() / 60.0 << " ms/frame\n";
    const QImage gpu = frame.toImage();
    assert(gpu.pixelColor(1080, 5).red() > 240);
    assert(gpu.pixelColor(1080, 1075).green() > 190);
    assert(gpu.pixelColor(1080, 540) == QColor(Qt::black));
    assert(gpu.save(output + "/halo-gpu.png"));
  }
}
