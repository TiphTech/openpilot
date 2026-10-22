#pragma once

#include <algorithm>
#include <cmath>

#include <QConicalGradient>
#include <QImage>
#include <QPainter>
#include <QPainterPath>

struct OnroadHaloState {
  bool lateral = false;
  bool cruise = false;
  bool braking = false;
  bool left_blinker = false;
  bool right_blinker = false;
};

constexpr int ONROAD_HALO_FRAME_MARGIN = 12;

inline void paint_onroad_halo(QPainter &painter, const QRectF &bounds,
                              const OnroadHaloState &state, double seconds) {
  if (bounds.width() <= 0 || bounds.height() <= 0) return;

  const QColor green(0, 205, 65), turquoise(0, 205, 215), blue(25, 135, 245);
  const QColor base = state.lateral && state.cruise ? green :
                      state.lateral ? turquoise : state.cruise ? blue : QColor(0, 0, 0, 0);
  const bool blink_on = std::fmod(seconds, 0.8) < 0.4;
  const QColor top = state.braking ? QColor(255, 35, 30) : base;
  const QColor left = state.left_blinker && blink_on ? QColor(255, 165, 0) : base;
  const QColor right = state.right_blinker && blink_on ? QColor(255, 165, 0) : base;
  if (!base.alpha() && !top.alpha() && !left.alpha() && !right.alpha()) return;

  // Color transitions stay at the corners, independent of the viewport aspect ratio.
  constexpr double pi = 3.14159265358979323846;
  const double corner = std::atan2(bounds.height(), bounds.width()) / (2 * pi);
  const double blend = std::min(0.025, corner * 0.4);
  QConicalGradient colors(bounds.center(), 0);
  colors.setColorAt(0, right);
  colors.setColorAt(corner - blend, right);
  colors.setColorAt(corner + blend, top);
  colors.setColorAt(0.5 - corner - blend, top);
  colors.setColorAt(0.5 - corner + blend, left);
  colors.setColorAt(0.5 + corner - blend, left);
  colors.setColorAt(0.5 + corner + blend, base);
  colors.setColorAt(1 - corner - blend, base);
  colors.setColorAt(1 - corner + blend, right);
  colors.setColorAt(1, right);

  // Parent and camera share two bounded blink-phase caches, not per-frame image work.
  struct Cache {
    QSize size;
    QColor base, top, left, right;
    QImage opaque, fog;
  };
  static Cache caches[2];
  Cache &cache = caches[(state.left_blinker || state.right_blinker) && blink_on ? 1 : 0];
  const QSize size = bounds.size().toSize();
  if (cache.size != size || cache.base != base || cache.top != top || cache.left != left || cache.right != right) {
    cache.size = size;
    cache.base = base;
    cache.top = top;
    cache.left = left;
    cache.right = right;
    // The fog is intentionally soft: half-resolution textures bound RAM and rebuild cost.
    const QSize texture_size((size.width() + 1) / 2, (size.height() + 1) / 2);
    const QRectF frame(QPointF(0, 0), texture_size);
    colors.setCenter(frame.center());
    const double scale = std::min(texture_size.height(), texture_size.width()) / 1080.0;
    const double opaque = std::ceil(12 * scale), total = std::ceil(60 * scale), radius = std::ceil(28 * scale);
    QPainterPath outside, inside;
    outside.addRoundedRect(frame, radius, radius);
    inside.addRoundedRect(frame.adjusted(opaque, opaque, -opaque, -opaque), radius - opaque, radius - opaque);
    QPainterPath solid = outside.subtracted(inside);
    QPainterPath fog_inside;
    // Overlap by one texture pixel so bilinear sampling cannot leave a dark seam.
    const double fog_inset = std::max(0.0, opaque - 1);
    fog_inside.addRoundedRect(frame.adjusted(fog_inset, fog_inset, -fog_inset, -fog_inset),
                              radius - fog_inset, radius - fog_inset);

    cache.opaque = QImage(texture_size, QImage::Format_ARGB32_Premultiplied);
    cache.opaque.fill(Qt::transparent);
    {
      QPainter p(&cache.opaque);
      p.setRenderHint(QPainter::Antialiasing);
      p.fillPath(solid, colors);
    }
    QImage mask(texture_size, QImage::Format_ARGB32_Premultiplied);
    mask.fill(Qt::transparent);
    {
      QPainter p(&mask);
      p.setRenderHint(QPainter::Antialiasing);
      p.setClipPath(fog_inside);
      const auto band = [&](const QRectF &area, QPointF from, QPointF to) {
        QLinearGradient fade(from, to);
        fade.setColorAt(0, QColor(255, 255, 255, 255));
        const double start = opaque / total;
        fade.setColorAt(start, QColor(255, 255, 255, 255));
        fade.setColorAt(start + (1 - start) * 0.25, QColor(255, 255, 255, 180));
        fade.setColorAt(start + (1 - start) * 0.5, QColor(255, 255, 255, 90));
        fade.setColorAt(start + (1 - start) * 0.75, QColor(255, 255, 255, 20));
        fade.setColorAt(1, QColor(255, 255, 255, 0));
        p.fillRect(area, fade);
      };
      band(QRectF(0, 0, texture_size.width(), total), QPointF(0, 0), QPointF(0, total));
      band(QRectF(0, texture_size.height() - total, texture_size.width(), total), QPointF(0, texture_size.height()), QPointF(0, texture_size.height() - total));
      band(QRectF(0, 0, total, texture_size.height()), QPointF(0, 0), QPointF(total, 0));
      band(QRectF(texture_size.width() - total, 0, total, texture_size.height()), QPointF(texture_size.width(), 0), QPointF(texture_size.width() - total, 0));
    }
    cache.fog = QImage(texture_size, QImage::Format_ARGB32_Premultiplied);
    cache.fog.fill(Qt::transparent);
    {
      QPainter p(&cache.fog);
      const QRect center = frame.adjusted(total, total, -total, -total).toAlignedRect();
      p.setClipRegion(QRegion(frame.toRect()).subtracted(QRegion(center)));
      p.fillRect(frame, colors);
      p.setClipping(false);
      p.setCompositionMode(QPainter::CompositionMode_DestinationIn);
      p.drawImage(0, 0, mask);
    }
  }
  painter.save();
  painter.setRenderHint(QPainter::SmoothPixmapTransform);
  painter.drawImage(bounds, cache.opaque);
  painter.setOpacity(0.97 + 0.03 * std::sin(seconds * 2 * pi / 7.0));
  painter.drawImage(bounds, cache.fog);
  painter.restore();
}
