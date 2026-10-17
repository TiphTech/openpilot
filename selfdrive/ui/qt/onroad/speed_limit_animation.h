#pragma once

#include <optional>

#include <QRegularExpression>
#include <QString>

class RoadLimitSpeedAnimation {
public:
  std::optional<int> update(int displayed_limit_kph, const QString &cruise_log,
                            bool automatic, bool engaged, bool camera_zone,
                            bool healthy, qint64 now_ms) {
    const int previous = last_limit;
    last_limit = healthy ? displayed_limit_kph : 0;
    if (!healthy || !automatic || !engaged || camera_zone || displayed_limit_kph <= 0) {
      pending_limit = 0;
      return std::nullopt;
    }
    if (displayed_limit_kph != previous) {
      pending_previous = previous;
      pending_limit = previous > 0 ? displayed_limit_kph : 0;
      pending_since_ms = now_ms;
    }
    if (!pending_limit) return std::nullopt;
    if (now_ms < pending_since_ms || now_ms - pending_since_ms > 3000) {
      pending_limit = 0;
      return std::nullopt;
    }

    // The log confirms automatic application; a real sign transition is also required.
    static const QRegularExpression event("^Auto road limit ([0-9]{1,3})->([0-9]{1,3}) => ([0-9]{1,3})->([0-9]{1,3})$");
    const auto match = event.match(cruise_log);
    if (!match.hasMatch()) return std::nullopt;
    const int old_limit = match.captured(1).toInt();
    const int new_limit = match.captured(2).toInt();
    const int applied_limit = match.captured(3).toInt();
    const int target = match.captured(4).toInt();
    if (old_limit != pending_previous || new_limit != pending_limit || applied_limit != pending_limit ||
        new_limit < 30 || old_limit == new_limit || target < 1 || target > 255) return std::nullopt;
    pending_limit = 0;
    return target;
  }

private:
  int last_limit = 0;
  int pending_previous = 0;
  int pending_limit = 0;
  qint64 pending_since_ms = 0;
};
