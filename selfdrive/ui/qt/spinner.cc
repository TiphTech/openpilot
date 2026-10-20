#include "selfdrive/ui/qt/spinner.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>

#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <QApplication>
#include <QCoreApplication>
#include <QFont>
#include <QFontDatabase>
#include <QGridLayout>
#include <QLinearGradient>
#include <QPainter>
#include <QPainterPath>
#include <QString>
#include <QStringList>
#include <QTimer>
#include <QVBoxLayout>

#include "system/hardware/hw.h"
#include "selfdrive/ui/qt/qt_window.h"
#include "selfdrive/ui/qt/util.h"

static QString getLocalIPv4() {
  int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) return QString();

  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 300000;
  ::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  struct sockaddr_in target;
  std::memset(&target, 0, sizeof(target));
  target.sin_family = AF_INET;
  target.sin_port = htons(80);
  ::inet_pton(AF_INET, "8.8.8.8", &target.sin_addr);

  if (::connect(sock, (struct sockaddr*)&target, sizeof(target)) < 0) {
    ::close(sock);
    return QString();
  }

  struct sockaddr_in local;
  socklen_t local_len = sizeof(local);
  if (::getsockname(sock, (struct sockaddr*)&local, &local_len) < 0) {
    ::close(sock);
    return QString();
  }

  char ip_str[INET_ADDRSTRLEN];
  ::inet_ntop(AF_INET, &local.sin_addr, ip_str, INET_ADDRSTRLEN);
  ::close(sock);
  return QString(ip_str);
}

static QString recoveryLabel() {
  QString ip = getLocalIPv4();
  if (ip.isEmpty()) return QStringLiteral("no network");
  return ip + QStringLiteral(":6999");
}

static QString spinnerFontFamily() {
  QString font_path = QCoreApplication::applicationDirPath() + "/../assets/fonts/Pretendard-SemiBold.ttf";
  int font_id = QFontDatabase::addApplicationFont(font_path);
  if (font_id < 0) return QStringLiteral("Inter");

  QStringList families = QFontDatabase::applicationFontFamilies(font_id);
  return families.isEmpty() ? QStringLiteral("Inter") : families.first();
}

KiaLoadingWidget::KiaLoadingWidget(QWidget *parent) : QWidget(parent) {
  setFixedSize(loading_image_size);
  kia_image = loadPixmap("../assets/kia_logo_white.png", QSize(680, 250));
  animation_clock.start();
  animation_timer.setInterval(33);
  connect(&animation_timer, &QTimer::timeout, this, [this]() { update(); });
  animation_timer.start();
}

void KiaLoadingWidget::paintEvent(QPaintEvent *event) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::SmoothPixmapTransform);
  painter.fillRect(rect(), Qt::black);

  const QRect logo_rect((width() - kia_image.width()) / 2,
                        (height() - kia_image.height()) / 2,
                        kia_image.width(), kia_image.height());
  QPixmap animated_logo(kia_image.size());
  animated_logo.fill(Qt::transparent);
  QPainter logo_painter(&animated_logo);
  logo_painter.setRenderHint(QPainter::SmoothPixmapTransform);
  logo_painter.drawPixmap(0, 0, kia_image);

  const double phase = std::fmod(animation_clock.elapsed() / 3600.0, 1.0);
  const double sweep_x = phase * (kia_image.width() * 1.4) - kia_image.width() * 0.2;
  QLinearGradient sheen(sweep_x - 80, 0, sweep_x + 80, 0);
  sheen.setColorAt(0.0, QColor(255, 255, 255, 0));
  sheen.setColorAt(0.5, QColor(190, 235, 255, 70));
  sheen.setColorAt(1.0, QColor(255, 255, 255, 0));
  logo_painter.setCompositionMode(QPainter::CompositionMode_SourceAtop);
  logo_painter.fillRect(animated_logo.rect(), sheen);
  logo_painter.end();
  painter.drawPixmap(logo_rect, animated_logo);
}

// Spinner

Spinner::Spinner(QWidget *parent) : QWidget(parent) {
  QGridLayout *main_layout = new QGridLayout(this);
  main_layout->setSpacing(0);
  main_layout->setContentsMargins(200, 170, 200, 70);

  QString font_family = spinnerFontFamily();

  ipLabel = new QLabel(recoveryLabel());
  ipLabel->setObjectName("ipLabel");
  ipLabel->setFont(QFont(font_family, 84));
  ipLabel->setAlignment(Qt::AlignCenter);
  main_layout->addWidget(ipLabel, 0, 0, Qt::AlignHCenter | Qt::AlignTop);

  main_layout->addWidget(new KiaLoadingWidget(this), 1, 0, Qt::AlignHCenter | Qt::AlignVCenter);

  text = new QLabel();
  text->setObjectName("statusLabel");
  text->setFixedWidth(1000);
  text->setFixedHeight(68);
  text->setFont(QFont(font_family, 52));
  text->setWordWrap(false);
  text->setVisible(false);
  text->setAlignment(Qt::AlignCenter);

  progress_bar = new QProgressBar();
  progress_bar->setRange(0, 0);
  progress_bar->setTextVisible(false);
  progress_bar->setVisible(true);
  progress_bar->setFixedWidth(1000);
  progress_bar->setFixedHeight(20);

  QVBoxLayout *progress_layout = new QVBoxLayout();
  progress_layout->setContentsMargins(0, 0, 0, 0);
  progress_layout->setSpacing(6);
  progress_layout->addWidget(text, 0, Qt::AlignHCenter);
  progress_layout->addWidget(progress_bar, 0, Qt::AlignHCenter);
  main_layout->addLayout(progress_layout, 2, 0, Qt::AlignHCenter);

  main_layout->setRowStretch(0, 0);
  main_layout->setRowStretch(1, 1);
  main_layout->setRowStretch(2, 0);

  QTimer *ipTimer = new QTimer(this);
  QObject::connect(ipTimer, &QTimer::timeout, this, &Spinner::refreshIPLabel);
  ipTimer->start(5000);

  setStyleSheet(QString(R"(
    Spinner {
      background-color: black;
    }
    QLabel {
      color: white;
      font-size: 80px;
      background-color: transparent;
    }
    QLabel#ipLabel {
      color: white;
      font-size: 84px;
      font-family: "%1";
    }
    QLabel#statusLabel {
      color: #cfcfcf;
      font-size: 52px;
      font-family: "%1";
    }
    QProgressBar {
      background-color: #373737;
      width: 1000px;
      border solid white;
      border-radius: 10px;
    }
    QProgressBar::chunk {
      border-radius: 10px;
      background-color: white;
    }
  )").arg(font_family));

  notifier = new QSocketNotifier(fileno(stdin), QSocketNotifier::Read);
  QObject::connect(notifier, &QSocketNotifier::activated, this, &Spinner::update);
}

void Spinner::refreshIPLabel() {
  if (ipLabel) ipLabel->setText(recoveryLabel());
}

void Spinner::update(int n) {
  std::string line;
  std::getline(std::cin, line);

  if (line.length()) {
    bool number = std::all_of(line.begin(), line.end(), ::isdigit);
    if (number) {
      progress_bar->setRange(5, 100);
      progress_bar->setVisible(true);
      progress_bar->setValue(std::stoi(line));
    } else {
      QString status = QString::fromStdString(line);
      text->setText(text->fontMetrics().elidedText(status, Qt::ElideRight, text->width()));
      text->setVisible(true);
    }
  }
}

int main(int argc, char *argv[]) {
  initApp(argc, argv);
  QApplication a(argc, argv);
  Spinner spinner;
  setMainWindow(&spinner);
  return a.exec();
}
