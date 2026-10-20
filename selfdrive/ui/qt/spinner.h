#include <QElapsedTimer>
#include <QLabel>
#include <QPixmap>
#include <QProgressBar>
#include <QSocketNotifier>
#include <QTimer>
#include <QWidget>

constexpr QSize loading_image_size = QSize(1000, 563);

class KiaLoadingWidget : public QWidget  {
  Q_OBJECT
public:
  KiaLoadingWidget(QWidget *parent = nullptr);

private:
  void paintEvent(QPaintEvent *event) override;
  QPixmap kia_image;
  QElapsedTimer animation_clock;
  QTimer animation_timer;
};

class Spinner : public QWidget {
  Q_OBJECT

public:
  explicit Spinner(QWidget *parent = 0);

private:
  QLabel *text;
  QLabel *ipLabel;
  QProgressBar *progress_bar;
  QSocketNotifier *notifier;

private slots:
  void refreshIPLabel();

public slots:
  void update(int n);
};
