#include <QtWidgets/QWidget>

#include <QtGui/QPen>
#include <QtGui/QBrush>

#include "rrt.h"

class RrtVisualizer : public QWidget {
  Q_OBJECT
 public:
  RrtVisualizer(rrt::Rrt& rrt, QWidget* parent = nullptr);

  void TimerCallback();

 protected:
  void paintEvent(QPaintEvent* event) override;

 private:
  QFont text_font_;
  QPen text_pen_;
  QPen edge_pen_;
  QPen node_pen_;
  QBrush node_brush_;
  QPen init_pen_;
  QBrush init_brush_;
  QPen goal_pen_;
  QBrush goal_brush_;
  QPen collision_pen_;
  QBrush collision_brush_;
  QBrush background_brush_;
  rrt::Rrt& rrt_;
};
