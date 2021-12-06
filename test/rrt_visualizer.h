#include <QtGui/QBrush>
#include <QtGui/QPen>
#include <QtWidgets/QWidget>

#include "planner.h"

class RrtVisualizer : public QWidget {
  Q_OBJECT
 public:
  RrtVisualizer(rrt::PlannerInterface* planner, QWidget* parent = nullptr);

  void TimerCallback();

  void set_planner(rrt::PlannerInterface* value) { planner_ = value; }

  void SetInit(int x, int y) {
    init_x_ = x;
    init_y_ = y;
  }

  void SetGoal(int x, int y) {
    goal_x_ = x;
    goal_y_ = y;
  }

 protected:
  void paintEvent(QPaintEvent* event) override;

 private:
  QFont text_font_;
  QPen text_pen_;
  QPen edge_pen_;
  QPen solution_pen_;
  QPen smooth_solution_pen_;
  QPen node_pen_;
  QBrush node_brush_;
  QPen init_pen_;
  QBrush init_brush_;
  QPen goal_pen_;
  QBrush goal_brush_;
  QPen collision_pen_;
  QBrush collision_brush_;
  QBrush background_brush_;
  rrt::PlannerInterface* planner_;
  int init_x_{0};
  int init_y_{0};
  int goal_x_{0};
  int goal_y_{0};
};
