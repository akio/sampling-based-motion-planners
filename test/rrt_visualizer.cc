#include "rrt_visualizer.h"

#include <iostream>

#include "QtGui/QPainter"
#include "space_2d.h"

RrtVisualizer::RrtVisualizer(rrt::PlannerInterface* planner, QWidget* parent)
    : QWidget(parent), planner_(planner) {
  setFixedSize(1000, 1000);

  QPalette pal = palette();
  // set black background
  pal.setColor(QPalette::Background, Qt::white);
  setAutoFillBackground(true);
  setPalette(pal);

  text_font_.setPixelSize(50);
  text_pen_ = QPen(Qt::black);
  node_pen_ = QPen(Qt::gray);
  node_brush_ = QBrush(Qt::gray);
  edge_pen_ = QPen(Qt::gray);
  solution_pen_ = QPen(Qt::green);
  solution_pen_.setWidth(3);
  smooth_solution_pen_ = QPen(Qt::magenta);
  smooth_solution_pen_.setWidth(3);
  init_pen_ = QPen(Qt::red);
  init_brush_ = QBrush(Qt::red);
  goal_pen_ = QPen(Qt::blue);
  goal_brush_ = QBrush(Qt::blue);
  collision_pen_ = QPen(Qt::black);
  collision_brush_ = QBrush(Qt::gray);
  background_brush_ = QBrush(Qt::white);
}

void RrtVisualizer::TimerCallback() { update(); }

void RrtVisualizer::paintEvent(QPaintEvent* event) {
  QPainter painter;
  painter.begin(this);
  painter.setRenderHint(QPainter::Antialiasing);

  painter.setPen(node_pen_);
  painter.setBrush(node_brush_);
  painter.setBackground(background_brush_);

  // Draw collision objects
  for (const auto& collision : planner_->space()->collisions()) {
    auto p = std::dynamic_pointer_cast<rrt::Box2D>(collision);
    painter.save();

    painter.setPen(collision_pen_);
    painter.setBrush(collision_brush_);
    painter.drawRect(p->x0, 1000 - p->y0 - p->height, p->width, p->height);

    painter.restore();
  }

  // Draw graph
  for (const auto& node : planner_->nodes()) {
    auto m = std::dynamic_pointer_cast<rrt::Motion2D>(node);
    painter.setPen(node_pen_);
    painter.setBrush(node_brush_);
    painter.drawEllipse(m->x - 1, 1000 - m->y - 1, 3, 3);
    auto parent = m->parent();
    if (parent) {
      auto p = std::dynamic_pointer_cast<rrt::Motion2D>(parent);
      painter.setPen(edge_pen_);
      painter.drawLine(m->x, 1000 - m->y, p->x, 1000 - p->y);
    }
  }

  // Draw solution
  painter.setPen(solution_pen_);
  for (const auto& node : planner_->solution()) {
    auto m = std::dynamic_pointer_cast<rrt::Motion2D>(node);
    painter.drawEllipse(m->x - 1, 1000 - m->y - 1, 3, 3);
    auto parent = m->parent();
    if (parent) {
      auto p = std::dynamic_pointer_cast<rrt::Motion2D>(parent);
      painter.drawLine(m->x, 1000 - m->y, p->x, 1000 - p->y);
    }
  }

  // Execute path pruning
  auto path = planner_->solution();
  auto new_path = rrt::path_pruning(planner_->space(), path);
  painter.setPen(smooth_solution_pen_);
  for (const auto& node : new_path) {
    auto m = std::dynamic_pointer_cast<rrt::Motion2D>(node);
    painter.drawEllipse(m->x - 1, 1000 - m->y - 1, 3, 3);
    auto parent = m->parent();
    if (parent) {
      auto p = std::dynamic_pointer_cast<rrt::Motion2D>(parent);
      painter.drawLine(m->x, 1000 - m->y, p->x, 1000 - p->y);
    }
  }

  painter.setPen(init_pen_);
  painter.setBrush(init_brush_);
  painter.drawEllipse(init_x_ - 4, 1000 - init_y_ - 4, 9, 9);

  painter.setPen(goal_pen_);
  painter.setBrush(goal_brush_);
  painter.drawEllipse(goal_x_ - 4, 1000 - goal_y_ - 4, 9, 9);

  painter.end();
}
