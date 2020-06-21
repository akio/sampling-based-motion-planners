#include "test_window.h"

#include <iostream>
#include <map>

#include "QtCore/QTimer"
#include "QtWidgets/QGridLayout"
#include "QtWidgets/QHBoxLayout"
#include "QtWidgets/QPushButton"
#include "QtWidgets/QComboBox"
#include "QtWidgets/QRadioButton"

#include "space_2d.h"
#include "rrt_planner.h"
#include "birrt_planner.h"
#include "rrt_star_planner.h"
#include "birrt_star_planner.h"


MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
  auto space = std::make_shared<rrt::Space2D>(0, 1000, 0, 1000, 10.0);
  space->add_collision_box(400, 100, 200, 800);
  //space->add_collision_box(400, 950, 200, 50);

  auto rrt = std::make_shared<rrt::RrtPlanner>(space);
  rrt->set_goal_tolerance(5.0);
  rrt->set_use_connect(false);

  auto birrt = std::make_shared<rrt::BidirectionalRrtPlanner>(space);
  birrt->set_goal_tolerance(5.0);

  auto rrt_star = std::make_shared<rrt::RrtStarPlanner>(space);
  rrt_star->set_goal_tolerance(5.0);
  rrt_star->set_gamma(300.0);
  //rrt_star->set_max_samples(20);

  auto birrt_star = std::make_shared<rrt::BiRrtStarPlanner>(space);
  birrt_star->set_goal_tolerance(5.0);
  birrt_star->set_gamma(50.0);

  current_planner_ = rrt;

  planners_["RRT"] = rrt;
  planners_["BiRRT"] = birrt;
  planners_["RRT*"] = rrt_star;
  planners_["BiRRT*"] = birrt_star;

  setWindowTitle("RRT Visualizer");
  setFixedSize(1000, 1100);
  QVBoxLayout* layout = new QVBoxLayout();
  QPushButton* button = new QPushButton("Compute");
  layout->addWidget(button);
  QComboBox* combo_box = new QComboBox();
  for (const auto& pair : planners_) {
    combo_box->addItem(pair.first.c_str());
  }
  combo_box->setCurrentText("RRT");
  layout->addWidget(combo_box);

  connect(combo_box, &QComboBox::currentTextChanged, this, &MainWindow::HandleAlgorithmChanged);

  visualizer_ = new RrtVisualizer(rrt.get());
  layout->addWidget(visualizer_);

  QWidget* container = new QWidget();
  container->setLayout(layout);
  setCentralWidget(container);

  connect(button, &QPushButton::clicked, this, &MainWindow::HandleButtonClicked);

  auto init = std::make_shared<rrt::Motion2D>();
  init->x = 100;
  init->y = 500;

  auto goal = std::make_shared<rrt::Motion2D>();
  goal->x = 900;
  goal->y = 500;

  visualizer_->SetInit(init->x, init->y);
  visualizer_->SetGoal(goal->x, goal->y);

  QTimer* timer = new QTimer(this);
  connect(timer, &QTimer::timeout, visualizer_, &RrtVisualizer::TimerCallback);
  timer->start(50);
}

MainWindow::~MainWindow() {}

void MainWindow::HandleButtonClicked() {
  current_planner_->reset();

  auto init = std::make_shared<rrt::Motion2D>();
  init->x = 100;
  init->y = 500;

  auto goal = std::make_shared<rrt::Motion2D>();
  goal->x = 900;
  goal->y = 500;

  visualizer_->SetInit(init->x, init->y);
  visualizer_->SetGoal(goal->x, goal->y);

  if (current_planner_->solve(init, goal)) {
    std::cout << "success: " << current_planner_->nodes().size() << std::endl;
  } else {
    std::cout << "failure: " << current_planner_->nodes().size() << std::endl;
  }
}

void MainWindow::HandleAlgorithmChanged(const QString& text) {
  auto algorithm = text.toUtf8().constData();
  std::cout << "algorithm: " << algorithm << std::endl;
  current_planner_ = planners_[algorithm];
  visualizer_->set_planner(current_planner_.get());
}
