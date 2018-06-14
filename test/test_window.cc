#include "test_window.h"

#include <iostream>

#include "QtCore/QTimer"
#include "QtWidgets/QGridLayout"
#include "QtWidgets/QHBoxLayout"
#include "QtWidgets/QPushButton"

#include "rrt_visualizer.h"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
  auto space = std::make_shared<rrt::Space2D>(0, 1000, 0, 1000, 10.0);
  space->AddCollisionBox(200, 200, 200, 200);
  rrt_.reset(new rrt::Rrt(space));
  rrt_->set_goal_tolerance(5.0);
  rrt_->set_use_connect(true);

  setWindowTitle("RRT Visualizer");
  setFixedSize(1000, 1100);
  QVBoxLayout* layout = new QVBoxLayout();
  QPushButton* button = new QPushButton("Compute");
  layout->addWidget(button);
  RrtVisualizer* visualizer = new RrtVisualizer(*rrt_);
  layout->addWidget(visualizer);

  QWidget* container = new QWidget();
  container->setLayout(layout);
  setCentralWidget(container);

  connect(button, &QPushButton::clicked, this, &MainWindow::HandleButtonClicked);

  QTimer* timer = new QTimer(this);
  connect(timer, &QTimer::timeout, visualizer, &RrtVisualizer::TimerCallback);
  timer->start(50);
}

MainWindow::~MainWindow() {}

void MainWindow::HandleButtonClicked() {
  rrt_->Clear();

  auto init = std::make_shared<rrt::Motion2D>();
  init->x = 100;
  init->y = 100;

  auto goal = std::make_shared<rrt::Motion2D>();
  goal->x = 900;
  goal->y = 900;

  if (rrt_->Solve(init, goal)) {
    std::cout << "success: " << rrt_->nodes().size() << std::endl;
  } else {
    std::cout << "failure: " << rrt_->nodes().size() << std::endl;
  }
}
