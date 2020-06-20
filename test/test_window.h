#include <memory>

#include <QtWidgets/QWidget>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QApplication>

#include "rrt_visualizer.h"

class MainWindow : public QMainWindow {
  Q_OBJECT
 public:
  explicit MainWindow(QWidget* parent=nullptr);

  virtual ~MainWindow();

  void HandleButtonClicked();

  void HandleAlgorithmChanged(const QString& text);
 private:
  std::map<std::string, std::shared_ptr<rrt::PlannerInterface>> planners_;
  std::shared_ptr<rrt::PlannerInterface> current_planner_;
  RrtVisualizer* visualizer_;
};

