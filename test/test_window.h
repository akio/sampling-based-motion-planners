#include <memory>

#include <QtWidgets/QWidget>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QApplication>

#include "rrt.h"

class MainWindow : public QMainWindow {
  Q_OBJECT
 public:
  explicit MainWindow(QWidget* parent=nullptr);

  virtual ~MainWindow();

  void HandleButtonClicked();

 private:
  std::unique_ptr<rrt::Rrt> rrt_;
};

