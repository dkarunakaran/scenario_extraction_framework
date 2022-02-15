
#include "dataset_panel.h"

#include <QtGui>
#include <QApplication>
#include <QVBoxLayout>
#include <QHBoxLayout>



int main(int argv, char **args)
{
  ros::init(argv, args, "DatasetPlayback");

  QApplication app(argv, args);
  
  DatasetPanel *textEdit = new DatasetPanel;
  
  QVBoxLayout *layout = new QVBoxLayout;
  layout->addWidget(textEdit);
  
  QWidget window;
  window.setLayout(layout);
  
  window.show();
  
  return app.exec();
}




