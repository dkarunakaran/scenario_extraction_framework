#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>


#include "boost/date_time/posix_time/posix_time.hpp"
#include <boost/filesystem.hpp>


#include <QFileInfo>
#include <QThread>

#include <QDateTime>
#include <QSlider>

#include <QLineEdit>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QString>

#include "dataset_panel.h"

#include <dataset_msgs/DatasetEvent.h>

#include "ros/ros.h"
#include <cstdlib>


DatasetPanel::DatasetPanel( QWidget* parent )
    : QFrame( parent ),
      workerThread(NULL)
{
  nh_ = ros::NodeHandle("/");

    file_select_button_ = new QPushButton( "Select file");
    file_select_button_->setFixedHeight(100);

    slider = new QSlider(Qt::Horizontal);
    slider->setRange(0, 999);
    slider->setValue(0);

    start_button_ = new QPushButton( "Play");
    pause_button_ = new QPushButton( "Pause");
    stop_button_ = new QPushButton( "Stop");
    convert_button_ = new QPushButton( "Convert Files");

    playback_realtime_ = new QCheckBox("Playback in real-time");

    //start_button_->setEnabled(false);
    start_button_->setFixedHeight(100);
    pause_button_->setFixedHeight(100);
    stop_button_->setFixedHeight(100);

    QFont font = start_button_->font();
    font.setPointSize(32);
    font.setWeight(QFont::Bold);
    start_button_->setFont(font);
    start_button_->setStyleSheet("");

    pause_button_->setFont(font);
    pause_button_->setStyleSheet("");

    stop_button_->setFont(font);
    stop_button_->setStyleSheet("");

    file_select_button_->setFont(font);
    file_select_button_->setStyleSheet("");

    statusText = new QTextEdit();
    currentTimeText = new QLineEdit();

    QHBoxLayout* button_layout = new QHBoxLayout;
    button_layout->addWidget( start_button_ );
    button_layout->addWidget( pause_button_ );
    button_layout->addWidget( stop_button_ );
    button_layout->addWidget( convert_button_ );


    tree_view_ = new QTreeView;

    QStringList headers;
    headers << "Event time" << "Event description";

    QString data = "\tlocation\t\t\nnavigation\t\t\t\nmovement\t\t\t\n";

    tree_model = new TreeModel(headers, data);

    tree_view_->setModel(tree_model);

  for (int column = 0; column < tree_model->columnCount(); ++column)
    tree_view_->resizeColumnToContents(column);



   eventText = new QLineEdit("event text");
  currentEventTimeText = new QLineEdit("event time");
  recordEventButton = new QPushButton("Mark Event");
  saveEventsButton = new QPushButton("Save events to file");

  QVBoxLayout* eventLayout = new QVBoxLayout;
  QVBoxLayout* eventLayoutButtons = new QVBoxLayout;
  QHBoxLayout* eventLayoutHorizontal = new QHBoxLayout;

  eventLayout->addWidget(eventText);
  eventLayout->addWidget(currentEventTimeText);

  eventLayoutButtons->addWidget(recordEventButton);
  eventLayoutButtons->addWidget(saveEventsButton);

  eventLayoutHorizontal->addLayout(eventLayout);
  eventLayoutHorizontal->addLayout(eventLayoutButtons);
  //recordEvent


  // Lay out the fields
    QVBoxLayout* main_layout = new QVBoxLayout;

    main_layout->addLayout(button_layout);
    main_layout->addWidget(file_select_button_);

    main_layout->addWidget(statusText);

    main_layout->addWidget(playback_realtime_);

    main_layout->addWidget(slider);
    main_layout->addWidget(currentTimeText);

    main_layout->addWidget(tree_view_);

  main_layout->addLayout(eventLayoutHorizontal);

    setLayout( main_layout );

    connect( file_select_button_, SIGNAL( pressed() ), this, SLOT( selectBagFile() ));
    connect( start_button_, SIGNAL( pressed() ), this, SLOT( startPressed() ));
    connect( pause_button_, SIGNAL( pressed() ), this, SLOT( pausePressed() ));
    connect( stop_button_, SIGNAL( pressed() ), this, SLOT( stopPressed() ));
  connect( convert_button_, SIGNAL( pressed() ), this, SLOT( convertPressed() ));

  connect( recordEventButton, SIGNAL( pressed() ), this, SLOT( recordEvent() ));
  connect( saveEventsButton, SIGNAL( pressed() ), this, SLOT( saveEventsToFile() ));

  connect(tree_view_,SIGNAL(clicked(QModelIndex)),this,SLOT(OnClickedTree(QModelIndex)));



  connect( playback_realtime_, SIGNAL( pressed() ), this, SLOT( changePlaybackRealtime() ));

    connect( slider, SIGNAL(sliderPressed()), this, SLOT(sliderPressed()));
    connect( slider, SIGNAL(sliderMoved(int)), this, SLOT(sliderMoved(int)));
    connect( slider, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(PollROS()));
    timer->start(100);

}


void DatasetPanel::selectBagFile(){
  QString fileName = QFileDialog::getOpenFileName(this, tr("Open dataset file"), "/media/stew/datasets/conversions/", tr("Bag Files (*.bag)"));
  std::cout << fileName.toStdString() << std::endl;

  // reset GUI
  slider->setValue(0.);

  workerThread = new DatasetThread();//this);

  workerThread->file_name = fileName.toStdString();

  connect(workerThread, SIGNAL(resultReady(QString)), this, SLOT(handleResults(QString)));
  connect(workerThread, SIGNAL(finished()), workerThread, SLOT(deleteLater()));
  workerThread->start();

  statusText->setText("Loading dataset");

  while (workerThread->current_state == DatasetThread::PLAYBACK_LOADING) {
//    std::cout << "loading..." << std::endl;
    ros::spinOnce();
  }

  double start_time = workerThread->bag_playback.requested_start_time.toSec();
  double end_time = workerThread->bag_playback.requested_end_time.toSec();
  double current_time = workerThread->bag_playback.last_packet_time.toSec();

  statusText->setText("Dataset details");
  statusText->append("Start time: " + QDateTime::fromSecsSinceEpoch(start_time).toString());
  statusText->append("End time: " + QDateTime::fromSecsSinceEpoch(end_time).toString());

  // get folder where the files are stored
  QFileInfo bag_file_folder(workerThread->bag_playback.bags.front()->bag_file_name.c_str());
  statusText->append("location: " + bag_file_folder.absoluteDir().dirName());


  for (auto bag: workerThread->bag_playback.bags) {
    QFileInfo bag_file(QString(bag->bag_file_name.c_str()));
    statusText->append("bag: " + bag_file.fileName());
  }

  for (auto video: workerThread->bag_playback.videos) {
    QFileInfo video_file(QString(video.second.file_name.c_str()));
    statusText->append("video: " + video_file.fileName());
  }


  currentTimeText->setText(QDateTime::fromSecsSinceEpoch(current_time).toString());

  playback_realtime_->setChecked(workerThread->bag_playback.limit_playback_speed);
  statusText->append("playback time scaled to " + QString::number(workerThread->bag_playback.scale_playback_speed));


  for (auto event_list: workerThread->bag_playback.dataset_events) {
    QString category = QString(event_list.first.c_str()).split('/').last();

    std::cout << "read a category " << category.toStdString() << std::endl;

    for (auto event_category_list: event_list.second) {
      addEventToCategory(category.toStdString(), event_category_list);
      std::cout << "data " << event_category_list->header.stamp.toSec() << ", " << event_category_list->event_description << std::endl;
    }
  }
}




void DatasetPanel::saveEventsToFile() {

  if (workerThread && !workerThread->bag_playback.bags.empty()) {
    QString event_bag_name = workerThread->bag_playback.bags.front()->bag_file_name.c_str();
    event_bag_name.insert(event_bag_name.lastIndexOf(".bag"), ".event");
    std::cout << "Writing events to " << event_bag_name.toStdString() << std::endl;

    rosbag::Bag event_bag;
    event_bag.open(event_bag_name.toStdString(), rosbag::bagmode::Write);

    //  Get top-level first.
    for (int i = 0; i < tree_model->rowCount(); ++i) {
      //  Use whatever column you are interested in.

      std::string category = tree_model->index(i, 0).data().toString().toLower().replace(' ', '_').toStdString();

      std::cout << "category: " << category << std::endl;

      for (int j = 0; j < tree_model->rowCount(tree_model->index(i, 0)); ++j) {
        //children << children[i].child( j, 0 );

        dataset_msgs::DatasetEvent new_event;

        auto boost_time = boost::posix_time::from_iso_extended_string(tree_model->index(i, 0).child(j, 0).data().toString().toStdString());
        boost::posix_time::time_duration diff = boost_time - boost::posix_time::from_time_t(0);

        new_event.header.stamp.sec = diff.total_seconds();
        new_event.header.stamp.nsec = diff.fractional_seconds()*1000;

        //new_event.header.stamp = requested_time;
        new_event.event_description = tree_model->index(i, 0).child(j, 1).data().toString().toStdString();

        event_bag.write("/event/" + category, new_event.header.stamp, new_event);

        std::cout << "events: " << tree_model->index(i, 0).child(j, 0).data().toString().toStdString() << ", "
                  << tree_model->index(i, 0).child(j, 1).data().toString().toStdString() << std::endl;
      }
    }

    event_bag.close();
  }
}

void DatasetPanel::recordEvent() {

  {
    QModelIndexList children;

    //  Get top-level first.
    for (int i = 0; i < tree_model->rowCount(); ++i) {
      children << tree_model->index(i, 0);  //  Use whatever column you are interested in.
    }

    for (auto child: children) {
      std::cout << "root: " << child.data().toString().toStdString() << std::endl;

    }


    /*
    // Now descend through the generations.
    for ( int i = 0; i < children.size(); ++i ) {
      for ( int j = 0; j < tree_model->rowCount( children[i] ); ++j ) {
        children << children[i].child( j, 0 );
      }
    }
     */


    // print each category
    QModelIndex index = tree_view_->selectionModel()->currentIndex().sibling(tree_view_->selectionModel()->currentIndex().row(), 0);

    QAbstractItemModel *test_model = tree_view_->model();

    QModelIndex parent_index = index;
    while (parent_index.parent().isValid())
      parent_index = parent_index.parent();

    std::cout << "selected parent " << parent_index.data().toString().toStdString() << std::endl;


    std::cout << index.data().toString().toStdString() << std::endl;

    for (int row = 0; row < tree_model->rowCount(); ++row) {
      QModelIndex child = tree_model->index(index.row() + 1, 1, index.parent());
      std::cout << child.data().toString().toStdString() << std::endl;
    }


    test_model->insertRow(0, parent_index);
    test_model->setData(tree_model->index(0, 0, parent_index), QVariant(currentEventTimeText->text()), Qt::EditRole);
    test_model->setData(tree_model->index(0, 1, parent_index), QVariant(eventText->text()), Qt::EditRole);

  }

  {
    //  Get top-level first.
  for (int i = 0; i < tree_model->rowCount(); ++i) {
    //  Use whatever column you are interested in.
    std::cout << "category: " << tree_model->index(i, 0).data().toString().toStdString() << std::endl;

    for ( int j = 0; j < tree_model->rowCount( tree_model->index(i, 0) ); ++j ) {
      //children << children[i].child( j, 0 );
      std::cout << "events: " << tree_model->index(i, 0).child(j,0).data().toString().toStdString() << ", " << tree_model->index(i, 0).child(j,1).data().toString().toStdString() << std::endl;
    }
  }


  /*
  // Now descend through the generations.
  for ( int i = 0; i < children.size(); ++i ) {
    for ( int j = 0; j < tree_model->rowCount( children[i] ); ++j ) {
      children << children[i].child( j, 0 );
    }
  }
   */

}

}



void DatasetPanel::addEventToCategory(std::string category, dataset_msgs::DatasetEvent::ConstPtr event) {

  QAbstractItemModel *test_model = tree_view_->model();

  for (int i = 0; i < tree_model->rowCount(); ++i) {
    auto parent_index = tree_model->index(i, 0);
    if (category == parent_index.data().toString().toStdString()) {
      test_model->insertRow(0, parent_index);
      std::string time_string = boost::posix_time::to_iso_extended_string(event->header.stamp.toBoost());
      test_model->setData(tree_model->index(0, 0, parent_index), QVariant(time_string.c_str()), Qt::EditRole);
      test_model->setData(tree_model->index(0, 1, parent_index), QVariant(event->event_description.c_str()), Qt::EditRole);
    }
  }

}


void DatasetPanel::convertPressed() {
  // test first whether a folder and files are selected

  // check the files size before, estimate the space required

  // write a confirmation dialog box to check whether the conversion should go ahead

  // perform the conversion
std::cout << "trying this " << std::endl;
  if (workerThread && workerThread->current_state == DatasetThread::PLAYBACK_PAUSE) {
    std::cout << "starting conversion" << std::endl;
    //execlp("/catkin/src/dataset_metapackage/dataset_tools/h264_bag_playback/scripts/convert_folder.sh",
/*    execv("/catkin/src/dataset_metapackage/dataset_tools/h264_bag_playback/scripts/convert_folder.sh",
           "/catkin/src/dataset_metapackage/dataset_tools/h264_bag_playback/scripts/convert_folder.sh",
           boost::filesystem::path(workerThread->bag_playback.bags.front()->bag_file_name).c_str());
    std::cout << "finished conversion" << std::endl;
*/  }

}

void DatasetPanel::sliderPressed(){
    // pause the playback

  if (workerThread && workerThread->current_state == DatasetThread::PLAYBACK_START) {
    workerThread->current_state = DatasetThread::PLAYBACK_PAUSE;
  }

}


void DatasetPanel::sliderMoved(int new_value) {

  if (workerThread && workerThread->current_state == DatasetThread::PLAYBACK_PAUSE) {
      double start_time = workerThread->bag_playback.requested_start_time.toSec();
      double end_time = workerThread->bag_playback.requested_end_time.toSec();
      double requested_time = start_time + ((end_time - start_time) * new_value / 1000.);

      currentTimeText->setText("seek to time: " + QDateTime::fromSecsSinceEpoch(requested_time).toString());
  }
}




void DatasetPanel::OnClickedTree(QModelIndex clicked_item) {

  std::cout << "clicked on the treeview" << std::endl;

  ros::Time requested_time(0);// =  boost::posix_time::to_iso_extended_string(workerThread->bag_playback.last_packet_time.toBoost()).c_str()
  QString requested_time_string;

  try {
  // print each category
  QModelIndex index = clicked_item.sibling(tree_view_->selectionModel()->currentIndex().row(), 0);
  std::cout << "data :" << index.data().toString().toStdString() << std::endl;

  auto boost_time = boost::posix_time::from_iso_extended_string(index.data().toString().toStdString());



  boost::posix_time::time_duration diff = boost_time - boost::posix_time::from_time_t(0);

  requested_time.sec = diff.total_seconds();
  requested_time.nsec = diff.fractional_seconds()*1000;

  requested_time_string = boost::posix_time::to_iso_extended_string(requested_time.toBoost()).c_str();

  }
  catch(...) {
    // couldn't convert the time
    std::cout << "clicked on the treeview, could not convert the value into a time" << std::endl;
    return;
  }

  if (workerThread) {

    workerThread->current_state = DatasetThread::PLAYBACK_PAUSE;

    while (workerThread->current_state != DatasetThread::PLAYBACK_PAUSE) {
      ros::spinOnce();
    }

    currentTimeText->setText("loading time: " + requested_time_string);

    workerThread->seek_time = ros::Time(requested_time);
    workerThread->current_state = DatasetThread::PLAYBACK_SEEK;

    currentTimeText->setText("still seeking: " + requested_time_string);

    while (workerThread->current_state == DatasetThread::PLAYBACK_SEEK) {
      ros::spinOnce();
    }

    workerThread->bag_playback.limit_playback_speed = !playback_realtime_->isChecked();

    currentTimeText->setText("finished seek");
  }
}


void DatasetPanel::sliderReleased() {

  if (workerThread && workerThread->current_state == DatasetThread::PLAYBACK_PAUSE) {

    int requested_value = slider->value();

    double start_time = workerThread->bag_playback.requested_start_time.toSec();
    double end_time = workerThread->bag_playback.requested_end_time.toSec();
    double requested_time = start_time + ((end_time - start_time) * requested_value / 1000.);

    currentTimeText->setText("loading time: " + QDateTime::fromSecsSinceEpoch(requested_time).toString());

    workerThread->seek_time = ros::Time(requested_time);
    workerThread->current_state = DatasetThread::PLAYBACK_SEEK;

    currentTimeText->setText("still seeking: " + QDateTime::fromSecsSinceEpoch(requested_time).toString());

    while (workerThread->current_state == DatasetThread::PLAYBACK_SEEK) {
      ros::spinOnce();
    }

    currentTimeText->setText("finished seek");
  }
}


void DatasetPanel::PollROS() {
  if (workerThread && workerThread->current_state == DatasetThread::PLAYBACK_START) {
    double start_time = workerThread->bag_playback.requested_start_time.toSec();
    double end_time = workerThread->bag_playback.requested_end_time.toSec();
    double current_time = workerThread->bag_playback.last_packet_time.toSec();

    currentEventTimeText->setText(boost::posix_time::to_iso_extended_string(workerThread->bag_playback.last_packet_time.toBoost()).c_str());

    double percentage_through = 0.;
    if (current_time - start_time > 0)
       percentage_through = 1000. * (double(current_time - start_time) / double(end_time - start_time));

    slider->setValue(percentage_through);
    currentTimeText->setText(QDateTime::fromSecsSinceEpoch(current_time).toString());
  }
  ros::spinOnce();
}


void DatasetPanel::changePlaybackRealtime(){
  if (workerThread) {
    workerThread->bag_playback.limit_playback_speed = !playback_realtime_->isChecked();
    //workerThread->bag_playback.scale_playback_speed = 1.;
    ROS_INFO_STREAM("is checked " << workerThread->bag_playback.limit_playback_speed);
  }
}


void DatasetPanel::startPressed() {
  if (workerThread){
    workerThread->current_state = DatasetThread::PLAYBACK_START;
  }
}

void DatasetPanel::stopPressed() {
  if (workerThread){
    workerThread->current_state = DatasetThread::PLAYBACK_STOP;

    std::cout << "exiting playback thread" << std::endl;
    workerThread->quit();
    std::cout << "waiting for finish" << std::endl;
    workerThread->wait();
    std::cout << "deleting playback thread" << std::endl;

    delete workerThread;
    workerThread = NULL;

  }
}

void DatasetPanel::pausePressed() {
  if (workerThread){
    workerThread->current_state = DatasetThread::PLAYBACK_PAUSE;
  }
}


