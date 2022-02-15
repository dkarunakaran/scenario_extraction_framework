#ifndef BUS_STOP_PANEL_H
#define BUS_STOP_PANEL_H

#include <ros/ros.h>

#include <QFileDialog>

#include <QTextEdit>
#include <QLineEdit>

#include <QThread>
#include <QThread>
#include <QSlider>
#include <QMouseEvent>

#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QCheckBox>

#include <h264_bag_playback/h264_bag_playback.hpp>

#include <QTreeView>
#include "treemodel.h"

class DatasetThread : public QThread
{


  Q_OBJECT

  void run() {
        QString result;
        /* expensive or blocking operation  */
        ros::NodeHandle("~").setParam("bag_file", file_name);

        bag_playback.init_playback();
        bag_playback.limit_playback_speed = false;

        bag_playback.OpenBags();

        current_state = PLAYBACK_PAUSE;

        do
        {
            while(current_state == PLAYBACK_PAUSE) {
            // Spin once so that any other ros controls/pub/sub can be actioned
              ros::spinOnce();
            }

            if (!ros::ok() || current_state == PLAYBACK_STOP) {
              break;
            }

            if (current_state == PLAYBACK_SEEK) {
              /*if (seek_time < bag_playback.last_packet_time) {
                // need to reload the data as the iterator only goes one way
                bag_playback.init_playback();

                for (auto video: bag_playback.videos) {
                  video.second.frame_counter = 0;
                }

                bag_playback.OpenBags();
              }
               */
              bag_playback.SeekTime(seek_time);
              current_state = PLAYBACK_START;
            }

            if (!bag_playback.ReadNextPacket()) {
              current_state = PLAYBACK_PAUSE;
            }

        } while (true);

        bag_playback.CloseBags();

        emit resultReady(result);
    }
signals:
    void resultReady(const QString &s);

public:
    std::string file_name;

    ros::Time seek_time;

    typedef enum PLAYBACK_CONTROL {
      PLAYBACK_LOADING,
      PLAYBACK_STOP,
      PLAYBACK_START,
      PLAYBACK_PAUSE,
      PLAYBACK_SEEK
    } PLAYBACK_CONTROL;

    PLAYBACK_CONTROL current_state;

    dataset_toolkit::h264_bag_playback bag_playback;

    DatasetThread() {current_state = PLAYBACK_LOADING;}

};


class DatasetPanel: public QFrame
{
    Q_OBJECT
public:
    DatasetPanel( QWidget* parent = 0 );

    DatasetThread *workerThread;

protected Q_SLOTS:

    // slot for when the start button is pressed
    void startPressed();
    void stopPressed();
    void pausePressed();
    void convertPressed();

    void sliderPressed();
    void sliderMoved(int new_value);
    void sliderReleased();

    void changePlaybackRealtime();

    void OnClickedTree(QModelIndex clicked_item);

  void recordEvent();
  void saveEventsToFile();

    void PollROS();

    void selectBagFile();


protected:


  void addEventToCategory(std::string category, dataset_msgs::DatasetEvent::ConstPtr event);

    QPushButton* start_button_;
    QPushButton* pause_button_;
    QPushButton* stop_button_;
  QPushButton* convert_button_;
    QPushButton* file_select_button_;

    QCheckBox* playback_realtime_;

    QTreeView* tree_view_;
    TreeModel* tree_model;


  QLineEdit *eventText;
  QLineEdit *currentEventTimeText;
  QPushButton* recordEventButton;
  QPushButton* saveEventsButton;


  QTextEdit *statusText;
    QLineEdit *currentTimeText;

    QSlider *slider;

    // The ROS node handle.
    ros::NodeHandle nh_;

};


#endif // BUS_STOP_PANEL_H
