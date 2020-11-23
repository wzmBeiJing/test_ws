#include <stdio.h>
#include <iostream>

#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QCheckBox>
#include <QSpinBox>
#include <QComboBox>
#include <QMenu>
#include <QLabel>
#include <QListWidget>
#include <QGroupBox>

#include "navgraph_solver/waypoint_panel.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include <string>

namespace navgraph_rviz_panel
{

  WaypointPanel::WaypointPanel( QWidget* parent )
  : rviz::Panel( parent ), station_count(1)
  {
    QHBoxLayout* topic_layout = new QHBoxLayout;
    topic_layout->addWidget( new QLabel( "Points File:" ));
    point_file_editor_ = new QLineEdit;
    topic_layout->addWidget( point_file_editor_ );

    QHBoxLayout* topic_layout2 = new QHBoxLayout;
    topic_layout2->addWidget( new QLabel( "Adjacency File:" ));
    adjacency_file_editor_ = new QLineEdit;
    topic_layout2->addWidget( adjacency_file_editor_ );

    QHBoxLayout* points_layout = new QHBoxLayout;
    freeze_button = new QPushButton("Freeze OnScreen Points");
    points_layout->addWidget(freeze_button);
    load_button = new QPushButton("Load Files");
    points_layout->addWidget(load_button);

    QHBoxLayout* delete_layout = new QHBoxLayout;
    delete_points = new QPushButton("Delete Points");
    delete_layout->addWidget(delete_points);
    delete_adjacency = new QPushButton("Delete Adjacency");
    delete_layout->addWidget(delete_adjacency);

    QHBoxLayout* charging_layout = new QHBoxLayout;
    charging_button = new QPushButton("GO TO CHARGING STATION");
    charging_layout->addWidget(charging_button);
  // move_base_layout->addWidget( new QLabel( "Using files: " ));

    QHBoxLayout* station_layout = new QHBoxLayout;
    QCheckBox *local = new QCheckBox("DWA");
    station_layout->addWidget(local);
    station_layout->addWidget( new QLabel( "Station:" ));
    QSpinBox* station_id = new QSpinBox;
    station_layout->addWidget(station_id);
    QComboBox *comboBox = new QComboBox;
    comboBox->addItem("Pick Up Object");
    comboBox->addItem("Drop Object");
    comboBox->addItem("Charge Bot");
    station_layout->addWidget(comboBox);
    QCheckBox *checkbox = new QCheckBox("Stay at Station");
    station_layout->addWidget(checkbox);

    stations_layout = new QVBoxLayout;
    QCheckBox *loop = new QCheckBox("Loop Stations");
    stations_layout->addWidget(loop);
    stations_layout->addLayout( station_layout );

    QHBoxLayout* mission_layout = new QHBoxLayout;
    add_button = new QPushButton("Add Station");
    mission_layout->addWidget(add_button);
    mission_button = new QPushButton("Publish Mission");
    mission_layout->addWidget(mission_button);
    abort_button = new QPushButton("Abort Mission");
    mission_layout->addWidget(abort_button);

    QGridLayout* teach_mode = new QGridLayout;
    teach_mode->addWidget(RobotPoseGroup(),0,0) ; 
    teach_mode->addWidget(SetNodeGroup(),0,1) ; 



    QVBoxLayout* layout = new QVBoxLayout;
    layout->addLayout( teach_mode );
    layout->addLayout( topic_layout );
    layout->addLayout( topic_layout2 );
    layout->addLayout( points_layout );
    layout->addLayout( delete_layout );
    layout->addLayout(charging_layout);
    layout->addLayout(stations_layout);
    layout->addLayout(mission_layout);
    setLayout( layout );

    connect( point_file_editor_, SIGNAL( editingFinished() ), this, SLOT( updatePoints() ));
    connect( adjacency_file_editor_, SIGNAL( editingFinished() ), this, SLOT( updateAdjacency() ));
    connect(set_waypoint_button, SIGNAL (released()), this, SLOT (set_waypoint()));
    connect(set_station_button, SIGNAL (released()), this, SLOT (set_station()));
    connect(charging_button, SIGNAL (released()), this, SLOT (chargingButton()));
    connect(freeze_button, SIGNAL (released()), this, SLOT (freezeButton()));
    connect(load_button, SIGNAL (released()), this, SLOT (loadButton()));
    connect(add_button, SIGNAL (released()), this, SLOT (addButton()));
    connect(mission_button, SIGNAL (released()), this, SLOT (missionButton()));
    connect(abort_button, SIGNAL (released()), this, SLOT (abortButton()));
    connect(delete_points, SIGNAL (released()), this, SLOT (deletePoints()));
    connect(delete_adjacency, SIGNAL (released()), this, SLOT (deleteAdjacency()));

    commands = nh_.advertise<std_msgs::Int32>("/panel_msgs", 5);
    mission_pub = nh_.advertise<std_msgs::String>("/mission", 5);
    points_pub = nh_.advertise<std_msgs::String>("/points_files", 5);

    pose_sub1 = nh_.subscribe("/amcl_pose", 10, &WaypointPanel::updateTextCallback, this);

    charging_button->setEnabled(true);
    loop->setEnabled(false);

  }

  void WaypointPanel::updateTextCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg )
  {
    double x = msg->pose.pose.position.x ; 
    double y = msg->pose.pose.position.y ; 
    std::stringstream sp ; 
    sp <<"X: "<<std::setprecision(4)<<x<<" ; "<<"Y: "<<std::setprecision(4)<<y ; 
    QString qtxt = QString::fromStdString(sp.str()) ; 
    robot_pose_text->setText(qtxt) ;
  }

  QGroupBox *WaypointPanel::RobotPoseGroup(){
    QGroupBox *groupBox = new QGroupBox(tr("Robot Position: "));
    robot_pose_text = new QLineEdit ; 
    robot_pose_text->setPlaceholderText("robot position") ; 
  //robot_pose_text->setReadOnly(true) ; 
    QVBoxLayout *vbox = new QVBoxLayout;
    vbox->addWidget(robot_pose_text) ; 
    groupBox->setLayout(vbox) ; 
    return groupBox ; 
  }
  QGroupBox *WaypointPanel::SetNodeGroup(){
    QGroupBox *groupBox2 = new QGroupBox(tr("Set waypoint/station :"));
    set_waypoint_button = new QPushButton("Place Waypoint");
    set_station_button = new QPushButton("Place Station");
    QVBoxLayout *vbox = new QVBoxLayout;
    vbox->addWidget(set_waypoint_button);
    vbox->addWidget(set_station_button);
    groupBox2->setLayout(vbox) ; 
    return groupBox2 ; 
  }

  void WaypointPanel::set_waypoint(){
    std_msgs::Int32 msg;
    msg.data = 17;
    commands.publish(msg);
  }

  void WaypointPanel::set_station(){
    std_msgs::Int32 msg;
    msg.data = 18;
    commands.publish(msg);
  }

  void WaypointPanel::updatePoints()
  {
    point_file_ = point_file_editor_->text();
    Q_EMIT configChanged();
  // move_base_button->setEnabled( point_file_ != "" && adjacency_file_ != "");
    load_button->setEnabled( point_file_ != "");
  }

  void WaypointPanel::updateAdjacency()
  {
    adjacency_file_ = adjacency_file_editor_->text();
    Q_EMIT configChanged();
  // move_base_button->setEnabled( point_file_ != "" && adjacency_file_ != "");
    load_button->setEnabled( point_file_ != "");
  }

  void WaypointPanel::freezeButton(){
    std_msgs::Int32 msg;
    msg.data = 1;
    commands.publish(msg);

    std::stringstream filename, adjacency;
    std::string workspace;
	  ros::param::get("/workspace", workspace);
	  filename << "/home/" << getenv("USER") <<"/"<< workspace <<"/src/navgraph_solver/navgraphs/points_rviz.csv";
		adjacency << "/home/" << getenv("USER") <<"/"<< workspace <<"/src/navgraph_solver/navgraphs/adjacency_rviz.csv";

    point_file_editor_->setText(QString::fromStdString(filename.str()));
    updatePoints();
    adjacency_file_editor_->setText(QString::fromStdString(adjacency.str()));
    updateAdjacency();
  }

  void WaypointPanel::loadButton(){
    std_msgs::Int32 msg;
    msg.data = 2;
    commands.publish(msg);
    std_msgs::String files;
    std::stringstream s;
    s << point_file_editor_->text().toStdString() << "+" << adjacency_file_editor_->text().toStdString();
    files.data = s.str();
    points_pub.publish(files);
  }

  void WaypointPanel::chargingButton(){
    std_msgs::Int32 msg;
    msg.data = 15;
    commands.publish(msg);
  }

  void WaypointPanel::addButton(){
    QHBoxLayout* station_layout = new QHBoxLayout;
    QCheckBox *local = new QCheckBox("DWA");
    station_layout->addWidget(local);
    station_layout->addWidget( new QLabel( "Station:" ));
    QSpinBox* station_id = new QSpinBox;
    station_layout->addWidget(station_id);
    QComboBox *comboBox = new QComboBox;
    comboBox->addItem("Pick Up Object");
    comboBox->addItem("Drop Object");
    comboBox->addItem("Charge Bot");
    station_layout->addWidget(comboBox);
    QCheckBox *checkbox = new QCheckBox("Stay at Station");
    station_layout->addWidget(checkbox);

    stations_layout->addLayout( station_layout );
  }

  void WaypointPanel::missionButton(){
    std_msgs::String msg;
    std::stringstream s;
    for (int i = 1; i < stations_layout->count(); i++){
      QWidget *planner_widget = stations_layout->itemAt(i)->layout()->itemAt(0)->widget();
      QWidget *station_widget = stations_layout->itemAt(i)->layout()->itemAt(2)->widget();
      QWidget *task_widget = stations_layout->itemAt(i)->layout()->itemAt(3)->widget();
      QWidget *stay_widget = stations_layout->itemAt(i)->layout()->itemAt(4)->widget();
      QSpinBox *station_widget_cast;
      QComboBox *task_widget_cast;
      QCheckBox *stay_widget_cast;
      QCheckBox *planner_widget_cast;
      station_widget_cast = qobject_cast<QSpinBox*>(station_widget);
      task_widget_cast = qobject_cast<QComboBox*>(task_widget);
      stay_widget_cast = qobject_cast<QCheckBox*>(stay_widget);
      planner_widget_cast = qobject_cast<QCheckBox*>(planner_widget);
      int station = station_widget_cast->value();
      int task = task_widget_cast->currentIndex();
      bool stay = stay_widget_cast->isChecked();
      bool planner = planner_widget_cast->isChecked();

      s << station << " " << task << " " << stay << " " << planner << "/";

    }
    msg.data = s.str();
    mission_pub.publish(msg);
    mission_button->setEnabled(false);
  }

  void WaypointPanel::abortButton(){
    std_msgs::Int32 msg;
    msg.data = 4;
    commands.publish(msg);
    mission_button->setEnabled(true);
  }

  void WaypointPanel::deletePoints(){
    std_msgs::Int32 msg;
    msg.data = 5;
    commands.publish(msg);
  }

  void WaypointPanel::deleteAdjacency(){
    std_msgs::Int32 msg;
    msg.data = 6;
    commands.publish(msg);
  }

  void WaypointPanel::save( rviz::Config config ) const
  {
    rviz::Panel::save( config );
    config.mapSetValue( "Points", point_file_ );
    config.mapSetValue( "Adjacency", adjacency_file_ );
  }

  void WaypointPanel::load( const rviz::Config& config )
  {
    rviz::Panel::load( config );
    if( config.mapGetString( "Points", &point_file_ ))
    {
      point_file_editor_->setText( point_file_ );
      updatePoints();
    }
    if( config.mapGetString( "Adjacency", &adjacency_file_ ))
    {
      adjacency_file_editor_->setText( adjacency_file_ );
      updateAdjacency();
    }
  }

} // end namespace rviz_plugin_tutorials

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(navgraph_rviz_panel::WaypointPanel,rviz::Panel )
