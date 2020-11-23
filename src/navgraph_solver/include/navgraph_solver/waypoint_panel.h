#ifndef WAYPOINT_PANEL_H
#define WAYPOINT_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>


# include <rviz/panel.h>
#endif

class QLineEdit;
class QPushButton;
class QVBoxLayout;
class QGroupBox ; 
class QString ; 
class QGridLayout ; 

namespace navgraph_rviz_panel
{
  class WaypointPanel: public rviz::Panel
  {
    Q_OBJECT
  public:
    WaypointPanel( QWidget* parent = 0 );

    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;
    QLineEdit* robot_pose_text;

    public Q_SLOTS:

    protected Q_SLOTS:
    void updatePoints();
    void updateAdjacency();
    void chargingButton();
    void freezeButton();
    void loadButton();
    void addButton();
    void missionButton();
    void abortButton();
    void deletePoints();
    void deleteAdjacency();
    void set_waypoint() ; 
    void set_station() ; 


  protected:

    QLineEdit* point_file_editor_;
    QLineEdit* adjacency_file_editor_;
    QString point_file_;
    QString adjacency_file_;
    QPushButton *charging_button;
    QPushButton *freeze_button;
    QPushButton *load_button;
    QPushButton *add_button;
    QPushButton *mission_button;
    QPushButton *abort_button;
    QPushButton *delete_points;
    QPushButton *delete_adjacency;
    QVBoxLayout* stations_layout;
    int station_count;
    ros::Publisher commands;
    ros::Publisher points_pub;
    ros::Publisher mission_pub;
    ros::NodeHandle nh_;

    QPushButton *set_waypoint_button;
    QPushButton *set_station_button;

    QGroupBox *RobotPoseGroup() ; 
    QGroupBox *SetNodeGroup() ; 
    void updateTextCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg ) ;  

    ros::Subscriber  pose_sub1 ;

  };

} // end namespace rviz_plugin_tutorials

#endif // WAYPOINT_PANEL_H