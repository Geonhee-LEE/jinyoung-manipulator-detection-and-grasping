/**
 * @file /include/qtros/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef qtros_QNODE_HPP_
#define qtros_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include "sensor_msgs/JointState.h"
#include "iostream"
#include "vector"
#include "find_object_2d/ObjectsStamped.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <QTransform>
#include <QtNetwork/QTcpSocket>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_interface/planning_interface.h>
#endif


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qtros {

/*****************************************************************************
** Class
*******#ifndef Q_MOC_RUN**********************************************************************/

class QNode : public QThread {
    Q_OBJECT

public:

  QNode(int argc, char** argv);
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
  void chatterCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void ObjectCallback(const std_msgs::Float32MultiArrayConstPtr& msg);
  void detectObjectCallback(const std_msgs::Float32MultiArrayConstPtr& msg);
  void detectLabeledCallback(const std_msgs::Float32MultiArrayConstPtr& msg);
  moveit::planning_interface::MoveGroupInterface *move_group;
  const std::string PLANNING_GROUP = "arm"; // JointModelGroup
  QString Robot_CurrentValue;
  QStringListModel logging_model;
  robot_model::RobotModelPtr kinematic_model;


	void run();
	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);
  double GetJointValueArr[9];

  ros::Time cur_time;
  ros::Time pre_time;

  double cur_time_double;
  double pre_time_double;

  float _currentX;
  float _currentY;
  float _currentZ;
  float _current_Orientation_X;
  float _current_Orientation_Y;
  float _current_Orientation_Z;

  float _detectPixelObject_x;
  float _detectPixelObject_y;
  float _detectPixelObject_z;
  float _detectPixelObject_degree;
  float _detectEnableGripper;//important
  float _detectEnabledepth;
  int _detectpixel_u;
  int _detectpixel_v;
  float _detectNKalmanObject_x;
  float _detectNKalmanObject_y;

  //Label_center pixel point;
  float _labeledCenter_x;
  float _labeledCenter_y;

  float _pre_labeledCenter_x;
  float _pre_labeledCenter_y;

  //float _dist_pixel_u = 1.923;
  //float _dist_pixel_v = 1.681;

  //Calibration offset
  float _dist_pixel_u = 1.923;
  float _dist_pixel_v = 2.0588;


  float _labeledCenter_calculate_x; //camera coordinate 457, 471;
  float _labeledCenter_calculate_y;

  float _pre_labeledCenter_calculate_x;
  float _pre_labeledCenter_calculate_y;

  float _expected_x;
  float _expected_y;

  float _expected_2_x;
  float _expected_2_y;

  double velocity_enclosure;


  cv::Mat image;



Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();

public Q_SLOTS:

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
  ros::Subscriber sub;
  ros::Subscriber subObject;
  ros::Subscriber sub_detectObject;
  ros::Subscriber sub_labeledObject;


//  const robot_state::JointModelGroup *joint_model_group;

};

}  // namespace qtros

#endif /* qtros_QNODE_HPP_ */
