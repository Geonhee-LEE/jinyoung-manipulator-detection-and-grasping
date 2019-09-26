/**
 * @file /include/qtros/main_window.hpp
 *
 * @brief Qt based gui for qtros.
 *
 * @date November 2010
 **/
#ifndef qtros_MAIN_WINDOW_H
#define qtros_MAIN_WINDOW_H

/*****************************************************************************
** Includes
**************************************#include <QtNetwork/QTcpSocket>***************************************/
#ifndef Q_MOC_RUN
#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "QtNetwork/QTcpSocket"
#include "math.h"
#include <QThread>
#include "thread"
#include <boost/thread/thread.hpp>
#include "QFile"
#include "QTextStream"



#endif
/*****************************************************************************
** Namespace
*****************************************************************************/
class QTcpSocket;
namespace qtros {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */


class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
  void showNoMasterMessage();
  void Thread_point();
  void Check_pixel();
  void Repeat();


  void Gripper_Open();
  void Gripper_Close();

  void SonaMalding_Start();
  void SonaMalding_Stop();
  void pick_and_place_task();

  void inverse_and_move(double position_x,double position_y,double position_z,double orientation_r,double orientation_p,double orientation_y);
  void joint_and_move(double joint_1, double joint_2, double joint_3, double joint_4, double joint_5, double joint_6);
  void Task_pick_and_place();
  void test_move_func();
  void tracking();
  void tracking_test1();

  void _190714_trakint_test();
  void _190714_trakint_test1();
  void _190714_trakint_test2();
  void _190714_trakint_test3();
  void _190714_trakint_test4();

  void _190714_trakint_test_sona_500x300_point();
  void _190714_trakint_test_sona_700x300_point();
  void _190714_trakint_test_sona_700x200_point();

  void _task_function();
  void _task_moveit_function();

  void _task_test();
  void _task_moveit_test();

  void _inverse_Moveit(double x_point, double y_point, double z_point, double roll, double pitch, double yaw);
  void _inverse_point(double x_point, double y_point, double z_point, double roll, double pitch, double yaw);

  geometry_msgs::Vector3 Quaternion_to_RPY();
  geometry_msgs::Vector3 Quaternion_to_RPY(geometry_msgs::PoseStamped current_pose, tf::Quaternion quat);
  tf::Quaternion RPY_to_Quaternion(double rotate_x, double rotate_y, double rotate_z);

  bool check = true;
  bool complete_pose = false;
  bool check_pixel_enable = true;




  //void moveit_test(moveit::planning_interface::MoveGroupInterface *move_group);
 public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
  *******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);


  /******************************************
   ** TCP/IP client set
   ** **************************************/
   void error();
   void ReceivedreadyRead();
   void connect();
    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void on_Joint_subscriber_clicked();
    void on_btnsend_clicked();
    void on_btnConnect_clicked();
    void on_Joint_FwValue_clicked();
    void on_btnMoveit_setvalue_clicked();
    void on_btnMoveit_setJointValue_clicked();
    void on_btnDisConnect_clicked();
    void on_btnSRV_ON_clicked();
    void on_btnSRV_OFF_clicked();
    void on_btnRobot_SetJoint_clicked();
    void on_btnRobot_GetJoint_clicked();
    void on_btnPosition_Set_clicked();
    void on_btnFree_Set_clicked();
    void on_btnSRV_ErrorClear_clicked();
    void on_btnHomming_clicked();
    void on_btnSendtoRobot_clicked();
    void SyncMoveitToRobot();
    void on_btnDetectPose_clicked();
    void on_btntest_clicked();
    void on_btnEndToCameraCoordinate_clicked();
    double HomogeneousTransformationMatrix(double theta, double alpha, double ai, double di);
    void on_btnMoveToPixelPoint_clicked();
    void on_btnGPIO_ON_clicked();
    void on_btnGPIO_OFF_clicked();
    void on_btnpixel_check_clicked();
    void on_btnRepeat_clicked();
    void on_btn_test_move_clicked();
    void on_btn_fk_clicked();
    void on_btn_get_joint_clicked();
    void on_btn_get_orientation_clicked();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
  double pi = 3.141592;
  double Joint_threshold = 0.05;
  int flag;
  double error_1;
  double error_2;
  double error_3;
  double error_4;
  double error_5;
  double error_6;

  double Moveit_Endpoint_x;
  double Moveit_Endpoint_y;
  double Moveit_Endpoint_z;
  double Moveit_Endpoint_orientation_x;
  double Moveit_Endpoint_orientation_y;
  double Moveit_Endpoint_orientation_z;
  double Moveit_Endpoint_orientation_w;


  double Moveit_Jointvalue_1;
  double Moveit_Jointvalue_2;
  double Moveit_Jointvalue_3;
  double Moveit_Jointvalue_4;
  double Moveit_Jointvalue_5;
  double Moveit_Jointvalue_6;

  double Robot_GetJointValue_1;
  double Robot_GetJointValue_2;
  double Robot_GetJointValue_3;
  double Robot_GetJointValue_4;
  double Robot_GetJointValue_5;
  double Robot_GetJointValue_6;


  double _CurrentCamera_x;
  double _CurrentCamera_y;
  double _CurrentCamera_z;
  double _CurrentCamera_R;
  double _CurrentCamera_P;
  double _CurrentCamera_Y;

  double _task_roll = -94.46;
  double _task_pitch = -75.67;
  double _task_yaw = 94.46;

  double _task_sona_point_roll = -94.46;
  double _task_sona_point_pitch = -75.67;
  double _task_sona_point_yaw = 4.46;

  double point_enclosure_x;

  double avg_velocity;
  int vel_check_count;

  int grip_value;
  int inv_count = 0;

  bool found_ik;
  std::vector<double> joint_values;
  QTcpSocket* tcpSocket;
  quint16 nextBlockSize = 0;
  boost::thread th1;





};

}  // namespace qtros

#endif // qtros_MAIN_WINDOW_H
