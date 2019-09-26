/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include "../include/qtros/main_window.hpp"
#include <string>
#include <fstream>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <QCoreApplication>
#include <QFile>
#include <QString>
#include <QDebug>
#include <QTextStream>

//include unfind


/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace qtros {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/



MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent)
  , qnode(argc,argv)
{
  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
  ReadSettings();
  setWindowIcon(QIcon(":/images/icon.png"));
  ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  /*********************
  ** Logging
  **********************/
  ui.view_logging->setModel(qnode.loggingModel());
  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

  /*********************
  ** thread
  **********************/



  /*********************
  ** TCP/IP init
  **********************/
  tcpSocket = new QTcpSocket(this);
  QObject::connect(tcpSocket,SIGNAL(readyRead()),this,SLOT(ReceivedreadyRead()));
  QObject::connect(tcpSocket,SIGNAL(connected()),this,SLOT(connect()));
  QObject::connect(tcpSocket,SIGNAL(error(QAbstractSocket::SocketError)),this,SLOT(error()));




  /*********************
   ** Moveit API
   ** ******************/


 // QObject::connect(ui.btnMoveit_setJointValue,SIGNAL(clicked(bool)),this,SLOT(SyncMoveitToRobot()));


    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }

 // tcpClient->connectToHost(ui.txtTcpip->text(), ui.txtport->text().toInt());
}

MainWindow::~MainWindow() {
  delete tcpSocket;
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
    close();
}

/*****************TCP/IP***************
 * *************************/

void MainWindow::connect()
{

  QString msg  = "Connect";
  ui.listWidget_Server_text->addItem(QString("%1").arg(msg));
  ui.listWidget_Server_text->scrollToBottom();
  //tcpSocket->write(msg.toUtf8());
}

void MainWindow::error()
{
  QMessageBox msgBox;
  msgBox.setText("error!");
  msgBox.exec();
    close();
}



void MainWindow::ReceivedreadyRead()
{
  QByteArray Message = tcpSocket->readAll();
  QString msg = QString::fromUtf8(Message.data());
  ui.listWidget_Server_text->addItem(QString("%1").arg(msg));
  ui.listWidget_Server_text->scrollToBottom();

  if(msg.contains("@"))
  {
    QString Joint_1 = msg.section("@",1,1);
    QStringList Robot_Jointlist = msg.split("@");
    int Robot_Jointlength = Robot_Jointlist.length();
    Robot_GetJointValue_1 = Robot_Jointlist[1].toDouble();
    Robot_GetJointValue_2 = Robot_Jointlist[2].toDouble();
    Robot_GetJointValue_3 = Robot_Jointlist[3].toDouble();
    Robot_GetJointValue_4 = Robot_Jointlist[4].toDouble();
    Robot_GetJointValue_5 = Robot_Jointlist[5].toDouble();
    Robot_GetJointValue_6 = Robot_Jointlist[6].toDouble();

    ui.txtRobotGet_Jointvalue_1->setText(QString("%1").arg(Robot_GetJointValue_1));
    ui.txtRobotGet_Jointvalue_2->setText(QString("%1").arg(Robot_GetJointValue_2));
    ui.txtRobotGet_Jointvalue_3->setText(QString("%1").arg(Robot_GetJointValue_3));
    ui.txtRobotGet_Jointvalue_4->setText(QString("%1").arg(Robot_GetJointValue_4));
    ui.txtRobotGet_Jointvalue_5->setText(QString("%1").arg(Robot_GetJointValue_5));
    ui.txtRobotGet_Jointvalue_6->setText(QString("%1").arg(Robot_GetJointValue_6));
  /*  QString GetJoint_msg = QString("\nROBOT State!\nJ1: %1\nJ2: %2\nJ3: %3\nJ4: %4\nJ5: %5\nJ6: %6\n").arg(Robot_GetJointValue_1).arg(Robot_GetJointValue_2)
                  .arg(Robot_GetJointValue_3).arg(Robot_GetJointValue_4).arg(Robot_GetJointValue_5).arg(Robot_GetJointValue_6);
    qnode.logging_model.insertRows(qnode.logging_model.rowCount(),1);
    QModelIndex index = qnode.logging_model.index(qnode.logging_model.rowCount()-1);
    qnode.logging_model.setData(index,GetJoint_msg);*/
  }
  //if(msg.contains("complete"))
  //{
    complete_pose = true;
  //}

  //Debug
  ROS_INFO("ReceivedreadyRead");
  QString joint = QString("ReceivedreadyRead");
  ui.listWidget_Server_text->addItem(QString("%1").arg(joint));
  ui.listWidget_Server_text->scrollToBottom();


}



void MainWindow::on_button_connect_clicked(bool check ) {
   //ROS Connection
  if ( ui.checkbox_use_environment->isChecked() ) {
    if ( !qnode.init() ) {
      showNoMasterMessage();
    } else {
      ui.button_connect->setEnabled(false);
    }
  } else {
    if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
           ui.line_edit_host->text().toStdString()) ) {
      showNoMasterMessage();
    } else {
      ui.button_connect->setEnabled(false);
      ui.line_edit_master->setReadOnly(true);
      ui.line_edit_host->setReadOnly(true);
      ui.line_edit_topic->setReadOnly(true);
      ui.lblRos_State->setText("Connect");
    }
  }
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
  bool enabled;
  if ( state == 0 ) {
    enabled = true;
  } else {
    enabled = false;
  }
  ui.line_edit_master->setEnabled(enabled);
  ui.line_edit_host->setEnabled(enabled);
}

void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "qtros");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
      ui.line_edit_master->setEnabled(false);
      ui.line_edit_host->setEnabled(false);
      //ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "qtros");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
  WriteSettings();
  QMainWindow::closeEvent(event);
}

void MainWindow::SyncMoveitToRobot()
{

}

}  // namespace qtros


void qtros::MainWindow::on_Joint_subscriber_clicked()   //we can get the endpoint values
{
  //  moveit::planning_interface::PlanningSceneInterface planning_scene_interface; //virtual space
  //  const robot_state::JointModelGroup* joint_model_group =
  //    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP); // Current Joint State
  //   moveit::core::RobotStatePtr current_state = move_group.getCurrentState()

      //moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
      geometry_msgs::PoseStamped current_pose = qnode.move_group->getCurrentPose();
      Moveit_Endpoint_x = current_pose.pose.position.x;
      Moveit_Endpoint_y = current_pose.pose.position.y;
      Moveit_Endpoint_z = current_pose.pose.position.z;
      tf::Quaternion q(current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll,pitch,yaw);
      Moveit_Endpoint_orientation_x = roll * 180/pi;
      Moveit_Endpoint_orientation_y = pitch * 180/pi;
      Moveit_Endpoint_orientation_z = yaw * 180/pi;

      qnode.Robot_CurrentValue = QString("X: %1\nY: %2\nZ: %3\nRotate_x: %4\nRotate_y: %5\nRotate_z: %6\n").arg(Moveit_Endpoint_x).arg(Moveit_Endpoint_y)
                                          .arg(Moveit_Endpoint_z).arg(Moveit_Endpoint_orientation_x).arg(Moveit_Endpoint_orientation_y).arg(Moveit_Endpoint_orientation_z);
     // ROS_INFO("Current EndEffector: %f %f %f %f %f %f %f",Moveit_Endpoint_x,Moveit_Endpoint_y,Moveit_Endpoint_z,Moveit_Endpoint_orientation_x,Moveit_Endpoint_orientation_y,Moveit_Endpoint_orientation_z);
      QString str = QString("%1").arg(qnode.Robot_CurrentValue);
      ui.txtJointvalue->setText(str);
}


void qtros::MainWindow::on_btnsend_clicked()
{
    QString msg = ui.txtSend->text().trimmed();
    if(!msg.isEmpty())
    {
      tcpSocket->write(QString(msg).toLocal8Bit().constData());

    }
    ui.txtSend->clear();
    ui.txtSend->setFocus();
    ui.listWidget_Server_text->addItem(QString("%1").arg(msg));
    ui.listWidget_Server_text->scrollToBottom();
}


void qtros::MainWindow::on_btnConnect_clicked()
{
  // TCP IP connect
  QString TcpIp = ui.txtTcpip->text();
  int port = ui.txtport->text().toInt();
  tcpSocket->connectToHost(TcpIp,port);
}

void qtros::MainWindow::on_btnDisConnect_clicked()
{
    tcpSocket->close();
    QString msg  = "DisConnect";
    ui.listWidget_Server_text->addItem(QString("%1").arg(msg));
    ui.listWidget_Server_text->scrollToBottom();
}


void qtros::MainWindow::on_Joint_FwValue_clicked()
{ 
  const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
  const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
  Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
  Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
  Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
  Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
  Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();


  QString str = QString("joint_1: %1\njoint_2: %2\njoint_3: %3\njoint_4: %4\njoint_5: %5\njoint_6: %6\n").arg(Moveit_Jointvalue_1)
                          .arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5)
                          .arg(Moveit_Jointvalue_6);
  ui.txtFWJointValue->setText(str);
}

void qtros::MainWindow::on_btn_get_joint_clicked()
{
  const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
  moveit::core::RobotStatePtr current_state = qnode.move_group->getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group,joint_group_positions);
 
  qnode.move_group->setEndEffectorLink("gripper");
  geometry_msgs::PoseStamped current_pose = qnode.move_group->getCurrentPose();

  // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
  tf::Quaternion quat;

  //////////////
  tf::quaternionMsgToTF(current_pose.pose.orientation, quat);

  // the tf::Quaternion has a method to acess roll pitch and yaw
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  // the found angles are written in a geometry_msgs::Vector3
  geometry_msgs::Vector3 rpy;
  rpy.x = roll;
  rpy.y = pitch;
  rpy.z = yaw;

  geometry_msgs::Pose target_pose;


  target_pose.position.x = current_pose.pose.position.x;
  target_pose.position.y = current_pose.pose.position.y;
  target_pose.position.z = current_pose.pose.position.z;

  target_pose.orientation.x = quat.x();
  target_pose.orientation.y = quat.y();
  target_pose.orientation.z = quat.z();
  target_pose.orientation.w = quat.w();


  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
  const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
  found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms
  


  if (found_ik)
  {
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
    Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
    Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
    Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
    Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
    Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();
  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }

  ui.txtsetJointValue_1->setText(QString("%1").arg(Moveit_Jointvalue_1));
  ui.txtsetJointValue_2->setText(QString("%1").arg(Moveit_Jointvalue_2));
  ui.txtsetJointValue_3->setText(QString("%1").arg(Moveit_Jointvalue_3));
  ui.txtsetJointValue_4->setText(QString("%1").arg(Moveit_Jointvalue_4));
  ui.txtsetJointValue_5->setText(QString("%1").arg(Moveit_Jointvalue_5));
  ui.txtsetJointValue_6->setText(QString("%1").arg(Moveit_Jointvalue_6));

}

void qtros::MainWindow::on_btn_get_orientation_clicked()
{ 
  
  const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
  moveit::core::RobotStatePtr current_state = qnode.move_group->getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group,joint_group_positions);
 
  qnode.move_group->setEndEffectorLink("gripper");
  geometry_msgs::PoseStamped current_pose = qnode.move_group->getCurrentPose();

  // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
  tf::Quaternion quat;

  //////////////
  tf::quaternionMsgToTF(current_pose.pose.orientation, quat);

  // the tf::Quaternion has a method to acess roll pitch and yaw
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  // the found angles are written in a geometry_msgs::Vector3
  geometry_msgs::Vector3 rpy;
  rpy.x = roll;
  rpy.y = pitch;
  rpy.z = yaw;
  
  
  ui.txtsetEndValue_roll->setText(QString("%1").arg(roll*180/pi));
  ui.txtsetEndValue_pitch->setText(QString("%1").arg(pitch*180/pi));
  ui.txtsetEndValue_yaw->setText(QString("%1").arg(yaw*180/pi));
  
}
void qtros::MainWindow::on_btnMoveit_setvalue_clicked()
{

  geometry_msgs::Pose target_pose;
  target_pose.position.x = ui.txtsetEndValue_x->text().trimmed().toDouble();
  target_pose.position.y = ui.txtsetEndValue_y->text().trimmed().toDouble();
  target_pose.position.z = ui.txtsetEndValue_z->text().trimmed().toDouble();
  double rotate_x = ui.txtsetEndValue_roll->text().trimmed().toDouble()*pi/180;
  double rotate_y = ui.txtsetEndValue_pitch->text().trimmed().toDouble()*pi/180;
  double rotate_z = ui.txtsetEndValue_yaw->text().trimmed().toDouble()*pi/180;
  tf::Quaternion q;

  q.setRPY(rotate_x,rotate_y,rotate_z);
 
  ROS_INFO("rad : roll, pitch, yaw : %f , %f , %f", rotate_x, rotate_y, rotate_z);
  
  QString rpy_rad_str = QString("rad : roll, pitch, yaw : %1, %2, %3").arg(rotate_x).arg(rotate_y).arg(rotate_z);
  ui.rpy_q_listwidget->addItem(QString("%1").arg(rpy_rad_str));

  ROS_INFO("deg : roll, pitch, yaw : %f , %f , %f", rotate_x*180/pi, rotate_y*180/pi, rotate_z*180/pi);
  
  QString rpy_deg_str = QString("deg : roll, pitch, yaw : %1, %2, %3").arg(rotate_x*180/pi).arg(rotate_y*180/pi).arg(rotate_z*180/pi);
  ui.rpy_q_listwidget->addItem(QString("%1").arg(rpy_deg_str));
  
  ROS_INFO("q.x, q.y, q.z, q.w : %f , %f , %f , %f", q.x(), q.y(), q.z(), q.w());

  QString q_rad_str = QString("quaternion : x, y, z, w : %1, %2, %3, %4").arg(q.x()).arg(q.y()).arg(q.z()).arg(q.w());
  ui.rpy_q_listwidget->addItem(QString("%1").arg(q_rad_str));
  ui.rpy_q_listwidget->addItem(QString("  "));

  target_pose.orientation.x = q.x();
  target_pose.orientation.y = q.y();
  target_pose.orientation.z = q.z();
  target_pose.orientation.w = q.w();

  qnode.move_group->setPoseTarget(target_pose);
  qnode.move_group->move();


    const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

    found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms

    if (found_ik)
    {
      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

      Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

    }
    else
    {
      ROS_INFO("Did not find IK solution");
    }

  QString joint_vals = QString("joint : %1, %2, %3, %4, %5, %6").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);
  ui.rpy_q_listwidget->addItem(QString("%1").arg(joint_vals));
  ui.rpy_q_listwidget->addItem(QString("  "));
  ui.rpy_q_listwidget->scrollToBottom();

}

void qtros::MainWindow::on_btnMoveit_setJointValue_clicked()
{
  const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
  moveit::core::RobotStatePtr current_state = qnode.move_group->getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group,joint_group_positions);
 
  joint_group_positions[0] = ui.txtsetJointValue_1->text().trimmed().toDouble()*pi/180;
  joint_group_positions[1] = ui.txtsetJointValue_2->text().trimmed().toDouble()*pi/180;
  joint_group_positions[4] = ui.txtsetJointValue_3->text().trimmed().toDouble()*pi/180;
  joint_group_positions[7] = ui.txtsetJointValue_4->text().trimmed().toDouble()*pi/180;
  joint_group_positions[8] = ui.txtsetJointValue_5->text().trimmed().toDouble()*pi/180;
  joint_group_positions[9] = ui.txtsetJointValue_6->text().trimmed().toDouble()*pi/180;

  qnode.move_group->setJointValueTarget(joint_group_positions);
  qnode.move_group->move();

  // Get pose & orientation
  qnode.move_group->setEndEffectorLink("gripper");
  geometry_msgs::PoseStamped current_pose = qnode.move_group->getCurrentPose();

  // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
  tf::Quaternion quat;

  //////////////
  tf::quaternionMsgToTF(current_pose.pose.orientation, quat);

  // the tf::Quaternion has a method to acess roll pitch and yaw
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  // the found angles are written in a geometry_msgs::Vector3
  geometry_msgs::Vector3 rpy;
  rpy.x = roll;
  rpy.y = pitch;
  rpy.z = yaw;

  ROS_INFO("rad : roll, pitch, yaw : %f , %f , %f", roll, pitch, yaw);
  ROS_INFO("deg : roll, pitch, yaw : %f , %f , %f", roll*180/pi, pitch*180/pi, yaw*180/pi);
  
  QString rpy_rad_str = QString("rad : roll, pitch, yaw : %1, %2, %3").arg(roll).arg(pitch).arg(yaw);
  ui.rpy_q_listwidget->addItem(QString("%1").arg(rpy_rad_str));

  QString rpy_deg_str = QString("deg : roll, pitch, yaw : %1, %2, %3").arg(roll*180/pi).arg(pitch*180/pi).arg(yaw*180/pi);
  ui.rpy_q_listwidget->addItem(QString("%1").arg(rpy_deg_str));
  ui.rpy_q_listwidget->addItem(QString("  "));
  ui.rpy_q_listwidget->scrollToBottom();

  //double qx = pose.
  /////////////////
  /*
  ROS_INFO_STREAM(current_pose.pose.position.x <<"," << current_pose.pose.position.y <<"," << current_pose.pose.position.z <<"\n" <<
                  rpy.x<<"," << rpy.y<<"," << rpy.z);
  ROS_INFO_STREAM(current_pose.pose.orientation.x <<"," << current_pose.pose.orientation.y <<"," << current_pose.pose.orientation.z<<","<<
                  current_pose.pose.orientation.w);
  */

}

void qtros::MainWindow::on_btnSRV_ON_clicked()
{
    QString srv_on = "SRV_ON";
    tcpSocket->write(QString(srv_on).toLocal8Bit().constData());
    ui.listWidget_Server_text->addItem(QString("%1").arg(srv_on));
    ui.listWidget_Server_text->scrollToBottom();
}

void qtros::MainWindow::on_btnSRV_OFF_clicked()
{
  QString srv_off = "SRV_OFF";
  tcpSocket->write(QString(srv_off).toLocal8Bit().constData());
  ui.listWidget_Server_text->addItem(QString("%1").arg(srv_off));
  ui.listWidget_Server_text->scrollToBottom();
}

void qtros::MainWindow::on_btnRobot_SetJoint_clicked() //  BUTTEN SET
{


  QString joint = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(ui.txtRobotSet_Jointvalue_1->text()).arg(ui.txtRobotSet_Jointvalue_2->text()).arg(ui.txtRobotSet_Jointvalue_3->text()).arg(ui.txtRobotSet_Jointvalue_4->text()).arg(ui.txtRobotSet_Jointvalue_5->text()).arg(ui.txtRobotSet_Jointvalue_6->text());
  tcpSocket->write(QString(joint).toLocal8Bit().constData());
  ui.listWidget_Server_text->addItem(QString("%1").arg(joint));
  ui.listWidget_Server_text->scrollToBottom();

}

void qtros::MainWindow::on_btnRobot_GetJoint_clicked()
{

    QString msg = "SRV_GET_POSITION";
    tcpSocket->write(QString(msg).toLocal8Bit().constData());
    ui.listWidget_Server_text->addItem(QString("%1").arg(msg));
    ui.listWidget_Server_text->scrollToBottom();

}

void qtros::MainWindow::on_btnPosition_Set_clicked()
{
  QString msg = "MODE_OF_OPERATION8";
  tcpSocket->write(QString(msg).toLocal8Bit().constData());
  ui.listWidget_Server_text->addItem(QString("%1").arg(msg));
  ui.listWidget_Server_text->scrollToBottom();
}

void qtros::MainWindow::on_btnFree_Set_clicked()
{
  QString msg = "MODE_OF_OPERATION0";
  tcpSocket->write(QString(msg).toLocal8Bit().constData());
  ui.listWidget_Server_text->addItem(QString("%1").arg(msg));
  ui.listWidget_Server_text->scrollToBottom();
}

void qtros::MainWindow::on_btnSRV_ErrorClear_clicked()
{
  QString msg = "SRV_RESET_ALARM";
  tcpSocket->write(QString(msg).toLocal8Bit().constData());
  ui.listWidget_Server_text->addItem(QString("%1").arg(msg));
  ui.listWidget_Server_text->scrollToBottom();
}

void qtros::MainWindow::on_btnHomming_clicked()
{
  
  //moveit
  const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
  moveit::core::RobotStatePtr current_state = qnode.move_group->getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group,joint_group_positions);
  joint_group_positions[0] = 0*pi/180;
  joint_group_positions[1] = 0*pi/180;
  joint_group_positions[4] = 0*pi/180;
  joint_group_positions[7] = 0*pi/180;
  joint_group_positions[8] = 0*pi/180;
  joint_group_positions[9] = 0*pi/180;
  qnode.move_group->setJointValueTarget(joint_group_positions);
  qnode.move_group->move();

  //real robot
  QString msg = "Homming!";
  tcpSocket->write(QString("J10J20J30J40J50J60").toLocal8Bit().constData());
  ui.listWidget_Server_text->addItem(QString("%1").arg(msg));
  ui.listWidget_Server_text->scrollToBottom();

}

void qtros::MainWindow::on_btnSendtoRobot_clicked()
{
  //boost::thread th1(boost::bind(&Thread_point));

}

void qtros::MainWindow::on_btntest_clicked() //here test
{
    //boost::thread th3(boost::bind(&MainWindow::task_moveit_funtion, this));
  _task_moveit_test();
}


void qtros::MainWindow::test_move_func()
{
   
}


void qtros::MainWindow::on_btn_test_move_clicked() //here test
{

    //boost::thread th3(&MainWindow::_190714_trakint_test, this);
    boost::thread th3(&MainWindow::_task_function, this);
    //_task_function();
}



void qtros::MainWindow::on_btnDetectPose_clicked() //here2
{
    geometry_msgs::Pose target_pose;
    target_pose.position.x = ui.txt_x_data_1->text().trimmed().toDouble();
    target_pose.position.y = ui.txt_y_data_1->text().trimmed().toDouble();
    target_pose.position.z = ui.txt_z_data_1->text().trimmed().toDouble();
    double rotate_x = ui.txt_R_data_1->text().trimmed().toDouble()*pi/180;
    double rotate_y = ui.txt_P_data_1->text().trimmed().toDouble()*pi/180;
    double rotate_z = ui.txt_yaw_data_1->text().trimmed().toDouble()*pi/180;

    tf::Quaternion q;
    q.setRPY(rotate_x,rotate_y,rotate_z);
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();
    const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

    found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms

    if (found_ik)
    {
      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
      double arr[9];
      for (std::size_t i = 0; i < joint_names.size(); ++i)
      {

        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      }
      //std::copy(joint_values.begin(),joint_values.end(),arr);



      Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

      qnode.move_group->setPoseTarget(target_pose);
      qnode.move_group->move();

      QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);
      //QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(qnode.joint_values[0]).arg(qnode.joint_values[1])
      //    .arg(qnode.joint_values[4]).arg(qnode.joint_values[7]).arg(qnode.joint_values[8]).arg(qnode.joint_values[9]);
      ROS_INFO("joint1: %f",Moveit_Jointvalue_1);
      ROS_INFO("joint2: %f",Moveit_Jointvalue_2);
      ROS_INFO("joint3: %f",Moveit_Jointvalue_3);
      ROS_INFO("joint4: %f",Moveit_Jointvalue_4);
      ROS_INFO("joint5: %f",Moveit_Jointvalue_5);
      ROS_INFO("joint6: %f",Moveit_Jointvalue_6);
      QString msg = "Moving!";
      tcpSocket->write(Joint_data.toLocal8Bit().constData());
      ui.listWidget_Server_text->addItem(QString("%1").arg(Joint_data));
      ui.listWidget_Server_text->scrollToBottom();

      usleep(2000000);

      complete_pose = false;

    }
    else
    {
      ROS_INFO("Did not find IK solution");
    }

}

void qtros::MainWindow::inverse_and_move(double position_x,double position_y,double position_z,double orientation_r,double orientation_p,double orientation_y)
{
    geometry_msgs::Pose target_pose;
    target_pose.position.x = position_x;
    target_pose.position.y = position_y;
    target_pose.position.z = position_z;
    double rotate_x = orientation_r*pi/180;
    double rotate_y = orientation_p*pi/180;
    double rotate_z = orientation_y*pi/180;

    tf::Quaternion q;
    q.setRPY(rotate_x,rotate_y,rotate_z);
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();

    const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

    found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms

    if (found_ik)
    {
      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
      double arr[9];
      for (std::size_t i = 0; i < joint_names.size(); ++i)
      {

        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      }
      //std::copy(joint_values.begin(),joint_values.end(),arr);



      Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

      qnode.move_group->setPoseTarget(target_pose);
   //   qnode.move_group->move();
      QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);
      //QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(qnode.joint_values[0]).arg(qnode.joint_values[1])
      //    .arg(qnode.joint_values[4]).arg(qnode.joint_values[7]).arg(qnode.joint_values[8]).arg(qnode.joint_values[9]);
      ROS_INFO("joint1: %f",Moveit_Jointvalue_1);
      ROS_INFO("joint2: %f",Moveit_Jointvalue_2);
      ROS_INFO("joint3: %f",Moveit_Jointvalue_3);
      ROS_INFO("joint4: %f",Moveit_Jointvalue_4);
      ROS_INFO("joint5: %f",Moveit_Jointvalue_5);
      ROS_INFO("joint6: %f",Moveit_Jointvalue_6);
      QString msg = "Moving!";
      tcpSocket->write(Joint_data.toLocal8Bit().constData());
      ui.listWidget_Server_text->addItem(QString("%1").arg(Joint_data));
      ui.listWidget_Server_text->scrollToBottom();

      usleep(2000000);

      complete_pose = false;

    }
    else
    {
      ROS_INFO("Did not find IK solution");
    }

}

void qtros::MainWindow::on_btnEndToCameraCoordinate_clicked()
{
  geometry_msgs::Pose target_pose;

  target_pose.position.x = ui.txtsetEndValue_x->text().trimmed().toDouble();
  target_pose.position.y = ui.txtsetEndValue_y->text().trimmed().toDouble();
  target_pose.position.z = ui.txtsetEndValue_z->text().trimmed().toDouble();
  double rotate_x = ui.txtsetEndValue_roll->text().trimmed().toDouble()*pi/180;
  double rotate_y = ui.txtsetEndValue_pitch->text().trimmed().toDouble()*pi/180;
  double rotate_z = ui.txtsetEndValue_yaw->text().trimmed().toDouble()*pi/180;


  tf::Quaternion q;
  q.setRPY(rotate_x,rotate_y,rotate_z);
  target_pose.orientation.x = q.x();
  target_pose.orientation.y = q.y();
  target_pose.orientation.z = q.z();
  target_pose.orientation.w = q.w();
  const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
  const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

  found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms

  if (found_ik)
  {
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    double arr[9];
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {

      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
    //std::copy(joint_values.begin(),joint_values.end(),arr);

    Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
    Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
    Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
    Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
    Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
    Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

    qnode.move_group->setPoseTarget(target_pose);
    qnode.move_group->move();
    QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);
    //QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(qnode.joint_values[0]).arg(qnode.joint_values[1])
    //    .arg(qnode.joint_values[4]).arg(qnode.joint_values[7]).arg(qnode.joint_values[8]).arg(qnode.joint_values[9]);
    ROS_INFO("joint1: %f",Moveit_Jointvalue_1);
    ROS_INFO("joint2: %f",Moveit_Jointvalue_2);
    ROS_INFO("joint3: %f",Moveit_Jointvalue_3);
    ROS_INFO("joint4: %f",Moveit_Jointvalue_4);
    ROS_INFO("joint5: %f",Moveit_Jointvalue_5);
    ROS_INFO("joint6: %f",Moveit_Jointvalue_6);
    QString msg = "Moving!";
    tcpSocket->write(Joint_data.toLocal8Bit().constData());
    ui.listWidget_Server_text->addItem(QString("%1").arg(Joint_data));
    ui.listWidget_Server_text->scrollToBottom();


  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }


}

//not used
double qtros::MainWindow::HomogeneousTransformationMatrix(double theta, double alpha, double ai, double di)
{

  double Ai[4][4];
  Ai[1][1] = cos(theta);
  Ai[1][2] = -sin(theta)*cos(alpha);
  Ai[1][3] = sin(theta)*sin(alpha);
  Ai[1][4] = ai*cos(theta);
  Ai[2][1] = sin(theta);
  Ai[2][2] = cos(theta)*sin(alpha);
  Ai[2][3] = -cos(theta)*sin(alpha);
  Ai[2][4] = ai*sin(theta);
  Ai[3][1] = 0;
  Ai[3][2] = sin(alpha);
  Ai[3][3] = cos(alpha);
  Ai[3][4] = di;
  Ai[4][1] = 0;
  Ai[4][2] = 0;
  Ai[4][3] = 0;
  Ai[4][4] = 1;

  return Ai[4][4];

}



void qtros::MainWindow::on_btnMoveToPixelPoint_clicked()
{
  //boost::thread th1(boost::bind(&MainWindow::Thread_point, this));
  // Original scenario
  //boost::thread th1(&MainWindow::Thread_point, this);
  //190603 test
  //boost::thread th1(&MainWindow::pick_and_place_task, this);
  boost::thread th1(&MainWindow::Task_pick_and_place, this);

  //th1.join();
}

geometry_msgs::Vector3 qtros::MainWindow::Quaternion_to_RPY(geometry_msgs::PoseStamped current_pose, tf::Quaternion quat)
{

    tf::quaternionMsgToTF( current_pose.pose.orientation, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // the found angles are written in a geometry_msgs::Vector3
    geometry_msgs::Vector3 rpy;
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;

    return rpy;

}

geometry_msgs::Vector3 qtros::MainWindow::Quaternion_to_RPY()
{
    // Get pose & orientation
    qnode.move_group->setEndEffectorLink("gripper");
    geometry_msgs::PoseStamped current_pose = qnode.move_group->getCurrentPose();

    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;

    tf::quaternionMsgToTF( current_pose.pose.orientation, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // the found angles are written in a geometry_msgs::Vector3
    geometry_msgs::Vector3 rpy;
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;

    return rpy;

}

tf::Quaternion qtros::MainWindow::RPY_to_Quaternion(double rotate_x, double rotate_y, double rotate_z)
{
    tf::Quaternion q;
    q.setRPY(rotate_x,rotate_y,rotate_z);

    return q;
}

void qtros::MainWindow::SonaMalding_Start()
{
    QString msg = "SonaMalding Start!";
    tcpSocket->write(QString("SonaMalding_Start\n").toLocal8Bit().constData());
    ui.listWidget_Server_text->addItem(QString("%1").arg(msg));
    ui.listWidget_Server_text->scrollToBottom();
}


void qtros::MainWindow::SonaMalding_Stop()
{
    QString msg = "SonaMalding Stop!";
    tcpSocket->write(QString("SonaMalding_Stop\n").toLocal8Bit().constData());
    ui.listWidget_Server_text->addItem(QString("%1").arg(msg));
    ui.listWidget_Server_text->scrollToBottom();
}


void qtros::MainWindow::Gripper_Open()
{
    QString msg = "GPIO ON!";
    tcpSocket->write(QString("GPIO_OUT_ON\n").toLocal8Bit().constData());
    ui.listWidget_Server_text->addItem(QString("%1").arg(msg));
    ui.listWidget_Server_text->scrollToBottom();
}

void qtros::MainWindow::Gripper_Close()
{
    QString msg = "GPIO OFF!";
    tcpSocket->write(QString("GPIO_OUT_OFF\n").toLocal8Bit().constData());
    ui.listWidget_Server_text->addItem(QString("%1").arg(msg));
    ui.listWidget_Server_text->scrollToBottom();
}


void qtros::MainWindow::joint_and_move(double joint_1, double joint_2, double joint_3, double joint_4, double joint_5, double joint_6)
{
    const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
    moveit::core::RobotStatePtr current_state = qnode.move_group->getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group,joint_group_positions);
    //ready to grip
    joint_group_positions[0] = joint_1*pi/180;
    joint_group_positions[1] = joint_2*pi/180;
    joint_group_positions[4] = joint_3*pi/180;
    joint_group_positions[7] = joint_4*pi/180;
    joint_group_positions[8] = joint_5*pi/180;
    joint_group_positions[9] = joint_6*pi/180;
    //pose: 666.942,14.1035,311.216
    //orientation:    2.04496,-1.56982,-2.04517

    qnode.move_group->setJointValueTarget(joint_group_positions);
    qnode.move_group->move();

     QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(joint_group_positions[0]*180/pi).arg(joint_group_positions[1]*180/pi).arg(joint_group_positions[4]*180/pi)
         .arg(joint_group_positions[7]*180/pi).arg(joint_group_positions[8]*180/pi).arg(joint_group_positions[9]*180/pi);
    tcpSocket->write(Joint_data.toLocal8Bit().constData());
}

void qtros::MainWindow::on_btn_fk_clicked()
{

    // Get pose & orientation
    qnode.move_group->setEndEffectorLink("gripper");
    geometry_msgs::PoseStamped current_pose =    qnode.move_group->getCurrentPose();

    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;

    //////////////
    tf::quaternionMsgToTF( current_pose.pose.orientation, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // the found angles are written in a geometry_msgs::Vector3
    geometry_msgs::Vector3 rpy;
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;

    /////////////////

    ROS_INFO_STREAM(current_pose.pose.position.x <<"," << current_pose.pose.position.y <<"," << current_pose.pose.position.z <<"\n" <<
                    rpy.x<<"," << rpy.y<<"," << rpy.z);

}

void qtros::MainWindow::pick_and_place_task()
{
    complete_pose = false;

    // 1. Ready to grip the enclosure
    const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
    moveit::core::RobotStatePtr current_state = qnode.move_group->getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group,joint_group_positions);
    //ready to grip
    joint_group_positions[0] = -69.6471*pi/180;
    joint_group_positions[1] = -3.4751*pi/180;
    joint_group_positions[4] = -95.536*pi/180;
    joint_group_positions[7] = 81.0136*pi/180;
    joint_group_positions[8] = -69.7951*pi/180;
    joint_group_positions[9] = 179.813*pi/180;
    //pose: 666.942,14.1035,311.216
    //orientation:    2.04496,-1.56982,-2.04517

    qnode.move_group->setJointValueTarget(joint_group_positions);
    qnode.move_group->move();

     QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(joint_group_positions[0]*180/pi).arg(joint_group_positions[1]*180/pi).arg(joint_group_positions[4]*180/pi)
         .arg(joint_group_positions[7]*180/pi).arg(joint_group_positions[8]*180/pi).arg(joint_group_positions[9]*180/pi);
    tcpSocket->write(Joint_data.toLocal8Bit().constData());

    // Get pose & orientation
    qnode.move_group->setEndEffectorLink("gripper");
    geometry_msgs::PoseStamped current_pose =    qnode.move_group->getCurrentPose();

    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;

    //////////////
    tf::quaternionMsgToTF( current_pose.pose.orientation, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // the found angles are written in a geometry_msgs::Vector3
    geometry_msgs::Vector3 rpy;
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;

    /////////////////


    ROS_INFO_STREAM(current_pose.pose.position.x <<"," << current_pose.pose.position.y <<"," << current_pose.pose.position.z <<"\n" <<
                    rpy.x<<"," << rpy.y<<"," << rpy.z);

    while(!complete_pose){};
    complete_pose = false;
    usleep(5000000);

    // 2. Gripping the enclosure
    /*
    QString msg = "GPIO OFF!";
    tcpSocket->write(QString("GPIO_OUT_OFF\n").toLocal8Bit().constData());//send a GPIO msg using tcp-ip
    ui.listWidget_Server_text->addItem(QString("%1").arg(msg));
    ui.listWidget_Server_text->scrollToBottom();
    */
    Gripper_Close();
    usleep(2000000);

    // 3. Waiting for ultrasonic welder

    current_state->copyJointGroupPositions(joint_model_group,joint_group_positions);
    joint_group_positions[0] = -150.281*pi/180;
    joint_group_positions[1] = 25.0644*pi/180;
    joint_group_positions[4] = -120.342*pi/180;
    joint_group_positions[7] = 84.7893*pi/180;
    joint_group_positions[8] = -62.454*pi/180;
    joint_group_positions[9] = 179.84*pi/180;
    qnode.move_group->setJointValueTarget(joint_group_positions);
    qnode.move_group->move();

    Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(joint_group_positions[0]*180/pi).arg(joint_group_positions[1]*180/pi).arg(joint_group_positions[4]*180/pi)
         .arg(joint_group_positions[7]*180/pi).arg(joint_group_positions[8]*180/pi).arg(joint_group_positions[9]*180/pi);
    tcpSocket->write(Joint_data.toLocal8Bit().constData());


    //inverse_and_move(0,-500,300,117,-90,155);

    //-150.281, 25.0644, -120.342, 84.7893, -62.454, 179.84
    // Get pose & orientation
    current_pose =    qnode.move_group->getCurrentPose();

    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::quaternionMsgToTF( current_pose.pose.orientation, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // the found angles are written in a geometry_msgs::Vector3
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;

    ROS_INFO_STREAM(current_pose.pose.position.x <<"," << current_pose.pose.position.y <<"," << current_pose.pose.position.z <<"\n" <<
                    rpy.x<<"," << rpy.y<<"," << rpy.z);
    //position -52.8897,-664.971,311.228
    //orientation: 2.11853,-1.56981,2.49315

    while(!complete_pose){};
    complete_pose = false;
    usleep(2000000);



    // 4. Concrete motion(insertion) & waiting for welder process
    current_state->copyJointGroupPositions(joint_model_group,joint_group_positions);
    joint_group_positions[0] = -163.576*pi/180;
    joint_group_positions[1] = -8.83349*pi/180;
    joint_group_positions[4] = -90.96*pi/180;
    joint_group_positions[7] = 79.6553*pi/180;
    joint_group_positions[8] = -76.0218*pi/180;
    joint_group_positions[9] = 179.29*pi/180;
    qnode.move_group->setJointValueTarget(joint_group_positions);
    qnode.move_group->move();

    Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(joint_group_positions[0]*180/pi).arg(joint_group_positions[1]*180/pi).arg(joint_group_positions[4]*180/pi)
         .arg(joint_group_positions[7]*180/pi).arg(joint_group_positions[8]*180/pi).arg(joint_group_positions[9]*180/pi);
    tcpSocket->write(Joint_data.toLocal8Bit().constData());

    /*
    current_state->copyJointGroupPositions(joint_model_group,joint_group_positions);
    joint_group_positions[0] = -165.4054*pi/180;
    joint_group_positions[1] = -3.4751*pi/180;
    joint_group_positions[4] = -95.536*pi/180;
    joint_group_positions[7] = 81.0136*pi/180;
    joint_group_positions[8] = -40.7951*pi/180;
    joint_group_positions[9] = 179.813*pi/180;
    qnode.move_group->setJointValueTarget(joint_group_positions);
    qnode.move_group->move();

    Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(joint_group_positions[0]*180/pi).arg(joint_group_positions[1]*180/pi).arg(joint_group_positions[4]*180/pi)
         .arg(joint_group_positions[7]*180/pi).arg(joint_group_positions[8]*180/pi).arg(joint_group_positions[9]*180/pi);
    tcpSocket->write(Joint_data.toLocal8Bit().constData());

    */

    while(!complete_pose){};
    complete_pose = false;
    usleep(2000000);

    Gripper_Open();
    /*
    msg = "GPIO ON!";
    tcpSocket->write(QString("GPIO_OUT_ON\n").toLocal8Bit().constData());//send a GPIO msg using tcp-ip
    ui.listWidget_Server_text->addItem(QString("%1").arg(msg));
    ui.listWidget_Server_text->scrollToBottom();
    */
    usleep(2000000);


    current_state->copyJointGroupPositions(joint_model_group,joint_group_positions);
    joint_group_positions[0] = -150.281*pi/180;
    joint_group_positions[1] = 25.0644*pi/180;
    joint_group_positions[4] = -120.342*pi/180;
    joint_group_positions[7] = 84.7893*pi/180;
    joint_group_positions[8] = -62.454*pi/180;
    joint_group_positions[9] = 179.84*pi/180;
    qnode.move_group->setJointValueTarget(joint_group_positions);
    qnode.move_group->move();

    Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(joint_group_positions[0]*180/pi).arg(joint_group_positions[1]*180/pi).arg(joint_group_positions[4]*180/pi)
         .arg(joint_group_positions[7]*180/pi).arg(joint_group_positions[8]*180/pi).arg(joint_group_positions[9]*180/pi);
    tcpSocket->write(Joint_data.toLocal8Bit().constData());


    while(!complete_pose){};
    complete_pose = false;
    usleep(10000000);

    current_state->copyJointGroupPositions(joint_model_group,joint_group_positions);
    joint_group_positions[0] = -163.576*pi/180;
    joint_group_positions[1] = -8.83349*pi/180;
    joint_group_positions[4] = -90.96*pi/180;
    joint_group_positions[7] = 79.6553*pi/180;
    joint_group_positions[8] = -76.0218*pi/180;
    joint_group_positions[9] = 179.29*pi/180;
    qnode.move_group->setJointValueTarget(joint_group_positions);
    qnode.move_group->move();

    Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(joint_group_positions[0]*180/pi).arg(joint_group_positions[1]*180/pi).arg(joint_group_positions[4]*180/pi)
         .arg(joint_group_positions[7]*180/pi).arg(joint_group_positions[8]*180/pi).arg(joint_group_positions[9]*180/pi);
    tcpSocket->write(Joint_data.toLocal8Bit().constData());
    //inverse_and_move(0,-700,300,117,-90,155);
    //-163.576, -8.83349, -90.96, 79.6553, -76.0218, 179.29

    while(!complete_pose){};
    complete_pose = false;
    usleep(2000000);
    /*
    msg = "GPIO OFF!";
    tcpSocket->write(QString("GPIO_OUT_OFF\n").toLocal8Bit().constData());//send a GPIO msg using tcp-ip
    ui.listWidget_Server_text->addItem(QString("%1").arg(msg));
    ui.listWidget_Server_text->scrollToBottom();
    */
    Gripper_Close();
    usleep(2000000);

    current_state->copyJointGroupPositions(joint_model_group,joint_group_positions);
    joint_group_positions[0] = -150.281*pi/180;
    joint_group_positions[1] = 25.0644*pi/180;
    joint_group_positions[4] = -120.342*pi/180;
    joint_group_positions[7] = 84.7893*pi/180;
    joint_group_positions[8] = -62.454*pi/180;
    joint_group_positions[9] = 179.84*pi/180;
    qnode.move_group->setJointValueTarget(joint_group_positions);
    qnode.move_group->move();

    Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(joint_group_positions[0]*180/pi).arg(joint_group_positions[1]*180/pi).arg(joint_group_positions[4]*180/pi)
         .arg(joint_group_positions[7]*180/pi).arg(joint_group_positions[8]*180/pi).arg(joint_group_positions[9]*180/pi);
    tcpSocket->write(Joint_data.toLocal8Bit().constData());

    while(!complete_pose){};
    complete_pose = false;
    usleep(2000000);

    // 5. Remove to conveyer belt
    current_state->copyJointGroupPositions(joint_model_group,joint_group_positions);
    joint_group_positions[0] = -69.6471*pi/180;
    joint_group_positions[1] = -3.4751*pi/180;
    joint_group_positions[4] = -95.536*pi/180;
    joint_group_positions[7] = 81.0136*pi/180;
    joint_group_positions[8] = -69.7951*pi/180;
    joint_group_positions[9] = 179.813*pi/180;
    qnode.move_group->setJointValueTarget(joint_group_positions);
    qnode.move_group->move();

     Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(joint_group_positions[0]*180/pi).arg(joint_group_positions[1]*180/pi).arg(joint_group_positions[4]*180/pi)
         .arg(joint_group_positions[7]*180/pi).arg(joint_group_positions[8]*180/pi).arg(joint_group_positions[9]*180/pi);
    tcpSocket->write(Joint_data.toLocal8Bit().constData());


    while(!complete_pose){};

    complete_pose = false;
    usleep(5000000);

    // 5. Grip on
    /*
    msg = "GPIO ON!";
    tcpSocket->write(QString("GPIO_OUT_ON\n").toLocal8Bit().constData());//send a GPIO msg using tcp-ip
    ui.listWidget_Server_text->addItem(QString("%1").arg(msg));
    ui.listWidget_Server_text->scrollToBottom();
    */
    Gripper_Open();

    usleep(2000000);

        /*
         *
    geometry_msgs::Pose target_pose;
    target_pose.position.x = ui.txt_x_data_1->text().trimmed().toDouble();
    target_pose.position.y = ui.txt_y_data_1->text().trimmed().toDouble();
    target_pose.position.z = ui.txt_z_data_1->text().trimmed().toDouble();
    double rotate_x = ui.txt_R_data_1->text().trimmed().toDouble()*pi/180;
    double rotate_y = ui.txt_P_data_1->text().trimmed().toDouble()*pi/180;
    double rotate_z = ui.txt_yaw_data_1->text().trimmed().toDouble()*pi/180;

    tf::Quaternion q;
    q.setRPY(rotate_x,rotate_y,rotate_z);
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();
    const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

    found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms

    if (found_ik)
    {
      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
      double arr[9];
      for (std::size_t i = 0; i < joint_names.size(); ++i)
      {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      }

      Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

      qnode.move_group->setPoseTarget(target_pose);
      QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);

      ROS_INFO("joint1: %f",Moveit_Jointvalue_1);
      ROS_INFO("joint2: %f",Moveit_Jointvalue_2);
      ROS_INFO("joint3: %f",Moveit_Jointvalue_3);
      ROS_INFO("joint4: %f",Moveit_Jointvalue_4);
      ROS_INFO("joint5: %f",Moveit_Jointvalue_5);
      ROS_INFO("joint6: %f",Moveit_Jointvalue_6);
      QString msg = "Moving!";
      tcpSocket->write(Joint_data.toLocal8Bit().constData());
      ui.listWidget_Server_text->addItem(QString("%1").arg(Joint_data));
      ui.listWidget_Server_text->scrollToBottom();
    }*/

}
//main code ( task )
void qtros::MainWindow::Thread_point()
{
  bool grip = false; //only this code make grip false
  inv_count = 0;

  while(1)// infinite loop // if grip on -> there is no code in while(1)
  {
      if(grip == true)
      {
          grip_value = 1;
      }
      else
      {
          grip_value = 0;
      }

      //ROS_INFO("grip value : %d", grip_value);

    if(complete_pose && grip != true) //complete_pose mean is motion complete(robot_hp_pc_tcp-ip data), grip
    {
      ROS_INFO("complete_pose = true -> start");

      if(qnode._detectEnableGripper == 1) // gripper on/off check //***_detectEnableGripper mean is important
      {
        //??? grip enable : true -> gripper open ?????
        ROS_INFO("grip_enable = true -> gripper open");

        QString msg = "GPIO ON!";
        tcpSocket->write(QString("GPIO_OUT_ON\n").toLocal8Bit().constData());//send a GPIO msg using tcp-ip
        ui.listWidget_Server_text->addItem(QString("%1").arg(msg));
        ui.listWidget_Server_text->scrollToBottom();
        usleep(10);

        grip = true;
      }
      else// if gripper off
      {
        ROS_INFO("grip_enable = false -> gripper close");

        QString msg = "GPIO OFF!";
        tcpSocket->write(QString("GPIO_OUT_OFF\n").toLocal8Bit().constData());//send a GPIO msg using tcp-ip
        ui.listWidget_Server_text->addItem(QString("%1").arg(msg));
        ui.listWidget_Server_text->scrollToBottom();
        usleep(10);
      }

      if(grip != true) // if grip off -> run task
      {

        ROS_INFO("STEP : move_arm");

        geometry_msgs::Pose target_pose;
        target_pose.position.x = qnode._detectPixelObject_x;
        target_pose.position.y = qnode._detectPixelObject_y;
        if(qnode._detectEnabledepth == 1 && qnode._detectPixelObject_y < 20)//detectEnabledepth???, when detectPixelObject_y is within a range.
        {

          ROS_INFO("STEP : Z down");

          target_pose.position.z = 135; // grip point_z
          qnode._detectPixelObject_z = 135;
        }
        else
        {
          ROS_INFO("STEP : Z up");

          target_pose.position.z = 200; // grip waiting point_z
          qnode._detectPixelObject_z = 200;
        }

        double rotate_x = -90*pi/180;
        double rotate_y = -90*pi/180;
        double rotate_z =  90*pi/180;
        tf::Quaternion q;
        q.setRPY(rotate_x,rotate_y,rotate_z);
        target_pose.orientation.x = q.x();
        target_pose.orientation.y = q.y();
        target_pose.orientation.z = q.z();
        target_pose.orientation.w = q.w();

        //inverse kinematics
        const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
        robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));

        moveit_msgs::JointConstraint joint_3;
        joint_3.joint_name = "joint3";
        joint_3.position = 0;
        joint_3.tolerance_above = 0;
        joint_3.tolerance_below = -3.14;
        moveit_msgs::JointConstraint joint_4;
        joint_4.joint_name = "joint4";
        joint_4.position = 0;
        joint_4.tolerance_above = 3.14;
        joint_4.tolerance_below = 0;
        moveit_msgs::Constraints path_constraints;
        path_constraints.joint_constraints.push_back(joint_4); //?
        qnode.move_group->setPathConstraints(path_constraints);

        //inverse kinematics
        const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
        found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.01s

        if (found_ik) // ik found
        {
          kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
          for (std::size_t i = 0; i < joint_names.size(); ++i)
          {
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
          }
          //std::copy(joint_values.begin(),joint_values.end(),arr);

          Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

          qnode.move_group->setPoseTarget(target_pose);
         // qnode.move_group->move();
          QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6\n").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);
          QString msg = "Moving!";
          tcpSocket->write(Joint_data.toLocal8Bit().constData());
          ui.listWidget_Server_text->addItem(QString("%1").arg(Joint_data));
          ui.listWidget_Server_text->scrollToBottom();

          inv_count++;

          ROS_INFO("inverse count : %d", inv_count);
        }

        else
        {
          ROS_INFO("Do not found IK!");
        }
          // make file
      }

      else // move to a specific position after gripping.
      {
        ROS_INFO("grip = true -> move to a specific position after gripping.");

        usleep(1000000);


        const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
        moveit::core::RobotStatePtr current_state = qnode.move_group->getCurrentState();
        std::vector<double> joint_group_positions;
        current_state->copyJointGroupPositions(joint_model_group,joint_group_positions);
        //ready to grip
        joint_group_positions[0] = -69.6471*pi/180;
        joint_group_positions[1] = -3.4751*pi/180;
        joint_group_positions[4] = -95.536*pi/180;
        joint_group_positions[7] = 81.0136*pi/180;
        joint_group_positions[8] = -69.7951*pi/180;
        joint_group_positions[9] = 179.813*pi/180;
        qnode.move_group->setJointValueTarget(joint_group_positions);
        qnode.move_group->move();

         QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(joint_group_positions[0]*180/pi).arg(joint_group_positions[1]*180/pi).arg(joint_group_positions[4]*180/pi)
             .arg(joint_group_positions[7]*180/pi).arg(joint_group_positions[8]*180/pi).arg(joint_group_positions[9]*180/pi);
        tcpSocket->write(Joint_data.toLocal8Bit().constData());

      }

      QFile file("RobotJoint_v2.txt"); // data save (used check)
      if(!file.open(QIODevice::WriteOnly | QIODevice::Text | QFile::Append))
        return;
      QTextStream out(&file);
      QString Data = QString("%1,%2,%3,%4,%5,%6,%7,%8,%9,%10,%11").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4)
                                                   .arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6).arg(qnode._detectPixelObject_x).arg(qnode._detectPixelObject_y).arg(qnode._detectPixelObject_z)
                                                   .arg(qnode._detectpixel_u).arg(qnode._detectpixel_v);
      QString time = QDateTime::currentDateTime().time().toString("hh:mm:ss.zzz");
      out << Data + "," + time <<"\n";
      complete_pose = false;
    }
  }
}


void qtros::MainWindow::on_btnGPIO_OFF_clicked()
{
  Gripper_Close();
}

void qtros::MainWindow::on_btnGPIO_ON_clicked()
{
  Gripper_Open();
}


void qtros::MainWindow::Check_pixel()//run only once.
{
  while(1) //infinite
  {
    QFile file("camshift_Kalman_pixel.txt");
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text | QFile::Append))
      return;
    QTextStream out(&file);

    //detectPixelObject_x, detectPixelObject_y, detectNKalmanObject_x, detectNKalmanObject_y data input
    QString Data = QString("%1,%2,%3,%4").arg(qnode._detectPixelObject_x).arg(qnode._detectPixelObject_y).arg(qnode._detectNKalmanObject_x).arg(qnode._detectNKalmanObject_y);

    //current time input
    QString time = QDateTime::currentDateTime().time().toString("hh:mm:ss.zzz");

    int time2 = QDateTime::currentDateTime().time().msec();
    QString time2_msec = QString("%1").arg(time2);

    out << Data + "," + time <<"\n";
    usleep(100000);
  }

}

void qtros::MainWindow::on_btnpixel_check_clicked()
{
  boost::thread th2(&MainWindow::Check_pixel, this);

  if(check_pixel_enable) //default is true but not used elsewhere. run only once.
  {
    check_pixel_enable = false;
  }
  else
  {
   pthread_cancel(th2.native_handle());
  }

}


// used test
void qtros::MainWindow::on_btnRepeat_clicked()
{
  //boost::thread th3(&MainWindow::Repeat, this);
}
void qtros::MainWindow::Repeat() //here
{
  int turn = 0;
  int i = 0;
  complete_pose = false;

  while(i<1)
  {
      switch (turn) {
      case 0:
      {
          geometry_msgs::Pose target_pose;
          target_pose.position.x = 470;
          target_pose.position.y = 0;
          target_pose.position.z = 500;
          double rotate_x = 90*pi/180;
          double rotate_y = 180*pi/180;
          double rotate_z = (-90)*pi/180;

          tf::Quaternion q;
          q.setRPY(rotate_x,rotate_y,rotate_z);
          target_pose.orientation.x = q.x();
          target_pose.orientation.y = q.y();
          target_pose.orientation.z = q.z();
          target_pose.orientation.w = q.w();
          const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
          robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
          const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

          found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms

          if (found_ik)
          {
            kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
            double arr[9];
            for (std::size_t i = 0; i < joint_names.size(); ++i)
            {

              ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
            }
            //std::copy(joint_values.begin(),joint_values.end(),arr);



            Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
            Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
            Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
            Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
            Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
            Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

            qnode.move_group->setPoseTarget(target_pose);
         //   qnode.move_group->move();
            QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);
            //QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(qnode.joint_values[0]).arg(qnode.joint_values[1])
            //    .arg(qnode.joint_values[4]).arg(qnode.joint_values[7]).arg(qnode.joint_values[8]).arg(qnode.joint_values[9]);
            ROS_INFO("joint1: %f",Moveit_Jointvalue_1);
            ROS_INFO("joint2: %f",Moveit_Jointvalue_2);
            ROS_INFO("joint3: %f",Moveit_Jointvalue_3);
            ROS_INFO("joint4: %f",Moveit_Jointvalue_4);
            ROS_INFO("joint5: %f",Moveit_Jointvalue_5);
            ROS_INFO("joint6: %f",Moveit_Jointvalue_6);
            QString msg = "Moving!";
            tcpSocket->write(Joint_data.toLocal8Bit().constData());
            ui.listWidget_Server_text->addItem(QString("%1").arg(Joint_data));
            ui.listWidget_Server_text->scrollToBottom();

            while(true)
            {
             if(complete_pose)
                break;
            }

            ros::Duration(5).sleep();
            complete_pose = false;
            turn = 1;

          }
          else
          {
            ROS_INFO("Did not find IK solution");
          }

          break;
        }


      case 1:
      {
        geometry_msgs::Pose target_pose;
        target_pose.position.x = 350;
        target_pose.position.y = 120;
        target_pose.position.z = 380;
        double rotate_x = 90*pi/180;
        double rotate_y = 180*pi/180;
        double rotate_z = (-90)*pi/180;


        tf::Quaternion q;
        q.setRPY(rotate_x,rotate_y,rotate_z);
        target_pose.orientation.x = q.x();
        target_pose.orientation.y = q.y();
        target_pose.orientation.z = q.z();
        target_pose.orientation.w = q.w();
        const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
        robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
        const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

        found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms

        if (found_ik)
        {
          kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
          double arr[9];
          for (std::size_t i = 0; i < joint_names.size(); ++i)
          {

            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
          }
          //std::copy(joint_values.begin(),joint_values.end(),arr);



          Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

          qnode.move_group->setPoseTarget(target_pose);
       //   qnode.move_group->move();
          QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);
          //QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(qnode.joint_values[0]).arg(qnode.joint_values[1])
          //    .arg(qnode.joint_values[4]).arg(qnode.joint_values[7]).arg(qnode.joint_values[8]).arg(qnode.joint_values[9]);
          ROS_INFO("joint1: %f",Moveit_Jointvalue_1);
          ROS_INFO("joint2: %f",Moveit_Jointvalue_2);
          ROS_INFO("joint3: %f",Moveit_Jointvalue_3);
          ROS_INFO("joint4: %f",Moveit_Jointvalue_4);
          ROS_INFO("joint5: %f",Moveit_Jointvalue_5);
          ROS_INFO("joint6: %f",Moveit_Jointvalue_6);
          QString msg = "Moving!";
          tcpSocket->write(Joint_data.toLocal8Bit().constData());
          ui.listWidget_Server_text->addItem(QString("%1").arg(Joint_data));
          ui.listWidget_Server_text->scrollToBottom();

          while(true)
          {
           if(complete_pose)
              break;
          }

          ros::Duration(5).sleep();
          complete_pose = false;
          turn = 2;

        }
        else
        {
          ROS_INFO("Did not find IK solution");
        }

        break;
      }

      case 2:
      {
        geometry_msgs::Pose target_pose;
        target_pose.position.x = 590;
        target_pose.position.y = 120;
        target_pose.position.z = 620;
        double rotate_x = 90*pi/180;
        double rotate_y = 180*pi/180;
        double rotate_z = (-90)*pi/180;


        tf::Quaternion q;
        q.setRPY(rotate_x,rotate_y,rotate_z);
        target_pose.orientation.x = q.x();
        target_pose.orientation.y = q.y();
        target_pose.orientation.z = q.z();
        target_pose.orientation.w = q.w();
        const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
        robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
        const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

        found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms

        if (found_ik)
        {
          kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
          double arr[9];
          for (std::size_t i = 0; i < joint_names.size(); ++i)
          {

            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
          }
          //std::copy(joint_values.begin(),joint_values.end(),arr);



          Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

          qnode.move_group->setPoseTarget(target_pose);
        //  qnode.move_group->move();
          QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);
          //QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(qnode.joint_values[0]).arg(qnode.joint_values[1])
          //    .arg(qnode.joint_values[4]).arg(qnode.joint_values[7]).arg(qnode.joint_values[8]).arg(qnode.joint_values[9]);
          ROS_INFO("joint1: %f",Moveit_Jointvalue_1);
          ROS_INFO("joint2: %f",Moveit_Jointvalue_2);
          ROS_INFO("joint3: %f",Moveit_Jointvalue_3);
          ROS_INFO("joint4: %f",Moveit_Jointvalue_4);
          ROS_INFO("joint5: %f",Moveit_Jointvalue_5);
          ROS_INFO("joint6: %f",Moveit_Jointvalue_6);
          QString msg = "Moving!";
          tcpSocket->write(Joint_data.toLocal8Bit().constData());
          ui.listWidget_Server_text->addItem(QString("%1").arg(Joint_data));
          ui.listWidget_Server_text->scrollToBottom();

          while(true)
          {
           if(complete_pose)
              break;
          }

          ros::Duration(5).sleep();
          complete_pose = false;
          turn = 3;

        }
        else
        {
          ROS_INFO("Did not find IK solution");
        }

        break;
      }

      case 3:
      {
        geometry_msgs::Pose target_pose;
        target_pose.position.x = 590;
        target_pose.position.y = -120;
        target_pose.position.z = 620;
        double rotate_x = 90*pi/180;
        double rotate_y = 180*pi/180;
        double rotate_z = (-90)*pi/180;


        tf::Quaternion q;
        q.setRPY(rotate_x,rotate_y,rotate_z);
        target_pose.orientation.x = q.x();
        target_pose.orientation.y = q.y();
        target_pose.orientation.z = q.z();
        target_pose.orientation.w = q.w();
        const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
        robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
        const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

        found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms

        if (found_ik)
        {
          kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
          double arr[9];
          for (std::size_t i = 0; i < joint_names.size(); ++i)
          {

            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
          }
          //std::copy(joint_values.begin(),joint_values.end(),arr);



          Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

          qnode.move_group->setPoseTarget(target_pose);
       //   qnode.move_group->move();
          QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);
          //QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(qnode.joint_values[0]).arg(qnode.joint_values[1])
          //    .arg(qnode.joint_values[4]).arg(qnode.joint_values[7]).arg(qnode.joint_values[8]).arg(qnode.joint_values[9]);
          ROS_INFO("joint1: %f",Moveit_Jointvalue_1);
          ROS_INFO("joint2: %f",Moveit_Jointvalue_2);
          ROS_INFO("joint3: %f",Moveit_Jointvalue_3);
          ROS_INFO("joint4: %f",Moveit_Jointvalue_4);
          ROS_INFO("joint5: %f",Moveit_Jointvalue_5);
          ROS_INFO("joint6: %f",Moveit_Jointvalue_6);
          QString msg = "Moving!";
          tcpSocket->write(Joint_data.toLocal8Bit().constData());
          ui.listWidget_Server_text->addItem(QString("%1").arg(Joint_data));
          ui.listWidget_Server_text->scrollToBottom();

          while(true)
          {
           if(complete_pose)
              break;
          }

          ros::Duration(5).sleep();


          complete_pose = false;
          turn = 4;

        }
        else
        {
          ROS_INFO("Did not find IK solution");
        }


        break;
      }
      case 4:
      {
        geometry_msgs::Pose target_pose;
        target_pose.position.x = 350;
        target_pose.position.y = -120;
        target_pose.position.z = 380;
        double rotate_x = 90*pi/180;
        double rotate_y = 180*pi/180;
        double rotate_z = (-90)*pi/180;


        tf::Quaternion q;
        q.setRPY(rotate_x,rotate_y,rotate_z);
        target_pose.orientation.x = q.x();
        target_pose.orientation.y = q.y();
        target_pose.orientation.z = q.z();
        target_pose.orientation.w = q.w();
        const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
        robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
        const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

        found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms

        if (found_ik)
        {
          kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
          double arr[9];
          for (std::size_t i = 0; i < joint_names.size(); ++i)
          {

            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
          }
          //std::copy(joint_values.begin(),joint_values.end(),arr);



          Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

          qnode.move_group->setPoseTarget(target_pose);
       //   qnode.move_group->move();
          QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);
          //QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(qnode.joint_values[0]).arg(qnode.joint_values[1])
          //    .arg(qnode.joint_values[4]).arg(qnode.joint_values[7]).arg(qnode.joint_values[8]).arg(qnode.joint_values[9]);
          ROS_INFO("joint1: %f",Moveit_Jointvalue_1);
          ROS_INFO("joint2: %f",Moveit_Jointvalue_2);
          ROS_INFO("joint3: %f",Moveit_Jointvalue_3);
          ROS_INFO("joint4: %f",Moveit_Jointvalue_4);
          ROS_INFO("joint5: %f",Moveit_Jointvalue_5);
          ROS_INFO("joint6: %f",Moveit_Jointvalue_6);
          QString msg = "Moving!";
          tcpSocket->write(Joint_data.toLocal8Bit().constData());
          ui.listWidget_Server_text->addItem(QString("%1").arg(Joint_data));
          ui.listWidget_Server_text->scrollToBottom();

          while(true)
          {
           if(complete_pose)
              break;
          }

          ros::Duration(5).sleep();


          complete_pose = false;
          turn = 0;
          i++;
          ui.txtNumofTimes->setText(QString("%1").arg(i));





        }
        else
        {
          ROS_INFO("Did not find IK solution");
        }


        break;
      }



    }
  }
}

int tracking_count;

void qtros::MainWindow::_task_function()
{
    QFile file("/home/nscl/catkin_ws/src/jinyoung-manipulator/task_data.txt");


    int nCount = 0;
    if(!file.open(QIODevice::ReadOnly))
    {
        QMessageBox::information(0, "error", file.errorString());
    }
    else {
        QTextStream in(&file);

        while(!in.atEnd())
        {
            QString line = in.readLine();
            QStringList split_comma = line.split(",");
            std::string srr = split_comma.at(0).toUtf8().constData();
            if(srr == "P1")
            {
                std::string srr1 = split_comma.at(1).toUtf8().constData();
                double position_x = 0;
                if(srr1 == "E")
                {
                    point_enclosure_x = qnode._labeledCenter_calculate_x;
                    position_x = point_enclosure_x;
                }
                else
                {
                    position_x = split_comma.at(1).toDouble();
                }

                double position_y = split_comma.at(2).toDouble();
                double position_z = split_comma.at(3).toDouble();
                double orientation_x = split_comma.at(4).toDouble();
                double orientation_y = split_comma.at(5).toDouble();
                double orientation_z = split_comma.at(6).toDouble();

                _inverse_point(position_x, position_y, position_z, orientation_x, orientation_y, orientation_z);

            }
            else if(srr == "P2")
            {
                std::string srr1 = split_comma.at(1).toUtf8().constData();

                double position_y = split_comma.at(2).toDouble();
                double position_z = split_comma.at(3).toDouble();
                double orientation_x = split_comma.at(4).toDouble();
                double orientation_y = split_comma.at(5).toDouble();
                double orientation_z = split_comma.at(6).toDouble();
                double offset_x = split_comma.at(7).toDouble();
                double y_point = split_comma.at(8).toDouble();
                double position_x = 0;

                if(srr1 == "E")
                {
                    point_enclosure_x = qnode._labeledCenter_calculate_x + offset_x;
                    position_x = point_enclosure_x;
                }
                else
                {
                    position_x = split_comma.at(1).toDouble();
                }

                while (true) {

                    if(qnode._labeledCenter_calculate_y < y_point)
                    {
                        _inverse_point(position_x, position_y, position_z, orientation_x, orientation_y, orientation_z);
                        break;
                    }

                }
            }
            else if(srr == "P3")
            {
                std::string srr1 = split_comma.at(1).toUtf8().constData();
                double position_x;

                if(srr1 == "E")
                {
                    position_x = point_enclosure_x;
                }
                else
                {

                    position_x = split_comma.at(1).toDouble();
                }
                double position_y = split_comma.at(2).toDouble();
                double position_z = split_comma.at(3).toDouble();
                double orientation_x = split_comma.at(4).toDouble();
                double orientation_y = split_comma.at(5).toDouble();
                double orientation_z = split_comma.at(6).toDouble();
                double z_down_time = split_comma.at(6).toDouble();


                avg_velocity = 0;
                vel_check_count = 0;

                while (true) {

                    avg_velocity = avg_velocity + qnode.velocity_enclosure;
                    vel_check_count++;

                    if(qnode._labeledCenter_calculate_y < (avg_velocity/vel_check_count) * z_down_time)
                    {
                        _inverse_point(point_enclosure_x, position_y, position_z, orientation_x, orientation_y, orientation_z);

                        break;
                    }
                    Gripper_Close();
                    ros::Duration(1).sleep();

                }
            }
            else if(srr == "P")
            {
                std::string srr1 = split_comma.at(1).toUtf8().constData();
                double position_x;

                if(srr1 == "E")
                {
                    position_x = point_enclosure_x;
                }
                else
                {
                    position_x = split_comma.at(1).toDouble();
                }
                double position_y = split_comma.at(2).toDouble();
                double position_z = split_comma.at(3).toDouble();
                double orientation_x = split_comma.at(4).toDouble();
                double orientation_y = split_comma.at(5).toDouble();
                double orientation_z = split_comma.at(6).toDouble();

                _inverse_point(position_x,position_y,position_z,orientation_x,orientation_y,orientation_z);
                ros::Duration(1).sleep();
            }
            else if(srr == "G")
            {
                int grip = split_comma.at(1).toInt();

                if(grip == 0)
                {
                    ROS_INFO("Gripper_Open");
                    //Gripper_Open();
                    ros::Duration(1).sleep();
                }
                else if(grip == 1)
                {
                    ROS_INFO("Gripper_Close");
                    //Gripper_Close();
                    ros::Duration(1).sleep();
                }

            }
            else if(srr == "S")
            {
                int grip = split_comma.at(1).toInt();

                if(grip == 0)
                {
                    ROS_INFO("SonaMalding Stop");
                    //SonaMalding_Stop();
                    ros::Duration(1).sleep();
                }
                else if(grip == 1)
                {
                    ROS_INFO("SonaMalding Start");
                    //SonaMalding_Start();
                    ros::Duration(1).sleep();
                }

            }
            else if(srr == "D")
            {
                int delay = split_comma.at(1).toInt();
                usleep(delay);

            }

        }

        file.close();

    }

    /*
    QTextStream in(&file);

    QString myText = in.readAll();
    std::string srr = myText.toUtf8().constData();
    ROS_INFO("%s", srr.c_str());
    */


}

void qtros::MainWindow::_task_moveit_function()
{
    QFile file("/home/nscl/catkin_ws/src/jinyoung-manipulator/task_data.txt");


    int nCount = 0;
    if(!file.open(QIODevice::ReadOnly))
    {
        QMessageBox::information(0, "error", file.errorString());
    }
    else {
        QTextStream in(&file);

        while(!in.atEnd())
        {
            QString line = in.readLine();
            QStringList split_comma = line.split(",");
            std::string srr = split_comma.at(0).toUtf8().constData();
            if(srr == "P1")
            {
                std::string srr1 = split_comma.at(1).toUtf8().constData();
                double position_x = 0;
                if(srr1 == "E")
                {
                    point_enclosure_x = qnode._labeledCenter_calculate_x;
                    position_x = point_enclosure_x;
                }
                else
                {
                    position_x = split_comma.at(1).toDouble();
                }

                double position_y = split_comma.at(2).toDouble();
                double position_z = split_comma.at(3).toDouble();
                double orientation_x = split_comma.at(4).toDouble();
                double orientation_y = split_comma.at(5).toDouble();
                double orientation_z = split_comma.at(6).toDouble();

                _inverse_Moveit(position_x, position_y, position_z, orientation_x, orientation_y, orientation_z);

            }
            else if(srr == "P2")
            {
                std::string srr1 = split_comma.at(1).toUtf8().constData();

                double position_y = split_comma.at(2).toDouble();
                double position_z = split_comma.at(3).toDouble();
                double orientation_x = split_comma.at(4).toDouble();
                double orientation_y = split_comma.at(5).toDouble();
                double orientation_z = split_comma.at(6).toDouble();
                double offset_x = split_comma.at(7).toDouble();
                double y_point = split_comma.at(8).toDouble();
                double position_x = 0;

                if(srr1 == "E")
                {
                    point_enclosure_x = qnode._labeledCenter_calculate_x + offset_x;
                    position_x = point_enclosure_x;
                }
                else
                {
                    position_x = split_comma.at(1).toDouble();
                }

                while (true) {

                    if(qnode._labeledCenter_calculate_y < y_point)
                    {
                        _inverse_Moveit(position_x, position_y, position_z, orientation_x, orientation_y, orientation_z);
                        break;
                    }

                }
            }
            else if(srr == "P3")
            {
                std::string srr1 = split_comma.at(1).toUtf8().constData();
                double position_x;

                if(srr1 == "E")
                {
                    position_x = point_enclosure_x;
                }
                else
                {

                    position_x = split_comma.at(1).toDouble();
                }
                double position_y = split_comma.at(2).toDouble();
                double position_z = split_comma.at(3).toDouble();
                double orientation_x = split_comma.at(4).toDouble();
                double orientation_y = split_comma.at(5).toDouble();
                double orientation_z = split_comma.at(6).toDouble();
                double z_down_time = split_comma.at(6).toDouble();


                avg_velocity = 0;
                vel_check_count = 0;

                while (true) {

                    avg_velocity = avg_velocity + qnode.velocity_enclosure;
                    vel_check_count++;

                    if(qnode._labeledCenter_calculate_y < (avg_velocity/vel_check_count) * z_down_time)
                    {
                        _inverse_Moveit(point_enclosure_x, position_y, position_z, orientation_x, orientation_y, orientation_z);

                        break;
                    }
                    Gripper_Close();
                    ros::Duration(1).sleep();

                }
            }
            else if(srr == "P")
            {
                std::string srr1 = split_comma.at(1).toUtf8().constData();
                double position_x;

                if(srr1 == "E")
                {
                    position_x = point_enclosure_x;
                }
                else
                {
                    position_x = split_comma.at(1).toDouble();
                }
                double position_y = split_comma.at(2).toDouble();
                double position_z = split_comma.at(3).toDouble();
                double orientation_x = split_comma.at(4).toDouble();
                double orientation_y = split_comma.at(5).toDouble();
                double orientation_z = split_comma.at(6).toDouble();

                _inverse_Moveit(position_x,position_y,position_z,orientation_x,orientation_y,orientation_z);
                ros::Duration(1).sleep();
            }
            else if(srr == "G")
            {
                int grip = split_comma.at(1).toInt();

                if(grip == 0)
                {
                    ROS_INFO("Gripper_Open");
                    //Gripper_Open();
                    ros::Duration(1).sleep();
                }
                else if(grip == 1)
                {
                    ROS_INFO("Gripper_Close");
                    //Gripper_Close();
                    ros::Duration(1).sleep();
                }

            }
            else if(srr == "S")
            {
                int grip = split_comma.at(1).toInt();

                if(grip == 0)
                {
                    ROS_INFO("SonaMalding Stop");
                    //SonaMalding_Stop();
                    ros::Duration(1).sleep();
                }
                else if(grip == 1)
                {
                    ROS_INFO("SonaMalding Start");
                    //SonaMalding_Start();
                    ros::Duration(1).sleep();
                }

            }
            else if(srr == "D")
            {
                int delay = split_comma.at(1).toInt();
                usleep(delay);

            }

        }

        file.close();

    }

}

void qtros::MainWindow::_inverse_Moveit(double x_point, double y_point, double z_point, double roll, double pitch, double yaw)
{

    geometry_msgs::Pose target_pose;

    target_pose.position.x = x_point;
    target_pose.position.y = y_point;
    target_pose.position.z = z_point;
    double rotate_x = roll*pi/180;
    double rotate_y = pitch*pi/180;
    double rotate_z = yaw*pi/180;

    tf::Quaternion q;
    q.setRPY(rotate_x,rotate_y,rotate_z);
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();
    qnode.move_group->setPoseTarget(target_pose);
    qnode.move_group->move();
}

void qtros::MainWindow::_inverse_point(double x_point, double y_point, double z_point, double roll, double pitch, double yaw)
{

    geometry_msgs::Pose target_pose;

    target_pose.position.x = x_point;
    target_pose.position.y = y_point;
    target_pose.position.z = z_point;
    double rotate_x = roll*pi/180;
    double rotate_y = pitch*pi/180;
    double rotate_z = yaw*pi/180;

    tf::Quaternion q;
    q.setRPY(rotate_x,rotate_y,rotate_z);
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();
    const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

    found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms

    if (found_ik)
    {
      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
      double arr[9];
      for (std::size_t i = 0; i < joint_names.size(); ++i)
      {

        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      }
      //std::copy(joint_values.begin(),joint_values.end(),arr);



      Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

      qnode.move_group->setPoseTarget(target_pose);
      QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);
      ROS_INFO("joint1: %f",Moveit_Jointvalue_1);
      ROS_INFO("joint2: %f",Moveit_Jointvalue_2);
      ROS_INFO("joint3: %f",Moveit_Jointvalue_3);
      ROS_INFO("joint4: %f",Moveit_Jointvalue_4);
      ROS_INFO("joint5: %f",Moveit_Jointvalue_5);
      ROS_INFO("joint6: %f",Moveit_Jointvalue_6);
      QString msg = "Moving!";
      tcpSocket->write(Joint_data.toLocal8Bit().constData());
      ui.listWidget_Server_text->addItem(QString("%1").arg(Joint_data));
      ui.listWidget_Server_text->scrollToBottom();

      while(true)
      {
       if(complete_pose)
          break;
      }

      ros::Duration(1).sleep();
      complete_pose = false;
    }
}

void qtros::MainWindow::_task_test()
{
    _inverse_point(750, 0, 160, _task_roll, _task_pitch, _task_yaw);

    Gripper_Open();
    ros::Duration(1).sleep();

    while (true) {

        if(qnode._labeledCenter_calculate_y < 500)
        {
            point_enclosure_x = qnode._labeledCenter_calculate_x + 15;

            _inverse_point(point_enclosure_x, 0, 40, _task_roll, _task_pitch, _task_yaw);
            break;
        }

    }

    avg_velocity = 0;
    vel_check_count = 0;

    while (true) {

        avg_velocity = avg_velocity + qnode.velocity_enclosure;
        vel_check_count++;

        if(qnode._labeledCenter_calculate_y < (avg_velocity/vel_check_count) * 3.23)
        {
            _inverse_point(point_enclosure_x, 0, 13, _task_roll, _task_pitch, _task_yaw);

            Gripper_Close();
            ros::Duration(1).sleep();

            break;
        }

    }

    _task_function();

}

void qtros::MainWindow::_task_moveit_test()
{
    _inverse_Moveit(750, 0, 160, _task_roll, _task_pitch, _task_yaw);

    Gripper_Open();
    ros::Duration(1).sleep();

    while (true) {

        if(qnode._labeledCenter_calculate_y < 500)
        {
            point_enclosure_x = qnode._labeledCenter_calculate_x + 15;

            _inverse_Moveit(point_enclosure_x, 0, 40, _task_roll, _task_pitch, _task_yaw);
            break;
        }

    }

    avg_velocity = 0;
    vel_check_count = 0;

    while (true) {

        avg_velocity = avg_velocity + qnode.velocity_enclosure;
        vel_check_count++;

        if(qnode._labeledCenter_calculate_y < (avg_velocity/vel_check_count) * 3.23)
        {
            _inverse_Moveit(point_enclosure_x, 0, 13, _task_roll, _task_pitch, _task_yaw);

            Gripper_Close();
            ros::Duration(1).sleep();

            break;
        }

    }

    _task_moveit_function();

}

void qtros::MainWindow::_190714_trakint_test()
{

    geometry_msgs::Pose target_pose;

    point_enclosure_x = qnode._labeledCenter_calculate_x+10;

    target_pose.position.x = point_enclosure_x;
    target_pose.position.y = 0;
    target_pose.position.z = 160;
    double rotate_x = _task_roll*pi/180;
    double rotate_y = _task_pitch*pi/180;
    double rotate_z = _task_yaw*pi/180;

    tf::Quaternion q;
    q.setRPY(rotate_x,rotate_y,rotate_z);
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();
    const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

    found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms

    if (found_ik)
    {
      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
      double arr[9];
      for (std::size_t i = 0; i < joint_names.size(); ++i)
      {

        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      }
      //std::copy(joint_values.begin(),joint_values.end(),arr);



      Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

      qnode.move_group->setPoseTarget(target_pose);
   //   qnode.move_group->move();
      QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);
      //QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(qnode.joint_values[0]).arg(qnode.joint_values[1])
      //    .arg(qnode.joint_values[4]).arg(qnode.joint_values[7]).arg(qnode.joint_values[8]).arg(qnode.joint_values[9]);
      ROS_INFO("joint1: %f",Moveit_Jointvalue_1);
      ROS_INFO("joint2: %f",Moveit_Jointvalue_2);
      ROS_INFO("joint3: %f",Moveit_Jointvalue_3);
      ROS_INFO("joint4: %f",Moveit_Jointvalue_4);
      ROS_INFO("joint5: %f",Moveit_Jointvalue_5);
      ROS_INFO("joint6: %f",Moveit_Jointvalue_6);
      QString msg = "Moving!";
      tcpSocket->write(Joint_data.toLocal8Bit().constData());
      ui.listWidget_Server_text->addItem(QString("%1").arg(Joint_data));
      ui.listWidget_Server_text->scrollToBottom();

      while(true)
      {
       if(complete_pose)
          break;
      }

      ros::Duration(1).sleep();
      complete_pose = false;
      tracking_count++;


      Gripper_Open();
      ros::Duration(1).sleep();

      ROS_INFO("Count: %d ",tracking_count);

    }

    while (true) {

        if(qnode._labeledCenter_calculate_y < 500)
        {
            _190714_trakint_test1();
            break;
        }

    }

    avg_velocity = 0;
    vel_check_count = 0;

    while (true) {

        avg_velocity = avg_velocity + qnode.velocity_enclosure;
        vel_check_count++;

        if(qnode._labeledCenter_calculate_y < (avg_velocity/vel_check_count) * 3.23)
        {
            _190714_trakint_test2();
            break;
        }

    }

    _190714_trakint_test3();
    usleep(300000);

    _190714_trakint_test_sona_500x300_point();
    usleep(300000);

    _190714_trakint_test_sona_700x300_point();
    usleep(300000);

    _190714_trakint_test_sona_700x200_point();
    usleep(150000);
    Gripper_Open();
    usleep(150000);

    _190714_trakint_test_sona_700x300_point();
    usleep(300000);

    _190714_trakint_test_sona_500x300_point();
    usleep(1000000);

    _190714_trakint_test_sona_700x300_point();
    usleep(300000);

    _190714_trakint_test_sona_700x200_point();
    usleep(150000);
    Gripper_Close();
    usleep(150000);

    _190714_trakint_test_sona_700x300_point();
    usleep(300000);

    _190714_trakint_test_sona_500x300_point();
    usleep(300000);

    _190714_trakint_test3();
    usleep(300000);

    _190714_trakint_test4();
    usleep(10000);
    Gripper_Open();
    usleep(10000);

    _190714_trakint_test3();
    usleep(300000);



}

void qtros::MainWindow::_190714_trakint_test1()
{

    geometry_msgs::Pose target_pose;

    point_enclosure_x = qnode._labeledCenter_calculate_x+15;

    target_pose.position.x = point_enclosure_x;
    target_pose.position.y = 0;
    target_pose.position.z = 160;
    double rotate_x = _task_roll*pi/180;
    double rotate_y = _task_pitch*pi/180;
    double rotate_z = _task_yaw*pi/180;

    tf::Quaternion q;
    q.setRPY(rotate_x,rotate_y,rotate_z);
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();
    const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

    found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms

    if (found_ik)
    {
      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
      double arr[9];
      for (std::size_t i = 0; i < joint_names.size(); ++i)
      {

        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      }
      //std::copy(joint_values.begin(),joint_values.end(),arr);



      Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

      qnode.move_group->setPoseTarget(target_pose);
   //   qnode.move_group->move();
      QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);
      //QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(qnode.joint_values[0]).arg(qnode.joint_values[1])
      //    .arg(qnode.joint_values[4]).arg(qnode.joint_values[7]).arg(qnode.joint_values[8]).arg(qnode.joint_values[9]);
      ROS_INFO("joint1: %f",Moveit_Jointvalue_1);
      ROS_INFO("joint2: %f",Moveit_Jointvalue_2);
      ROS_INFO("joint3: %f",Moveit_Jointvalue_3);
      ROS_INFO("joint4: %f",Moveit_Jointvalue_4);
      ROS_INFO("joint5: %f",Moveit_Jointvalue_5);
      ROS_INFO("joint6: %f",Moveit_Jointvalue_6);
      QString msg = "Moving!";
      tcpSocket->write(Joint_data.toLocal8Bit().constData());
      ui.listWidget_Server_text->addItem(QString("%1").arg(Joint_data));
      ui.listWidget_Server_text->scrollToBottom();

      while(true)
      {
       if(complete_pose)
          break;
      }

      ros::Duration(1).sleep();
      complete_pose = false;
    }
}

void qtros::MainWindow::_190714_trakint_test2()
{
    double robot_move_time_start = ros::Time::now().toSec();

    geometry_msgs::Pose target_pose;
    target_pose.position.x = point_enclosure_x;
    target_pose.position.y = 0;
    target_pose.position.z = 138;
    double rotate_x = _task_roll*pi/180;
    double rotate_y = _task_pitch*pi/180;
    double rotate_z = _task_yaw*pi/180;

    tf::Quaternion q;

    q.setRPY(rotate_x,rotate_y,rotate_z);
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();

    const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

    found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms

    if (found_ik)
    {
      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
      double arr[9];
      for (std::size_t i = 0; i < joint_names.size(); ++i)
      {

        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      }
      //std::copy(joint_values.begin(),joint_values.end(),arr);

      Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

      qnode.move_group->setPoseTarget(target_pose);
   //   qnode.move_group->move();
      QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);
      //QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(qnode.joint_values[0]).arg(qnode.joint_values[1])
      //    .arg(qnode.joint_values[4]).arg(qnode.joint_values[7]).arg(qnode.joint_values[8]).arg(qnode.joint_values[9]);
      ROS_INFO("joint1: %f",Moveit_Jointvalue_1);
      ROS_INFO("joint2: %f",Moveit_Jointvalue_2);
      ROS_INFO("joint3: %f",Moveit_Jointvalue_3);
      ROS_INFO("joint4: %f",Moveit_Jointvalue_4);
      ROS_INFO("joint5: %f",Moveit_Jointvalue_5);
      ROS_INFO("joint6: %f",Moveit_Jointvalue_6);
      QString msg = "Moving!";
      tcpSocket->write(Joint_data.toLocal8Bit().constData());
      ui.listWidget_Server_text->addItem(QString("%1").arg(Joint_data));
      ui.listWidget_Server_text->scrollToBottom();

      while(true)
      {
       if(complete_pose)
          break;
      }

      ros::Duration(1).sleep();

      complete_pose = false;

      Gripper_Close();
      double robot_move_time_stop = ros::Time::now().toSec();

      ros::Duration(1).sleep();
      tracking_count++;

      double robot_move_time = robot_move_time_stop - robot_move_time_start;

      ROS_INFO("z move 100mm -> time : %f ",robot_move_time);

      ROS_INFO("Count: %d ",tracking_count);

    }
}

void qtros::MainWindow::_190714_trakint_test3()
{

    geometry_msgs::Pose target_pose;
    target_pose.position.x = point_enclosure_x;
    target_pose.position.y = 0;
    target_pose.position.z = 300;
    double rotate_x = _task_roll*pi/180;
    double rotate_y = _task_pitch*pi/180;
    double rotate_z = _task_yaw*pi/180;

    tf::Quaternion q;

    q.setRPY(rotate_x,rotate_y,rotate_z);
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();

    const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

    found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms

    if (found_ik)
    {
      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
      double arr[9];
      for (std::size_t i = 0; i < joint_names.size(); ++i)
      {

        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      }
      //std::copy(joint_values.begin(),joint_values.end(),arr);

      Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

      qnode.move_group->setPoseTarget(target_pose);
   //   qnode.move_group->move();
      QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);
      //QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(qnode.joint_values[0]).arg(qnode.joint_values[1])
      //    .arg(qnode.joint_values[4]).arg(qnode.joint_values[7]).arg(qnode.joint_values[8]).arg(qnode.joint_values[9]);
      ROS_INFO("joint1: %f",Moveit_Jointvalue_1);
      ROS_INFO("joint2: %f",Moveit_Jointvalue_2);
      ROS_INFO("joint3: %f",Moveit_Jointvalue_3);
      ROS_INFO("joint4: %f",Moveit_Jointvalue_4);
      ROS_INFO("joint5: %f",Moveit_Jointvalue_5);
      ROS_INFO("joint6: %f",Moveit_Jointvalue_6);
      QString msg = "Moving!";
      tcpSocket->write(Joint_data.toLocal8Bit().constData());
      ui.listWidget_Server_text->addItem(QString("%1").arg(Joint_data));
      ui.listWidget_Server_text->scrollToBottom();

      while(true)
      {
       if(complete_pose)
          break;
      }

      ros::Duration(1).sleep();

      complete_pose = false;


    }
}

void qtros::MainWindow::_190714_trakint_test4()
{

    geometry_msgs::Pose target_pose;
    target_pose.position.x = point_enclosure_x;
    target_pose.position.y = 0;
    target_pose.position.z = 150;
    double rotate_x = _task_roll*pi/180;
    double rotate_y = _task_pitch*pi/180;
    double rotate_z = _task_yaw*pi/180;

    tf::Quaternion q;

    q.setRPY(rotate_x,rotate_y,rotate_z);
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();

    const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

    found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms

    if (found_ik)
    {
      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
      double arr[9];
      for (std::size_t i = 0; i < joint_names.size(); ++i)
      {

        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      }
      //std::copy(joint_values.begin(),joint_values.end(),arr);

      Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

      qnode.move_group->setPoseTarget(target_pose);
   //   qnode.move_group->move();
      QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);
      //QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(qnode.joint_values[0]).arg(qnode.joint_values[1])
      //    .arg(qnode.joint_values[4]).arg(qnode.joint_values[7]).arg(qnode.joint_values[8]).arg(qnode.joint_values[9]);
      ROS_INFO("joint1: %f",Moveit_Jointvalue_1);
      ROS_INFO("joint2: %f",Moveit_Jointvalue_2);
      ROS_INFO("joint3: %f",Moveit_Jointvalue_3);
      ROS_INFO("joint4: %f",Moveit_Jointvalue_4);
      ROS_INFO("joint5: %f",Moveit_Jointvalue_5);
      ROS_INFO("joint6: %f",Moveit_Jointvalue_6);
      QString msg = "Moving!";
      tcpSocket->write(Joint_data.toLocal8Bit().constData());
      ui.listWidget_Server_text->addItem(QString("%1").arg(Joint_data));
      ui.listWidget_Server_text->scrollToBottom();

      while(true)
      {
       if(complete_pose)
          break;
      }

      ros::Duration(1).sleep();

      complete_pose = false;


    }
}


void qtros::MainWindow::_190714_trakint_test_sona_500x300_point()
{

    geometry_msgs::Pose target_pose;

    target_pose.position.x = 0;
    target_pose.position.y = -500;
    target_pose.position.z = 300;
    double rotate_x = _task_sona_point_roll*pi/180;
    double rotate_y = _task_sona_point_pitch*pi/180;
    double rotate_z = _task_sona_point_yaw*pi/180;

    tf::Quaternion q;
    q.setRPY(rotate_x,rotate_y,rotate_z);
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();
    const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

    found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms

    if (found_ik)
    {
      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
      double arr[9];
      for (std::size_t i = 0; i < joint_names.size(); ++i)
      {

        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      }
      //std::copy(joint_values.begin(),joint_values.end(),arr);



      Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

      qnode.move_group->setPoseTarget(target_pose);
   //   qnode.move_group->move();
      QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);
      //QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(qnode.joint_values[0]).arg(qnode.joint_values[1])
      //    .arg(qnode.joint_values[4]).arg(qnode.joint_values[7]).arg(qnode.joint_values[8]).arg(qnode.joint_values[9]);
      ROS_INFO("joint1: %f",Moveit_Jointvalue_1);
      ROS_INFO("joint2: %f",Moveit_Jointvalue_2);
      ROS_INFO("joint3: %f",Moveit_Jointvalue_3);
      ROS_INFO("joint4: %f",Moveit_Jointvalue_4);
      ROS_INFO("joint5: %f",Moveit_Jointvalue_5);
      ROS_INFO("joint6: %f",Moveit_Jointvalue_6);
      QString msg = "Moving!";
      tcpSocket->write(Joint_data.toLocal8Bit().constData());
      ui.listWidget_Server_text->addItem(QString("%1").arg(Joint_data));
      ui.listWidget_Server_text->scrollToBottom();

      while(true)
      {
       if(complete_pose)
          break;
      }

      ros::Duration(1).sleep();
      complete_pose = false;
    }
}

void qtros::MainWindow::_190714_trakint_test_sona_700x300_point()
{

    geometry_msgs::Pose target_pose;

    target_pose.position.x = 0;
    target_pose.position.y = -700;
    target_pose.position.z = 300;
    double rotate_x = _task_sona_point_roll*pi/180;
    double rotate_y = _task_sona_point_pitch*pi/180;
    double rotate_z = _task_sona_point_yaw*pi/180;

    tf::Quaternion q;
    q.setRPY(rotate_x,rotate_y,rotate_z);
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();
    const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

    found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms

    if (found_ik)
    {
      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
      double arr[9];
      for (std::size_t i = 0; i < joint_names.size(); ++i)
      {

        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      }
      //std::copy(joint_values.begin(),joint_values.end(),arr);



      Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

      qnode.move_group->setPoseTarget(target_pose);
   //   qnode.move_group->move();
      QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);
      //QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(qnode.joint_values[0]).arg(qnode.joint_values[1])
      //    .arg(qnode.joint_values[4]).arg(qnode.joint_values[7]).arg(qnode.joint_values[8]).arg(qnode.joint_values[9]);
      ROS_INFO("joint1: %f",Moveit_Jointvalue_1);
      ROS_INFO("joint2: %f",Moveit_Jointvalue_2);
      ROS_INFO("joint3: %f",Moveit_Jointvalue_3);
      ROS_INFO("joint4: %f",Moveit_Jointvalue_4);
      ROS_INFO("joint5: %f",Moveit_Jointvalue_5);
      ROS_INFO("joint6: %f",Moveit_Jointvalue_6);
      QString msg = "Moving!";
      tcpSocket->write(Joint_data.toLocal8Bit().constData());
      ui.listWidget_Server_text->addItem(QString("%1").arg(Joint_data));
      ui.listWidget_Server_text->scrollToBottom();

      while(true)
      {
       if(complete_pose)
          break;
      }

      ros::Duration(1).sleep();
      complete_pose = false;
    }
}

void qtros::MainWindow::_190714_trakint_test_sona_700x200_point()
{

    geometry_msgs::Pose target_pose;

    target_pose.position.x = 0;
    target_pose.position.y = -700;
    target_pose.position.z = 200;
    double rotate_x = _task_sona_point_roll*pi/180;
    double rotate_y = _task_sona_point_pitch*pi/180;
    double rotate_z = _task_sona_point_yaw*pi/180;

    tf::Quaternion q;
    q.setRPY(rotate_x,rotate_y,rotate_z);
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();
    const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

    found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms

    if (found_ik)
    {
      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
      double arr[9];
      for (std::size_t i = 0; i < joint_names.size(); ++i)
      {

        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      }
      //std::copy(joint_values.begin(),joint_values.end(),arr);



      Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

      qnode.move_group->setPoseTarget(target_pose);
   //   qnode.move_group->move();
      QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);
      //QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(qnode.joint_values[0]).arg(qnode.joint_values[1])
      //    .arg(qnode.joint_values[4]).arg(qnode.joint_values[7]).arg(qnode.joint_values[8]).arg(qnode.joint_values[9]);
      ROS_INFO("joint1: %f",Moveit_Jointvalue_1);
      ROS_INFO("joint2: %f",Moveit_Jointvalue_2);
      ROS_INFO("joint3: %f",Moveit_Jointvalue_3);
      ROS_INFO("joint4: %f",Moveit_Jointvalue_4);
      ROS_INFO("joint5: %f",Moveit_Jointvalue_5);
      ROS_INFO("joint6: %f",Moveit_Jointvalue_6);
      QString msg = "Moving!";
      tcpSocket->write(Joint_data.toLocal8Bit().constData());
      ui.listWidget_Server_text->addItem(QString("%1").arg(Joint_data));
      ui.listWidget_Server_text->scrollToBottom();

      while(true)
      {
       if(complete_pose)
          break;
      }

      ros::Duration(1).sleep();
      complete_pose = false;
    }
}



void qtros::MainWindow::tracking_test1()
{
    geometry_msgs::Pose target_pose;
    target_pose.position.x = qnode._labeledCenter_calculate_x;
    target_pose.position.y = 50;
    target_pose.position.z = 300;
    double rotate_x = 117*pi/180;
    double rotate_y = (-90)*pi/180;
    double rotate_z = (-117)*pi/180;

    tf::Quaternion q;
    q.setRPY(rotate_x,rotate_y,rotate_z);
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();
    const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

    found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms

    if (found_ik)
    {
      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
      double arr[9];
      for (std::size_t i = 0; i < joint_names.size(); ++i)
      {

        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      }
      //std::copy(joint_values.begin(),joint_values.end(),arr);



      Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
      Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

      qnode.move_group->setPoseTarget(target_pose);
   //   qnode.move_group->move();
      QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);
      //QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(qnode.joint_values[0]).arg(qnode.joint_values[1])
      //    .arg(qnode.joint_values[4]).arg(qnode.joint_values[7]).arg(qnode.joint_values[8]).arg(qnode.joint_values[9]);
      ROS_INFO("joint1: %f",Moveit_Jointvalue_1);
      ROS_INFO("joint2: %f",Moveit_Jointvalue_2);
      ROS_INFO("joint3: %f",Moveit_Jointvalue_3);
      ROS_INFO("joint4: %f",Moveit_Jointvalue_4);
      ROS_INFO("joint5: %f",Moveit_Jointvalue_5);
      ROS_INFO("joint6: %f",Moveit_Jointvalue_6);
      QString msg = "Moving!";
      tcpSocket->write(Joint_data.toLocal8Bit().constData());
      ui.listWidget_Server_text->addItem(QString("%1").arg(Joint_data));
      ui.listWidget_Server_text->scrollToBottom();

      while(true)
      {
       if(complete_pose)
          break;
      }

      ros::Duration(1).sleep();
      complete_pose = false;
      tracking_count++;

      ROS_INFO("Count: %d ",tracking_count);

    }
    while(true)
    {

        if(qnode._labeledCenter_calculate_y < 50)
        {
            geometry_msgs::Pose target_pose;
            target_pose.position.x = qnode._labeledCenter_calculate_x;
            target_pose.position.y = 50;
            target_pose.position.z = 200;
            double rotate_x = 117*pi/180;
            double rotate_y = (-90)*pi/180;
            double rotate_z = (-117)*pi/180;

            tf::Quaternion q;
            q.setRPY(rotate_x,rotate_y,rotate_z);
            target_pose.orientation.x = q.x();
            target_pose.orientation.y = q.y();
            target_pose.orientation.z = q.z();
            target_pose.orientation.w = q.w();
            const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
            robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
            const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

            found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms

            if (found_ik)
            {
              kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
              double arr[9];
              for (std::size_t i = 0; i < joint_names.size(); ++i)
              {

                ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
              }
              //std::copy(joint_values.begin(),joint_values.end(),arr);



              Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
              Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
              Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
              Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
              Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
              Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

              qnode.move_group->setPoseTarget(target_pose);
           //   qnode.move_group->move();
              QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);
              //QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(qnode.joint_values[0]).arg(qnode.joint_values[1])
              //    .arg(qnode.joint_values[4]).arg(qnode.joint_values[7]).arg(qnode.joint_values[8]).arg(qnode.joint_values[9]);
              ROS_INFO("joint1: %f",Moveit_Jointvalue_1);
              ROS_INFO("joint2: %f",Moveit_Jointvalue_2);
              ROS_INFO("joint3: %f",Moveit_Jointvalue_3);
              ROS_INFO("joint4: %f",Moveit_Jointvalue_4);
              ROS_INFO("joint5: %f",Moveit_Jointvalue_5);
              ROS_INFO("joint6: %f",Moveit_Jointvalue_6);
              QString msg = "Moving!";
              tcpSocket->write(Joint_data.toLocal8Bit().constData());
              ui.listWidget_Server_text->addItem(QString("%1").arg(Joint_data));
              ui.listWidget_Server_text->scrollToBottom();

              while(true)
              {
               if(complete_pose)
                  break;
              }

              ros::Duration(1).sleep();

              Gripper_Close();

              complete_pose = false;
              tracking_count++;

              ROS_INFO("Count: %d ",tracking_count);

            break;
            }
        }
    }
}

void qtros::MainWindow::tracking()
{
    tracking_count = 0;

    while(qnode._labeledCenter_calculate_y > 0)
    {
        geometry_msgs::Pose target_pose;
        target_pose.position.x = qnode._labeledCenter_calculate_x;
        target_pose.position.y = qnode._labeledCenter_calculate_y;
        target_pose.position.z = 400;
        double rotate_x = 117*pi/180;
        double rotate_y = (-90)*pi/180;
        double rotate_z = (-117)*pi/180;

        tf::Quaternion q;
        q.setRPY(rotate_x,rotate_y,rotate_z);
        target_pose.orientation.x = q.x();
        target_pose.orientation.y = q.y();
        target_pose.orientation.z = q.z();
        target_pose.orientation.w = q.w();
        const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
        robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
        const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

        found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms

        if (found_ik)
        {
          kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
          double arr[9];
          for (std::size_t i = 0; i < joint_names.size(); ++i)
          {

            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
          }
          //std::copy(joint_values.begin(),joint_values.end(),arr);



          Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

          qnode.move_group->setPoseTarget(target_pose);
       //   qnode.move_group->move();
          QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);
          //QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(qnode.joint_values[0]).arg(qnode.joint_values[1])
          //    .arg(qnode.joint_values[4]).arg(qnode.joint_values[7]).arg(qnode.joint_values[8]).arg(qnode.joint_values[9]);
          ROS_INFO("joint1: %f",Moveit_Jointvalue_1);
          ROS_INFO("joint2: %f",Moveit_Jointvalue_2);
          ROS_INFO("joint3: %f",Moveit_Jointvalue_3);
          ROS_INFO("joint4: %f",Moveit_Jointvalue_4);
          ROS_INFO("joint5: %f",Moveit_Jointvalue_5);
          ROS_INFO("joint6: %f",Moveit_Jointvalue_6);
          QString msg = "Moving!";
          tcpSocket->write(Joint_data.toLocal8Bit().constData());
          ui.listWidget_Server_text->addItem(QString("%1").arg(Joint_data));
          ui.listWidget_Server_text->scrollToBottom();

          while(true)
          {
           if(complete_pose)
              break;
          }

          ros::Duration(1).sleep();
          complete_pose = false;
          tracking_count++;

          ROS_INFO("Count: %d ",tracking_count);

        }
    }
}

void qtros::MainWindow::Task_pick_and_place() //here3
{
  int turn = 0;
  int i = 0;
  complete_pose = false;

  while(i<1)
  {
      switch (turn) {
      case 0:
      {
          geometry_msgs::Pose target_pose;
          target_pose.position.x = 600;
          target_pose.position.y = 0;
          target_pose.position.z = 300;
          double rotate_x = 117*pi/180;
          double rotate_y = (-90)*pi/180;
          double rotate_z = (-117)*pi/180;

          tf::Quaternion q;
          q.setRPY(rotate_x,rotate_y,rotate_z);
          target_pose.orientation.x = q.x();
          target_pose.orientation.y = q.y();
          target_pose.orientation.z = q.z();
          target_pose.orientation.w = q.w();
          const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
          robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
          const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

          found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms

          if (found_ik)
          {
            kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
            double arr[9];
            for (std::size_t i = 0; i < joint_names.size(); ++i)
            {

              ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
            }
            //std::copy(joint_values.begin(),joint_values.end(),arr);



            Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
            Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
            Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
            Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
            Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
            Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

            qnode.move_group->setPoseTarget(target_pose);
         //   qnode.move_group->move();
            QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);
            //QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(qnode.joint_values[0]).arg(qnode.joint_values[1])
            //    .arg(qnode.joint_values[4]).arg(qnode.joint_values[7]).arg(qnode.joint_values[8]).arg(qnode.joint_values[9]);
            ROS_INFO("joint1: %f",Moveit_Jointvalue_1);
            ROS_INFO("joint2: %f",Moveit_Jointvalue_2);
            ROS_INFO("joint3: %f",Moveit_Jointvalue_3);
            ROS_INFO("joint4: %f",Moveit_Jointvalue_4);
            ROS_INFO("joint5: %f",Moveit_Jointvalue_5);
            ROS_INFO("joint6: %f",Moveit_Jointvalue_6);
            QString msg = "Moving!";
            tcpSocket->write(Joint_data.toLocal8Bit().constData());
            ui.listWidget_Server_text->addItem(QString("%1").arg(Joint_data));
            ui.listWidget_Server_text->scrollToBottom();

            while(true)
            {
             if(complete_pose)
                break;
            }

            ros::Duration(1).sleep();
            complete_pose = false;
            turn = 1;

          }
          else
          {
            ROS_INFO("Did not find IK solution");
          }

          break;
        }


      case 1:
      {
          geometry_msgs::Pose target_pose;
          target_pose.position.x = 600;
          target_pose.position.y = 0;
          target_pose.position.z = 150;
          double rotate_x = 117*pi/180;
          double rotate_y = (-90)*pi/180;
          double rotate_z = (-117)*pi/180;



        tf::Quaternion q;
        q.setRPY(rotate_x,rotate_y,rotate_z);
        target_pose.orientation.x = q.x();
        target_pose.orientation.y = q.y();
        target_pose.orientation.z = q.z();
        target_pose.orientation.w = q.w();
        const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
        robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
        const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

        found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms

        if (found_ik)
        {
          kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
          double arr[9];
          for (std::size_t i = 0; i < joint_names.size(); ++i)
          {

            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
          }
          //std::copy(joint_values.begin(),joint_values.end(),arr);



          Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

          qnode.move_group->setPoseTarget(target_pose);
       //   qnode.move_group->move();
          QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);
          //QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(qnode.joint_values[0]).arg(qnode.joint_values[1])
          //    .arg(qnode.joint_values[4]).arg(qnode.joint_values[7]).arg(qnode.joint_values[8]).arg(qnode.joint_values[9]);
          ROS_INFO("joint1: %f",Moveit_Jointvalue_1);
          ROS_INFO("joint2: %f",Moveit_Jointvalue_2);
          ROS_INFO("joint3: %f",Moveit_Jointvalue_3);
          ROS_INFO("joint4: %f",Moveit_Jointvalue_4);
          ROS_INFO("joint5: %f",Moveit_Jointvalue_5);
          ROS_INFO("joint6: %f",Moveit_Jointvalue_6);
          QString msg = "Moving!";
          tcpSocket->write(Joint_data.toLocal8Bit().constData());
          ui.listWidget_Server_text->addItem(QString("%1").arg(Joint_data));
          ui.listWidget_Server_text->scrollToBottom();

          while(true)
          {
           if(complete_pose)
              break;
          }

          ros::Duration(1).sleep();

          Gripper_Close();

          ros::Duration(1).sleep();

          complete_pose = false;
          turn = 2;

        }
        else
        {
          ROS_INFO("Did not find IK solution");
        }

        break;
      }

      case 2:
      {
          geometry_msgs::Pose target_pose;
          target_pose.position.x = 600;
          target_pose.position.y = 0;
          target_pose.position.z = 300;
          double rotate_x = 117*pi/180;
          double rotate_y = (-90)*pi/180;
          double rotate_z = (-117)*pi/180;

        tf::Quaternion q;
        q.setRPY(rotate_x,rotate_y,rotate_z);
        target_pose.orientation.x = q.x();
        target_pose.orientation.y = q.y();
        target_pose.orientation.z = q.z();
        target_pose.orientation.w = q.w();
        const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
        robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
        const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

        found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms

        if (found_ik)
        {
          kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
          double arr[9];
          for (std::size_t i = 0; i < joint_names.size(); ++i)
          {

            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
          }
          //std::copy(joint_values.begin(),joint_values.end(),arr);



          Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

          qnode.move_group->setPoseTarget(target_pose);
        //  qnode.move_group->move();
          QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);
          //QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(qnode.joint_values[0]).arg(qnode.joint_values[1])
          //    .arg(qnode.joint_values[4]).arg(qnode.joint_values[7]).arg(qnode.joint_values[8]).arg(qnode.joint_values[9]);
          ROS_INFO("joint1: %f",Moveit_Jointvalue_1);
          ROS_INFO("joint2: %f",Moveit_Jointvalue_2);
          ROS_INFO("joint3: %f",Moveit_Jointvalue_3);
          ROS_INFO("joint4: %f",Moveit_Jointvalue_4);
          ROS_INFO("joint5: %f",Moveit_Jointvalue_5);
          ROS_INFO("joint6: %f",Moveit_Jointvalue_6);
          QString msg = "Moving!";
          tcpSocket->write(Joint_data.toLocal8Bit().constData());
          ui.listWidget_Server_text->addItem(QString("%1").arg(Joint_data));
          ui.listWidget_Server_text->scrollToBottom();

          while(true)
          {
           if(complete_pose)
              break;
          }

          ros::Duration(1).sleep();
          complete_pose = false;
          turn = 3;

        }
        else
        {
          ROS_INFO("Did not find IK solution");
        }

        break;
      }

      case 3:
      {
          geometry_msgs::Pose target_pose;
          target_pose.position.x = 0;
          target_pose.position.y = -500;
          target_pose.position.z = 300;
          double rotate_x = 117*pi/180;
          double rotate_y = (-90)*pi/180;
          double rotate_z = 155*pi/180;


        tf::Quaternion q;
        q.setRPY(rotate_x,rotate_y,rotate_z);
        target_pose.orientation.x = q.x();
        target_pose.orientation.y = q.y();
        target_pose.orientation.z = q.z();
        target_pose.orientation.w = q.w();
        const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
        robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
        const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

        found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms

        if (found_ik)
        {
          kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
          double arr[9];
          for (std::size_t i = 0; i < joint_names.size(); ++i)
          {

            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
          }
          //std::copy(joint_values.begin(),joint_values.end(),arr);



          Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

          qnode.move_group->setPoseTarget(target_pose);
       //   qnode.move_group->move();
          QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);
          //QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(qnode.joint_values[0]).arg(qnode.joint_values[1])
          //    .arg(qnode.joint_values[4]).arg(qnode.joint_values[7]).arg(qnode.joint_values[8]).arg(qnode.joint_values[9]);
          ROS_INFO("joint1: %f",Moveit_Jointvalue_1);
          ROS_INFO("joint2: %f",Moveit_Jointvalue_2);
          ROS_INFO("joint3: %f",Moveit_Jointvalue_3);
          ROS_INFO("joint4: %f",Moveit_Jointvalue_4);
          ROS_INFO("joint5: %f",Moveit_Jointvalue_5);
          ROS_INFO("joint6: %f",Moveit_Jointvalue_6);
          QString msg = "Moving!";
          tcpSocket->write(Joint_data.toLocal8Bit().constData());
          ui.listWidget_Server_text->addItem(QString("%1").arg(Joint_data));
          ui.listWidget_Server_text->scrollToBottom();

          while(true)
          {
           if(complete_pose)
              break;
          }

          ros::Duration(1).sleep();


          complete_pose = false;
          turn = 4;






        }
        else
        {
          ROS_INFO("Did not find IK solution");
        }


        break;
      }

      case 4:
      {
          geometry_msgs::Pose target_pose;
          target_pose.position.x = 0;
          target_pose.position.y = -700;
          target_pose.position.z = 300;
          double rotate_x = 117*pi/180;
          double rotate_y = (-90)*pi/180;
          double rotate_z = 155*pi/180;


        tf::Quaternion q;
        q.setRPY(rotate_x,rotate_y,rotate_z);
        target_pose.orientation.x = q.x();
        target_pose.orientation.y = q.y();
        target_pose.orientation.z = q.z();
        target_pose.orientation.w = q.w();
        const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
        robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
        const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

        found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms

        if (found_ik)
        {
          kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
          double arr[9];
          for (std::size_t i = 0; i < joint_names.size(); ++i)
          {

            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
          }
          //std::copy(joint_values.begin(),joint_values.end(),arr);



          Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

          qnode.move_group->setPoseTarget(target_pose);
       //   qnode.move_group->move();
          QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);
          //QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(qnode.joint_values[0]).arg(qnode.joint_values[1])
          //    .arg(qnode.joint_values[4]).arg(qnode.joint_values[7]).arg(qnode.joint_values[8]).arg(qnode.joint_values[9]);
          ROS_INFO("joint1: %f",Moveit_Jointvalue_1);
          ROS_INFO("joint2: %f",Moveit_Jointvalue_2);
          ROS_INFO("joint3: %f",Moveit_Jointvalue_3);
          ROS_INFO("joint4: %f",Moveit_Jointvalue_4);
          ROS_INFO("joint5: %f",Moveit_Jointvalue_5);
          ROS_INFO("joint6: %f",Moveit_Jointvalue_6);
          QString msg = "Moving!";
          tcpSocket->write(Joint_data.toLocal8Bit().constData());
          ui.listWidget_Server_text->addItem(QString("%1").arg(Joint_data));
          ui.listWidget_Server_text->scrollToBottom();

          while(true)
          {
           if(complete_pose)
              break;
          }


          ros::Duration(1).sleep();

          Gripper_Open();

          ros::Duration(1).sleep();


          complete_pose = false;
          turn = 5;





        }
        else
        {
          ROS_INFO("Did not find IK solution");
        }


        break;
      }

      case 5:
      {
          geometry_msgs::Pose target_pose;
          target_pose.position.x = 0;
          target_pose.position.y = -500;
          target_pose.position.z = 300;
          double rotate_x = 117*pi/180;
          double rotate_y = (-90)*pi/180;
          double rotate_z = 155*pi/180;


        tf::Quaternion q;
        q.setRPY(rotate_x,rotate_y,rotate_z);
        target_pose.orientation.x = q.x();
        target_pose.orientation.y = q.y();
        target_pose.orientation.z = q.z();
        target_pose.orientation.w = q.w();
        const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
        robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
        const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

        found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms

        if (found_ik)
        {
          kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
          double arr[9];
          for (std::size_t i = 0; i < joint_names.size(); ++i)
          {

            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
          }
          //std::copy(joint_values.begin(),joint_values.end(),arr);



          Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

          qnode.move_group->setPoseTarget(target_pose);
       //   qnode.move_group->move();
          QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);
          //QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(qnode.joint_values[0]).arg(qnode.joint_values[1])
          //    .arg(qnode.joint_values[4]).arg(qnode.joint_values[7]).arg(qnode.joint_values[8]).arg(qnode.joint_values[9]);
          ROS_INFO("joint1: %f",Moveit_Jointvalue_1);
          ROS_INFO("joint2: %f",Moveit_Jointvalue_2);
          ROS_INFO("joint3: %f",Moveit_Jointvalue_3);
          ROS_INFO("joint4: %f",Moveit_Jointvalue_4);
          ROS_INFO("joint5: %f",Moveit_Jointvalue_5);
          ROS_INFO("joint6: %f",Moveit_Jointvalue_6);
          QString msg = "Moving!";
          tcpSocket->write(Joint_data.toLocal8Bit().constData());
          ui.listWidget_Server_text->addItem(QString("%1").arg(Joint_data));
          ui.listWidget_Server_text->scrollToBottom();

          while(true)
          {
           if(complete_pose)
              break;
          }

          ros::Duration(10).sleep();


          complete_pose = false;
          turn = 6;

        }
        else
        {
          ROS_INFO("Did not find IK solution");
        }


        break;
      }

      case 6:
      {
          geometry_msgs::Pose target_pose;
          target_pose.position.x = 0;
          target_pose.position.y = -700;
          target_pose.position.z = 300;
          double rotate_x = 117*pi/180;
          double rotate_y = (-90)*pi/180;
          double rotate_z = 155*pi/180;


        tf::Quaternion q;
        q.setRPY(rotate_x,rotate_y,rotate_z);
        target_pose.orientation.x = q.x();
        target_pose.orientation.y = q.y();
        target_pose.orientation.z = q.z();
        target_pose.orientation.w = q.w();
        const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
        robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
        const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

        found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms

        if (found_ik)
        {
          kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
          double arr[9];
          for (std::size_t i = 0; i < joint_names.size(); ++i)
          {

            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
          }
          //std::copy(joint_values.begin(),joint_values.end(),arr);



          Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

          qnode.move_group->setPoseTarget(target_pose);
       //   qnode.move_group->move();
          QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);
          //QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(qnode.joint_values[0]).arg(qnode.joint_values[1])
          //    .arg(qnode.joint_values[4]).arg(qnode.joint_values[7]).arg(qnode.joint_values[8]).arg(qnode.joint_values[9]);
          ROS_INFO("joint1: %f",Moveit_Jointvalue_1);
          ROS_INFO("joint2: %f",Moveit_Jointvalue_2);
          ROS_INFO("joint3: %f",Moveit_Jointvalue_3);
          ROS_INFO("joint4: %f",Moveit_Jointvalue_4);
          ROS_INFO("joint5: %f",Moveit_Jointvalue_5);
          ROS_INFO("joint6: %f",Moveit_Jointvalue_6);
          QString msg = "Moving!";
          tcpSocket->write(Joint_data.toLocal8Bit().constData());
          ui.listWidget_Server_text->addItem(QString("%1").arg(Joint_data));
          ui.listWidget_Server_text->scrollToBottom();

          while(true)
          {
           if(complete_pose)
              break;
          }

          ros::Duration(1).sleep();

          Gripper_Close();

          ros::Duration(1).sleep();


          complete_pose = false;
          turn = 7;

        }
        else
        {
          ROS_INFO("Did not find IK solution");
        }


        break;
      }
      case 7:
      {
          geometry_msgs::Pose target_pose;
          target_pose.position.x = 0;
          target_pose.position.y = -500;
          target_pose.position.z = 300;
          double rotate_x = 117*pi/180;
          double rotate_y = (-90)*pi/180;
          double rotate_z = 155*pi/180;


        tf::Quaternion q;
        q.setRPY(rotate_x,rotate_y,rotate_z);
        target_pose.orientation.x = q.x();
        target_pose.orientation.y = q.y();
        target_pose.orientation.z = q.z();
        target_pose.orientation.w = q.w();
        const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
        robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
        const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

        found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms

        if (found_ik)
        {
          kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
          double arr[9];
          for (std::size_t i = 0; i < joint_names.size(); ++i)
          {

            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
          }
          //std::copy(joint_values.begin(),joint_values.end(),arr);



          Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

          qnode.move_group->setPoseTarget(target_pose);
       //   qnode.move_group->move();
          QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);
          //QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(qnode.joint_values[0]).arg(qnode.joint_values[1])
          //    .arg(qnode.joint_values[4]).arg(qnode.joint_values[7]).arg(qnode.joint_values[8]).arg(qnode.joint_values[9]);
          ROS_INFO("joint1: %f",Moveit_Jointvalue_1);
          ROS_INFO("joint2: %f",Moveit_Jointvalue_2);
          ROS_INFO("joint3: %f",Moveit_Jointvalue_3);
          ROS_INFO("joint4: %f",Moveit_Jointvalue_4);
          ROS_INFO("joint5: %f",Moveit_Jointvalue_5);
          ROS_INFO("joint6: %f",Moveit_Jointvalue_6);
          QString msg = "Moving!";
          tcpSocket->write(Joint_data.toLocal8Bit().constData());
          ui.listWidget_Server_text->addItem(QString("%1").arg(Joint_data));
          ui.listWidget_Server_text->scrollToBottom();

          while(true)
          {
           if(complete_pose)
              break;
          }

          ros::Duration(1).sleep();


          complete_pose = false;
          turn = 8;

        }
        else
        {
          ROS_INFO("Did not find IK solution");
        }


        break;
      }
      case 8:
      {
          geometry_msgs::Pose target_pose;
          target_pose.position.x = 600;
          target_pose.position.y = 0;
          target_pose.position.z = 300;
          double rotate_x = 117*pi/180;
          double rotate_y = (-90)*pi/180;
          double rotate_z = (-117)*pi/180;


        tf::Quaternion q;
        q.setRPY(rotate_x,rotate_y,rotate_z);
        target_pose.orientation.x = q.x();
        target_pose.orientation.y = q.y();
        target_pose.orientation.z = q.z();
        target_pose.orientation.w = q.w();
        const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
        robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
        const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

        found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms

        if (found_ik)
        {
          kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
          double arr[9];
          for (std::size_t i = 0; i < joint_names.size(); ++i)
          {

            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
          }
          //std::copy(joint_values.begin(),joint_values.end(),arr);



          Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

          qnode.move_group->setPoseTarget(target_pose);
       //   qnode.move_group->move();
          QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);
          //QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(qnode.joint_values[0]).arg(qnode.joint_values[1])
          //    .arg(qnode.joint_values[4]).arg(qnode.joint_values[7]).arg(qnode.joint_values[8]).arg(qnode.joint_values[9]);
          ROS_INFO("joint1: %f",Moveit_Jointvalue_1);
          ROS_INFO("joint2: %f",Moveit_Jointvalue_2);
          ROS_INFO("joint3: %f",Moveit_Jointvalue_3);
          ROS_INFO("joint4: %f",Moveit_Jointvalue_4);
          ROS_INFO("joint5: %f",Moveit_Jointvalue_5);
          ROS_INFO("joint6: %f",Moveit_Jointvalue_6);
          QString msg = "Moving!";
          tcpSocket->write(Joint_data.toLocal8Bit().constData());
          ui.listWidget_Server_text->addItem(QString("%1").arg(Joint_data));
          ui.listWidget_Server_text->scrollToBottom();

          while(true)
          {
           if(complete_pose)
              break;
          }

          ros::Duration(1).sleep();


          complete_pose = false;
          turn = 9;

        }
        else
        {
          ROS_INFO("Did not find IK solution");
        }


        break;
      }
      case 9:
      {
          geometry_msgs::Pose target_pose;
          target_pose.position.x = 600;
          target_pose.position.y = 0;
          target_pose.position.z = 200;
          double rotate_x = 117*pi/180;
          double rotate_y = (-90)*pi/180;
          double rotate_z = (-117)*pi/180;


        tf::Quaternion q;
        q.setRPY(rotate_x,rotate_y,rotate_z);
        target_pose.orientation.x = q.x();
        target_pose.orientation.y = q.y();
        target_pose.orientation.z = q.z();
        target_pose.orientation.w = q.w();
        const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
        robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
        const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

        found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms

        if (found_ik)
        {
          kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
          double arr[9];
          for (std::size_t i = 0; i < joint_names.size(); ++i)
          {

            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
          }
          //std::copy(joint_values.begin(),joint_values.end(),arr);



          Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

          qnode.move_group->setPoseTarget(target_pose);
       //   qnode.move_group->move();
          QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);
          //QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(qnode.joint_values[0]).arg(qnode.joint_values[1])
          //    .arg(qnode.joint_values[4]).arg(qnode.joint_values[7]).arg(qnode.joint_values[8]).arg(qnode.joint_values[9]);
          ROS_INFO("joint1: %f",Moveit_Jointvalue_1);
          ROS_INFO("joint2: %f",Moveit_Jointvalue_2);
          ROS_INFO("joint3: %f",Moveit_Jointvalue_3);
          ROS_INFO("joint4: %f",Moveit_Jointvalue_4);
          ROS_INFO("joint5: %f",Moveit_Jointvalue_5);
          ROS_INFO("joint6: %f",Moveit_Jointvalue_6);
          QString msg = "Moving!";
          tcpSocket->write(Joint_data.toLocal8Bit().constData());
          ui.listWidget_Server_text->addItem(QString("%1").arg(Joint_data));
          ui.listWidget_Server_text->scrollToBottom();

          while(true)
          {
           if(complete_pose)
              break;
          }

          ros::Duration(1).sleep();

          Gripper_Open();

          ros::Duration(1).sleep();



          complete_pose = false;
          turn = 10;

        }
        else
        {
          ROS_INFO("Did not find IK solution");
        }


        break;
      }
      case 10:
      {
          geometry_msgs::Pose target_pose;
          target_pose.position.x = 600;
          target_pose.position.y = 0;
          target_pose.position.z = 300;
          double rotate_x = 117*pi/180;
          double rotate_y = (-90)*pi/180;
          double rotate_z = (-117)*pi/180;


        tf::Quaternion q;
        q.setRPY(rotate_x,rotate_y,rotate_z);
        target_pose.orientation.x = q.x();
        target_pose.orientation.y = q.y();
        target_pose.orientation.z = q.z();
        target_pose.orientation.w = q.w();
        const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
        robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(qnode.kinematic_model));
        const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

        found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.1ms

        if (found_ik)
        {
          kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
          double arr[9];
          for (std::size_t i = 0; i < joint_names.size(); ++i)
          {

            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
          }
          //std::copy(joint_values.begin(),joint_values.end(),arr);



          Moveit_Jointvalue_1 = QString::number(joint_values[0]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_2 = QString::number(joint_values[1]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_3 = QString::number(joint_values[4]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_4 = QString::number(joint_values[7]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_5 = QString::number(joint_values[8]*180/pi,'f',4).toDouble();
          Moveit_Jointvalue_6 = QString::number(joint_values[9]*180/pi,'f',4).toDouble();

          qnode.move_group->setPoseTarget(target_pose);
       //   qnode.move_group->move();
          QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(Moveit_Jointvalue_1).arg(Moveit_Jointvalue_2).arg(Moveit_Jointvalue_3).arg(Moveit_Jointvalue_4).arg(Moveit_Jointvalue_5).arg(Moveit_Jointvalue_6);
          //QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(qnode.joint_values[0]).arg(qnode.joint_values[1])
          //    .arg(qnode.joint_values[4]).arg(qnode.joint_values[7]).arg(qnode.joint_values[8]).arg(qnode.joint_values[9]);
          ROS_INFO("joint1: %f",Moveit_Jointvalue_1);
          ROS_INFO("joint2: %f",Moveit_Jointvalue_2);
          ROS_INFO("joint3: %f",Moveit_Jointvalue_3);
          ROS_INFO("joint4: %f",Moveit_Jointvalue_4);
          ROS_INFO("joint5: %f",Moveit_Jointvalue_5);
          ROS_INFO("joint6: %f",Moveit_Jointvalue_6);
          QString msg = "Moving!";
          tcpSocket->write(Joint_data.toLocal8Bit().constData());
          ui.listWidget_Server_text->addItem(QString("%1").arg(Joint_data));
          ui.listWidget_Server_text->scrollToBottom();

          while(true)
          {
           if(complete_pose)
              break;
          }

          ros::Duration(1).sleep();


          complete_pose = false;
          turn = 0;
          i++;

        }
        else
        {
          ROS_INFO("Did not find IK solution");
        }


        break;
      }


    }
  }
}
