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
#include <iostream>
#include "../include/qtros/main_window.hpp"
#include <string>
#include <iostream>
#include <tf/transform_datatypes.h>



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
  if(msg.contains("complete"))
  {
    complete_pose = true;
  }


}



void MainWindow::on_button_connect_clicked(bool check ) {
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
  target_pose.orientation.x = q.x();
  target_pose.orientation.y = q.y();
  target_pose.orientation.z = q.z();
  target_pose.orientation.w = q.w();
  qnode.move_group->setPoseTarget(target_pose);
  qnode.move_group->move();
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

void qtros::MainWindow::on_btnRobot_SetJoint_clicked()
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



void qtros::MainWindow::on_btnDetectPose_clicked()
{
  //moveit

  const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
  moveit::core::RobotStatePtr current_state = qnode.move_group->getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group,joint_group_positions);
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
  QString msg = "Pose_set!";
  tcpSocket->write(Joint_data.toLocal8Bit().constData());
  ui.listWidget_Server_text->addItem(QString("%1").arg(msg));
  ui.listWidget_Server_text->scrollToBottom();
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
  boost::thread th1(&MainWindow::Thread_point, this);



  //th1.join();
}

void qtros::MainWindow::on_btnGPIO_ON_clicked()
{
  QString msg = "GPIO ON!";
  tcpSocket->write(QString("GPIO_OUT_ON\n").toLocal8Bit().constData());
  ui.listWidget_Server_text->addItem(QString("%1").arg(msg));
  ui.listWidget_Server_text->scrollToBottom();
}

void qtros::MainWindow::on_btnGPIO_OFF_clicked()
{
  QString msg = "GPIO OFF!";
  tcpSocket->write(QString("GPIO_OUT_OFF\n").toLocal8Bit().constData());
  ui.listWidget_Server_text->addItem(QString("%1").arg(msg));
  ui.listWidget_Server_text->scrollToBottom();
}

void qtros::MainWindow::Thread_point()
{
  bool grip = false;
  while(1)
  {
    if(complete_pose && grip != true)
    {
      if(qnode._detectEnableGripper == 1)
      {
        QString msg = "GPIO ON!";
        tcpSocket->write(QString("GPIO_OUT_ON\n").toLocal8Bit().constData());
        ui.listWidget_Server_text->addItem(QString("%1").arg(msg));
        ui.listWidget_Server_text->scrollToBottom();
        usleep(10);
        grip = true;
      }
      else
      {
        QString msg = "GPIO OFF!";
        tcpSocket->write(QString("GPIO_OUT_OFF\n").toLocal8Bit().constData());
        ui.listWidget_Server_text->addItem(QString("%1").arg(msg));
        ui.listWidget_Server_text->scrollToBottom();
        usleep(10);
      }

      if(grip != true)
      {
        geometry_msgs::Pose target_pose;
        target_pose.position.x = qnode._detectPixelObject_x;
        target_pose.position.y = qnode._detectPixelObject_y;
        if(qnode._detectEnabledepth == 1 && qnode._detectPixelObject_y < 20)
        {
          target_pose.position.z = 135;
          qnode._detectPixelObject_z = 135;
        }
        else
        {
          target_pose.position.z = 200;
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
        path_constraints.joint_constraints.push_back(joint_4);
        qnode.move_group->setPathConstraints(path_constraints);

        const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
        found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.01); //0.01s

        if (found_ik)
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
        }

        else
        {
          ROS_INFO("Do not found IK!");
        }
          // make file
      }

      else
      {
        usleep(1000000);


        const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
        moveit::core::RobotStatePtr current_state = qnode.move_group->getCurrentState();
        std::vector<double> joint_group_positions;
        current_state->copyJointGroupPositions(joint_model_group,joint_group_positions);
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

      QFile file("RobotJoint_v2.txt");
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

void qtros::MainWindow::Check_pixel()
{
  while(1)
  {
    QFile file("camshift_Kalman_pixel.txt");
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text | QFile::Append))
      return;
    QTextStream out(&file);
    QString Data = QString("%1,%2,%3,%4").arg(qnode._detectPixelObject_x).arg(qnode._detectPixelObject_y).arg(qnode._detectNKalmanObject_x).arg(qnode._detectNKalmanObject_y);
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

  if(check_pixel_enable)
  {
    check_pixel_enable = false;
  }
  else
  {
   pthread_cancel(th2.native_handle());
  }

}

void qtros::MainWindow::on_btnRepeat_clicked()
{
  boost::thread th3(&MainWindow::Repeat, this);

}
void qtros::MainWindow::Repeat()
{
  int turn = 0;
  int i = 0;
  complete_pose = false;

  while(i<30)
  {
      switch (turn) {
      case 0:
      {
        const robot_state::JointModelGroup *joint_model_group = qnode.move_group->getCurrentState()->getJointModelGroup(qnode.PLANNING_GROUP);
        moveit::core::RobotStatePtr current_state = qnode.move_group->getCurrentState();
        std::vector<double> joint_group_positions;
        current_state->copyJointGroupPositions(joint_model_group,joint_group_positions);
        joint_group_positions[0] = -69.6471*pi/180;
        joint_group_positions[1] = -3.4751*pi/180;
        joint_group_positions[4] = -95.536*pi/180;
        joint_group_positions[7] = 81.0136*pi/180;
        joint_group_positions[8] = -69.7951*pi/180;
        joint_group_positions[9] = 179.813*pi/180;
        qnode.move_group->setJointValueTarget(joint_group_positions);
      //  qnode.move_group->move();

         QString Joint_data = QString("J1%1J2%2J3%3J4%4J5%5J6%6").arg(joint_group_positions[0]*180/pi).arg(joint_group_positions[1]*180/pi).arg(joint_group_positions[4]*180/pi)
             .arg(joint_group_positions[7]*180/pi).arg(joint_group_positions[8]*180/pi).arg(joint_group_positions[9]*180/pi);
        tcpSocket->write(Joint_data.toLocal8Bit().constData());
        while(true)
        {
          if(complete_pose)
            break;
        }
        complete_pose = false;
        turn = 1;
        break;
      }

      case 1:
      {
        geometry_msgs::Pose target_pose;
        target_pose.position.x = 300;
        target_pose.position.y = 450;
        target_pose.position.z = 150;
        double rotate_x = 0*pi/180;
        double rotate_y = 180*pi/180;
        double rotate_z = 180*pi/180;


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
        target_pose.position.x = 300;
        target_pose.position.y = 450;
        target_pose.position.z = 30;
        double rotate_x = 0*pi/180;
        double rotate_y = 180*pi/180;
        double rotate_z = 180*pi/180;


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
          usleep(2000000);
          while(true)
          {
            if(complete_pose)
              break;
          }
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
        target_pose.position.x = 300;
        target_pose.position.y = 450;
        target_pose.position.z = 150;
        double rotate_x = 0*pi/180;
        double rotate_y = 180*pi/180;
        double rotate_z = 180*pi/180;


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
