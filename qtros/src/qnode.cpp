/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/qtros/qnode.hpp"
#include "find_object_2d/ObjectsStamped.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <QTransform>

#include <pluginlib/class_loader.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <boost/scoped_ptr.hpp>

#endif


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qtros {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
  init_argv(argv)
  {}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init()// init function 1
{
        ros::init(init_argc,init_argv,"qtros"); //init function 2 call
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  ros::NodeHandle nh;
  ros::NodeHandle nh_2;

	// Add your ros communications here.
    //chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
  sub = n.subscribe("joint_states",1000,&QNode::chatterCallback,this);//data
  subObject = nh.subscribe("objects",1000,&QNode::ObjectCallback,this);//data?? where

  chatter_publisher = nh_2.advertise<std_msgs::String>("Endvalue_msg",1000);


  move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
  move_group->setEndEffectorLink("gripper");
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  kinematic_model = robot_model_loader.getModel();
  start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) { // init function 2
        std::map<std::string,std::string> remappings; //
        remappings["__master"] = master_url; //
  remappings["__hostname"] = host_url; //
        ros::init(remappings,"qtros"); //recursive call
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  ros::NodeHandle nh;
  ros::NodeHandle nh_2;


  sub = n.subscribe("joint_states",1000,&QNode::chatterCallback,this);//
  subObject = nh.subscribe("Object",10,&QNode::ObjectCallback,this);

  chatter_publisher = nh_2.advertise<std_msgs::Float32MultiArray>("Endvalue",100);

  sub_detectObject = nh_2.subscribe("FixedCamera_Object",100,&QNode::detectObjectCallback,this); // no code in init function 1
  sub_labeledObject = nh_2.subscribe("/object/center_pos",100, &QNode::detectLabeledCallback,this); // no code in init function 1

  //robot_ model_
  move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
  move_group->setEndEffectorLink("gripper");
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  kinematic_model = robot_model_loader.getModel();
  start();//???
	return true;
}

void QNode::run() {

  ros::Rate loop_rate(10); // 10 hz repeat

  while(ros::ok()) //current postion, orientation save.
  {
    std_msgs::Float32MultiArray array;
    array.data.clear();
    array.data.push_back(_currentX);
    array.data.push_back(_currentY);
    array.data.push_back(_currentZ);
    array.data.push_back(_current_Orientation_X);
    array.data.push_back(_current_Orientation_Y);
    array.data.push_back(_current_Orientation_Z);
    /*std_msgs::String msg;
    QString data = QString("x:%1y:%2z:%3R:%4P:%5Y:%6").arg(_currentX).arg(_currentY).arg(_currentZ).arg(_current_Orientation_X).arg(_current_Orientation_Y).arg(_current_Orientation_Z);
    msg.data = data.toUtf8().constData();*/
    chatter_publisher.publish(array); //publisher name : current_position_publisher
    ros::spinOnce();
    loop_rate.sleep();
  }

    //ros::spin();
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)

}

void QNode::log( const LogLevel &level, const std::string &msg) { //Debug, Info, Warn, Error, Fatal msg extraction.
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::chatterCallback(const sensor_msgs::JointState::ConstPtr& msg) // msg position data -> GetJointValueArr
{
// int joint_num;
// joint_num = msg->position.size();
// const double* joint = msg->position.data();

// double arr[joint_num];
 std::copy(msg->position.begin(),msg->position.end(),GetJointValueArr);
// ROS_INFO("joint=[%f, %f, %f, %f, %f]", arr[0],arr[1],arr[4],arr[7],arr[8]);
}

void QNode::ObjectCallback(const std_msgs::Float32MultiArrayConstPtr& msg)
{
  const std::vector<float> & data= msg->data;
  if(data.size()) //if there is data in the buffer.
  {
    for(unsigned int i=0; i<data.size(); i+=12)
    {
      int id =(int)data[i];
      float objectWidth = data[i+1];
      float objectHeight = data[i+2];

      QTransform qtHomography(data[i+3],data[i+4],data[i+5],data[i+6],data[i+7],data[i+8],data[i+9],data[i+10],data[i+11]);
      QPointF qtTopLeft = qtHomography.map(QPointF(0,0));
      QPointF qtTopRight = qtHomography.map(QPointF(objectWidth,0));
      QPointF qtBottomLeft = qtHomography.map(QPointF(0,objectHeight));
      QPointF qtBottomRight = qtHomography.map(QPointF(objectWidth,objectHeight));
      //ROS_INFO("point: (%f, %f), (%f, %f), (%f, %f), (%f, %f)",qtTopLeft.x(),qtTopLeft.y(),qtTopRight.x(),qtTopRight.y(),qtBottomLeft.x(),qtBottomLeft.y(),qtBottomRight.x(),qtBottomRight.y());

    }
  }
}
void QNode::detectLabeledCallback(const std_msgs::Float32MultiArrayConstPtr& msg)
{

  this->cur_time = ros::Time::now();
  cur_time_double = ros::Time::now().toSec();

  //cout << "cur_time - pre_time: " << cur_time - pre_time <<endl;

  const std::vector<float> & data= msg->data;
  if(data.size())
  {
    _labeledCenter_x = data[0];
    _labeledCenter_y = data[1];

    _expected_2_y = (_pre_labeledCenter_x - _labeledCenter_x)* _dist_pixel_u;

    // Camera position calibration & calculation

    //lab
    //_labeledCenter_calculate_x = 457 - (_labeledCenter_y - 480/2) * _dist_pixel_v; //camera coordinate 457, 471;
    //_labeledCenter_calculate_y = 471 - (_labeledCenter_x - 640/2) * _dist_pixel_u;

    //jinyoung (robot to camera = x : 620, y : 440) -> 190904 / (robot to camera = x : 630, y : 630) -> 190905
     _labeledCenter_calculate_x = 630 - (_labeledCenter_y - 480/2) * _dist_pixel_v; //camera coordinate 457, 471;
     _labeledCenter_calculate_y = 630 - (_labeledCenter_x - 640/2) * _dist_pixel_u;

     _expected_y =  _pre_labeledCenter_calculate_y - _labeledCenter_calculate_y;

     ROS_INFO("pixelObject: %f, %f ",_labeledCenter_x,_labeledCenter_y);
     ROS_INFO("pixelObject_calculate: %f, %f ", _labeledCenter_calculate_x, _labeledCenter_calculate_y);

     ROS_INFO("time_: %f", cur_time_double - pre_time_double);

     ROS_INFO("expected_y: %f  ", _expected_y);

     ROS_INFO("expected_2_y: %f  ", _expected_2_y);

     _pre_labeledCenter_x = _labeledCenter_x;
     _pre_labeledCenter_calculate_y = _labeledCenter_calculate_y;

     velocity_enclosure = _expected_y/(float)(cur_time_double - pre_time_double);

     ROS_INFO("velocity: %f  ", velocity_enclosure);

     pre_time_double = cur_time_double;
  }
  //ROS_INFO_STREAM("cur_time2 : "<< this->cur_time );
  //ROS_INFO_STREAM("pre_time2: "<<  this->pre_time);


}

void QNode::detectObjectCallback(const std_msgs::Float32MultiArrayConstPtr& msg)
{
  const std::vector<float> & data= msg->data;
  if(data.size())
  {
    _detectPixelObject_x = data[0];
    _detectPixelObject_y = data[1];
    _detectPixelObject_degree = data[2];
    _detectEnableGripper = data[3];
    _detectpixel_u = data[4];
    _detectpixel_v = data[5];
    _detectEnabledepth = data[6];
    _detectNKalmanObject_x = data[7];
    _detectNKalmanObject_y = data[8];
     //ROS_INFO("pixelObject: %f %f %f",_detectPixelObject_x,_detectPixelObject_y,_detectPixelObject_degree);
  }
}


}  // namespace qtros
