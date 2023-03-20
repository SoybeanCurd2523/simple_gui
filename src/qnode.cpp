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

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <sstream>
#include "../include/simple_gui/qnode.hpp"

#include <sensor_msgs/Joy.h> // jh
/*****************************************************************************
** Namespaces
*****************************************************************************/


//QImage qt_image_screenshot; //add

namespace simple_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

//bool camera_is_running; // jh
//extern COMMAND comm;
int State[8];
bool new_button_State[3];
int new_button_State_count[3];
//int Arm_State[5];
int Ready;
QImage qt_image;
QImage qt_image_top;
QImage qt_image_tpf;
//QImage qt_image_screenshot; //add

float axes[8] = {0.0};
int buttons[13] = {0};
extern bool slam_map_is_off = false;
extern bool reload_ = false;
extern bool closehtml_ = false;

extern int rov_command; // jh
extern int led_command;
extern int camera_command;

extern bool is_shutdown;

extern int ros_topic_data;
extern bool ros_status_flag;
extern bool ros_status_flag_cmd;
extern QString q_command_string;

std_msgs::UInt16 msg;
std_msgs::String cmd_msg;

std_msgs::UInt16 Autodriving_state;
std_msgs::UInt16 Door_state;
std_msgs::UInt16 Obstacle_state;
std_msgs::UInt16 Parking_state;
std_msgs::UInt16 Stair_state;

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
  {
  ROS_INFO("QNode constructor");
}

//COMMAND::COMMAND(){
//  ROS_INFO("COMMAND constructor");
//}
//COMMAND::~COMMAND(){}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
  ros::init(init_argc,init_argv,"simple_gui");
  ROS_INFO("init()");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
//        mission_publisher = n.advertise<std_msgs::UInt16>("mission", 1);
//        command_publisher = n.advertise<std_msgs::String>("gui_terminal_command", 1);

//        Reload_publisher = n.advertise<std_msgs::Bool>("/html_reload", 1);
//        Html_close_publisher = n.advertise<std_msgs::Bool>("/html_close", 1);

//        Slam_map_is_on_publisher = n.advertise<std_msgs::Bool>("/slam_image_is_off", 1);


//        get_Ready_subscriber = n.subscribe("getmission_ready", 1000, &QNode::getready_Callback, this);

      ///////////////////



        rov_publisher = n.advertise<std_msgs::Int32>("rov_command", 1);
        led_publisher = n.advertise<std_msgs::Int32>("led_command", 1);
        camera_publisher = n.advertise<std_msgs::Int32>("camera_command", 1);




    //////////////////////////////

        /*
        A_state_subscriber = n.subscribe("autodriving_state", 1000,  &QNode::A_state_Callback, this); //mission state
        D_state_subscriber = n.subscribe("door_state", 1000,  &QNode::D_state_Callback, this);
        O_state_subscriber = n.subscribe("obstacle_state", 1000,  &QNode::O_state_Callback, this);
        S_state_subscriber = n.subscribe("stair_state", 1000,  &QNode::S_state_Callback, this);
        P_state_subscriber = n.subscribe("parking_state", 1000,  &QNode::P_state_Callback, this);
        MD_state_subscriber = n.subs
  digitalWrite(LED_BUILTIN, HIGH);
}

void led_callback(const std_msgs::Int32& msg){
  servo.writeMicroseconds(1500);
  digitalWrite(LED_BUILTIN, LOW);
}

void camera_callback(const std_msgs::Int32& msg){
  servo.writeMicroseconds(1490);
}

ros::Subscriber<std_msgs::Int32> rov_sub("rov_command", rov_callback);
ros::Subscriber<std_msgs::Int32> led_sub("led_command", led_callback);
ros::Subscriber<std_msgs::Int32> camera_sub("camera_command", camera_callback);


void setup(){
  pinMode(LED_BUILTIN, OUTPUT); // led pin
  servo.attach(servoPin); // thruster pin number
  servo.writeMicroseconds(1500); // init

  nh.initNode();
  nh.subscribe(rov_sub);
  nh.subscribe(led_sub);
  nh.subscribe(camera_sub);
cribe("md_driver_status", 1000, &QNode::MD_state_Callback, this);
        JOY_state_subscriber = n.subscribe("rosjoy_status", 1000, &QNode::JOY_state_Callback, this);
        */
        joy_subscriber = n.subscribe("joy", 1000, &QNode::joyCallback, this);
        Front_Image_subscriber = n.subscribe("/usb_cam/image_raw",1000,&QNode::Front_ImageCb, this); // changed usb_cam_1 to usb_cam
//        Top_Image_subscriber = n.subscribe("/topCamera_1/image",1000,&QNode::Top_ImageCb, this);
//        Tpf_Image_subscriber = n.subscribe("/tpfCamera_1/image",1000,&QNode::Tpf_ImageCb, this);

//        html_is_on_subscriber = n.subscribe("/html_is_on",500,&QNode::html_is_on_Cb, this);


        //Arm
/*
        nuc2_status_subscriber = n.subscribe("/getmission_arm_ready",100,&QNode::nuc_status_Callback, this);


        Arm_status_subscriber = n.subscribe("/arm_status/arm",1000,&QNode::Arm_status_Callback, this);
        Arm_joy_status_subscriber = n.subscribe("/arm_status/joy",1000,&QNode::Arm_joy_status_Callback, this);
        Arm_key_status_subscriber = n.subscribe("/arm_status/key_sub",1000,&QNode::Arm_key_status_Callback, this);
        Arm_service_status_subscriber = n.subscribe("/arm_status/service",1000,&QNode::Arm_service_status_Callback, this);
*/
        //screenshot
        //screenshot_subscriber = n.subscribe("/screenshot/image_raw",1000,&QNode::screenshot_Callback, this);
        //tele_onoff(자율주행 or 조종)
        //teleop_onoff_subscriber = n.subscribe("/teleop_onoff",1000,&QNode::teleop_onoff_Callback, this);

	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
  ROS_INFO("init(parameter)");
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
  ros::init(remappings,"simple_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
//        mission_publisher = n.advertise<std_msgs::UInt16>("mission", 1);
//        command_publisher = n.advertise<std_msgs::String>("gui_terminal_command", 1);

//        Reload_publisher = n.advertise<std_msgs::Bool>("/html_reload", 1);
//        Html_close_publisher = n.advertise<std_msgs::Bool>("/html_close", 1);
//        Slam_map_is_on_publisher = n.advertise<std_msgs::Bool>("/slam_image_is_off", 1);


//        get_Ready_subscriber = n.subscribe("getmission_ready", 1000, &QNode::getready_Callback, this);

        //nuc2_status_subscriber = n.subscribe("/getmission_arm_ready",100,&QNode::nuc_status_Callback, this);

  ///////////////////

  joy_subscriber = n.subscribe("joy", 1000, &QNode::joyCallback, this);
  rov_publisher = n.advertise<std_msgs::Int32>("rov_command", 1);
  led_publisher = n.advertise<std_msgs::Int32>("led_command", 1);
  camera_publisher = n.advertise<std_msgs::Int32>("camera_command", 1);





//////////////////////////////

        /*
        A_state_subscriber = n.subscribe("autodriving_state", 1000,  &QNode::A_state_Callback, this); //mission state
        D_state_subscriber = n.subscribe("door_state", 1000,  &QNode::D_state_Callback, this);
        O_state_subscriber = n.subscribe("obstacle_state", 1000,  &QNode::O_state_Callback, this);
        S_state_subscriber = n.subscribe("stair_state", 1000,  &QNode::S_state_Callback, this);
        P_state_subscriber = n.subscribe("parking_state", 1000,  &QNode::P_state_Callback, this);
        MD_state_subscriber = n.subscribe("md_driver_status", 1000, &QNode::MD_state_Callback, this);
        JOY_state_subscriber = n.subscribe("r￩osjoy_status", 1000, &QNode::JOY_state_Callback, this);
        */
        Front_Image_subscriber = n.subscribe("/usb_cam/image_raw",1000,&QNode::Front_ImageCb, this);
//        Top_Image_subscriber = n.subscribe("/topCamera_1/image",1000,&QNode::Top_ImageCb, this);
//        Tpf_Image_subscriber = n.subscribe("/tpfCamera_1/image",1000,&QNode::Tpf_ImageCb, this);

//        html_is_on_subscriber = n.subscribe("/html_is_on",500,&QNode::html_is_on_Cb, this);

        //Arm

        //nuc2_status_subscriber = n.subscribe("/getmission_arm_ready",100,&QNode::nuc_status_Callback, this);
/*
        Arm_status_subscriber = n.subscribe("/arm_status/arm",1000,&QNode::Arm_status_Callback, this);
        Arm_joy_status_subscriber = n.subscribe("/arm_status/joy",1000,&QNode::Arm_joy_status_Callback, this);
        Arm_key_status_subscriber = n.subscribe("/arm_status/key_sub",1000,&QNode::Arm_key_status_Callback, this);
        Arm_service_status_subscriber = n.subscribe("/arm_status/service",1000,&QNode::Arm_service_status_Callback, this);
        */
        //screenshot
        //screenshot_subscriber = n.subscribe("/screenshot/image_raw",1000,&QNode::screenshot_Callback, this);
        //tele_onoff(자율주행 or 조종)
        //teleop_onoff_subscriber = n.subscribe("/teleop_onoff",1000,&QNode::teleop_onoff_Callback, this);


	start();
	return true;
}

void QNode::run() {
        ROS_INFO("run()");

        ros::Rate loop_rate(50);
        ros::NodeHandle n;
        //image_transport::ImageTransport it(n);

//        get_Ready_subscriber = n.subscribe("getmission_ready", 1000, &QNode::getready_Callback, this);

        ///////////////////

        joy_subscriber = n.subscribe("joy", 1000, &QNode::joyCallback, this);
        rov_publisher = n.advertise<std_msgs::Int32>("rov_command", 1);
        led_publisher = n.advertise<std_msgs::Int32>("led_command", 1);
        camera_publisher = n.advertise<std_msgs::Int32>("camera_command", 1);




      //////////////////////////////

/*
        A_state_subscriber = n.subscribe("autodriving_state", 1000,  &QNode::A_state_Callback, this); //mission state
        D_state_subscriber = n.subscribe("door_state", 1000,  &QNode::D_state_Callback, this);
        O_state_subscriber = n.subscribe("obstacle_state", 1000,  &QNode::O_state_Callback, this);
        S_state_subscriber = n.subscribe("stair_state", 1000,  &QNode::S_state_Callback, this);
        P_state_subscriber = n.subscribe("parking_state", 1000,  &QNode::P_state_Callback, this);
        MD_state_subscriber = n.subscribe("md_driver_status", 1000, &QNode::MD_state_Callback, this);
        JOY_state_subscriber = n.subscribe("rosjoy_status", 1000, &QNode::JOY_state_Callback, this);
        */
        Front_Image_subscriber = n.subscribe("/usb_cam/image_raw",1000,&QNode::Front_ImageCb, this);

//        Top_Image_subscriber = n.subscribe("/topCamera_1/image",1000,&QNode::Top_ImageCb, this);
//        Tpf_Image_subscriber = n.subscribe("/tpfCamera_1/image",1000,&QNode::Tpf_ImageCb, this);

//        html_is_on_subscriber = n.subscribe("/html_is_on",500,&QNode::html_is_on_Cb, this);

        //Front_Image_subscriber2 = it.subscribe("/image_raw/compressed",1000,&QNode::Front_ImageCb,image_transport::TransportHints("compressed"),this);

        //Arm

       // nuc2_status_subscriber = n.subscribe("/getmission_arm_ready",100,&QNode::nuc_status_Callback, this);
/*
        Arm_status_subscriber = n.subscribe("/arm_status/arm",1000,&QNode::Arm_status_Callback, this);
        Arm_joy_status_subscriber = n.subscribe("/arm_status/joy",1000,&QNode::Arm_joy_status_Callback, this);
        Arm_key_status_subscriber = n.subscribe("/arm_status/key_sub",1000,&QNode::Arm_key_status_Callback, this);
        Arm_service_status_subscriber = n.subscribe("/arm_status/service",1000,&QNode::Arm_service_status_Callback, this);
        */
        //screenshot
        //screenshot_subscriber = n.subscribe("/screenshot/image_raw",1000,&QNode::screenshot_Callback, this);
        //tele_onoff(자율주행 or 조종)
        //teleop_onoff_subscriber = n.subscribe("/teleop_onoff",1000,&QNode::teleop_onoff_Callback, this);


  //int count = 0;

//        std_msgs::Bool reload_msg;
//        reload_msg.data = false;

//        std_msgs::Bool close_msg;
//        close_msg.data = false;

//        std_msgs::Bool slam_image_is_off_msg;
//        slam_image_is_off_msg.data = slam_map_is_off;

          std_msgs::Int32 rov_msg;
          std_msgs::Int32 led_msg;
          std_msgs::Int32 camera_msg;

        while ( ros::ok() ) {

          /*
            if(ros_status_flag == true) {
                msg.data = ros_topic_data;
                //mission_publisher.publish(msg);
//                reload_msg.data = reload_;
//                close_msg.data = closehtml_;
//                Reload_publisher.publish(reload_msg);
//                Html_close_publisher.publish(close_msg);
//                reload_ = false;
//                closehtml_ = false;

//                slam_image_is_off_msg.data = slam_map_is_off;
//                Slam_map_is_on_publisher.publish(slam_image_is_off_msg);

                ros_status_flag = false;
            }
            if(ros_status_flag_cmd == true){
              //cmd_msg.data = q_comm￣and_string.toStdString();
              //command_publisher.publish(cmd_msg);
              //ros_status_flag_cmd = false;

            }
            blackout_count();
            blackout(0);
            */


//            Forward_msg.data = command.command_state;
//            ROS_INFO("command_state : ", command);
//            Forward_publisher.publish(Forward_msg);


          if(is_shutdown == false){
            rov_msg.data = rov_command;
            led_msg.data = led_command;
            camera_msg.data = camera_command;
          }
          else{
            rov_msg.data = -1;
            led_msg.data = -1;
            camera_msg.data = -1;
          }


            rov_publisher.publish(rov_msg);
            led_publisher.publish(led_msg);
            camera_publisher.publish(camera_msg);

            ros::spinOnce();
            loop_rate.sleep();
            //++count;
        }

        State[0] = 0;

//        ros::spin();
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

/*
void QNode::A_state_Callback(const std_msgs::UInt16& state_msg){
    State[0] = state_msg.data;
        Q_EMIT statusUpdated();
}

void QNode::D_state_Callback(const std_msgs::UInt16& state_msg){
    State[1] = state_msg.data;
        Q_EMIT statusUpdated();
}

void QNode::O_state_Callback(const std_msgs::UInt16& state_msg){
    State[2] = state_msg.data;
        Q_EMIT statusUpdated();
}

void QNode::P_state_Callback(const std_msgs::UInt16& state_msg){
    State[3] = state_msg.data;
        Q_EMIT statusUpdated();
}

void QNode::S_state_Callback(const std_msgs::UInt16& state_msg){
    State[4] = state_msg.data;
        Q_EMIT statusUpdated();
}

void QNode::MD_state_Callback(const std_msgs::UInt16& state_msg){
    State[5] = state_msg.data;
        Q_EMIT statusUpdated();
}

void QNode::JOY_state_Callback(const std_msgs::UInt16& state_msg){
    State[6] = state_msg.data;
        Q_EMIT statusUpdated();
}
*/
void QNode::blackout(int a){

    for(int i=0; i<3; i++) {
        State[i] = a;
        if(new_button_State_count[i]>30) //about 1 sec
          new_button_State[i] = false;
    }
    //for(int i=0;i<5;i++){
     // Arm_State[i] = a;
    //}
    Ready = a;

    Q_EMIT statusUpdated();
}

void QNode::blackout_count(){

    for(int i=0; i<3; i++) {
        new_button_State_count[i]++;
    }
}

void QNode::getready_Callback(const std_msgs::UInt16& ready){
    Ready = ready.data;
        Q_EMIT statusUpdated();
}

void QNode::Front_ImageCb(const sensor_msgs::ImageConstPtr& msg){ //ImageConstPtr
//  camera_is_running = true;
//  ROS_INFO("camera_is_running = true");
  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
   }
   catch (cv_bridge::Exception& e){
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
   }
   cv::Mat frame2 = cv_ptr->image;
   //cv::Mat frame2;
   //cv::resize(frame,frame2,cv::Size(640, 480),0,0,cv::INTER_CUBIC);
   //cv::Mat frame3 = cv_ptr->image;
   //cv::Mat frame4;
   //cv::cvtColor(frame3, frame4, cv::COLOR_RGB2BGR);
   //cv::cvShowImage("Received Image", &frame);
   //cv::imshow("aaaa",frame);
   int WIDTH = 1280;
   int HEIGHT = 720;
   qt_image = QImage((const unsigned char*)(frame2.data),frame2.cols,frame2.rows,QImage::Format_RGB888).scaled(WIDTH,HEIGHT,Qt::KeepAspectRatio, Qt::SmoothTransformation);
  //qt_image = qt_image.scaled(600,500,Qt::KeepAspectRatio, Qt::SmoothTransformation);
   Q_EMIT statusUpdated();
//   ROS_INFO("A");
}

void QNode::Top_ImageCb(const sensor_msgs::ImageConstPtr& msg){ //ImageConstPtr
  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
   }
   catch (cv_bridge::Exception& e){
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
   }
   cv::Mat frame2 = cv_ptr->image;
   //cv::Mat frame2;
   //cv::resize(frame,frame2,cv::Size(640, 480),0,0,cv::INTER_CUBIC);
   //cv::Mat frame3 = cv_ptr->image;
   //cv::Mat frame4;
   //cv::cvtColor(frame3, frame4, cv::COLOR_RGB2BGR);
   //cv::cvShowImage("Received Image", &frame);
   //cv::imshow("aaaa",frame);
   int WIDTH = 320*(1.4);
   int HEIGHT = 240*(1.4);
   qt_image_top = QImage((const unsigned char*)(frame2.data),frame2.cols,frame2.rows,QImage::Format_RGB888).scaled(WIDTH,HEIGHT,Qt::KeepAspectRatio, Qt::SmoothTransformation);
  //qt_image = qt_image.scaled(600,500,Qt::KeepAspectRatio, Qt::SmoothTransformation);
   Q_EMIT statusUpdated();

}

void QNode::Tpf_ImageCb(const sensor_msgs::ImageConstPtr& msg){ //ImageConstPtr
  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
   }
   catch (cv_bridge::Exception& e){
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
   }
   cv::Mat frame2 = cv_ptr->image;
   //cv::Mat frame2;
   //cv::resize(frame,frame2,cv::Size(640, 480),0,0,cv::INTER_CUBIC);
   //cv::Mat frame3 = cv_ptr->image;
   //cv::Mat frame4;
   //cv::cvtColor(frame3, frame4, cv::COLOR_RGB2BGR);
   //cv::cvShowImage("Received Image", &frame);
   //cv::imshow("aaaa",frame);
   int WIDTH = 320*(1.4);
   int HEIGHT = 240*(1.4);
   qt_image_tpf = QImage((const unsigned char*)(frame2.data),frame2.cols,frame2.rows,QImage::Format_RGB888).scaled(WIDTH,HEIGHT,Qt::KeepAspectRatio, Qt::SmoothTransformation);
  //qt_image = qt_image.scaled(600,500,Qt::KeepAspectRatio, Qt::SmoothTransformation);
   Q_EMIT statusUpdated();

}

void QNode::html_is_on_Cb(const std_msgs::Bool& msg){
  new_button_State[0]=true;
  new_button_State_count[0]=0; //msg.data;
  Q_EMIT statusUpdated();
}

/*
void QNode::nuc_status_Callback(const std_msgs::Bool& state_msg){
    Arm_State[4] = state_msg.data;
        Q_EMIT statusUpdated();
}


void QNode::Arm_status_Callback(const std_msgs::Bool& state_msg){
    Arm_State[0] = state_msg.data;
        Q_EMIT statusUpdated();
}
void QNode::Arm_joy_status_Callback(const std_msgs::Bool& state_msg){
    Arm_State[1] = state_msg.data;
        Q_EMIT statusUpdated();
}
void QNode::Arm_key_status_Callback(const std_msgs::Bool& state_msg){
    Arm_State[2] = state_msg.data;
        Q_EMIT statusUpdated();
}
void QNode::Arm_service_status_Callback(const std_msgs::Bool& state_msg){
    Arm_State[3] = state_msg.data;
        Q_EMIT statusUpdated();
}
*/
/*
void QNode::screenshot_Callback(const sensor_msgs::Image& msg){
  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
   }
   catch (cv_bridge::Exception& e){
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
   }
   cv::Mat frame2 = cv_ptr->image;
   //cv::Mat frame2;
   //cv::resize(frame,frame2,cv::Size(640, 480),0,0,cv::INTER_CUBIC);
   //cv::Mat frame3 = cv_ptr->image;
   //cv::Mat frame4;
   //cv::cvtColor(frame3, frame4, cv::COLOR_RGB2BGR);
   //cv::cvShowImage("Received Image", &frame);
   //cv::imshow("aaaa",frame);
   int WIDTH = 320*(3);
   int HEIGHT = 180*(3);
   qt_image_screenshot = QImage((const unsigned char*)(frame2.data),frame2.cols,frame2.rows,QImage::Format_RGB888).scaled(WIDTH,HEIGHT,Qt::KeepAspectRatio, Qt::SmoothTransformation);
  //qt_image = qt_image.scaled(600,500,Qt::KeepAspectRatio, Qt::SmoothTransformation);
   Q_EMIT statusUpdated_sc();
}
*/
void QNode::teleop_onoff_Callback(const std_msgs::Int8& msg){
  State[7] = msg.data;
  Q_EMIT statusUpdated();

}



void QNode::log( const LogLevel &level, const std::string &msg) {
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

void QNode::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  using namespace std;
//  ROS_INFO("joyCallback()");

  for(int i=0 ; i< 8 ; i++){
    axes[i] = joy->axes[i];
  }

  for(int i=0 ; i< 13 ; i++){
    buttons[i] = joy->buttons[i];
  }

  if(buttons[0] == 1.0){
    ROS_INFO("joystick_stop");
    rov_command = 0;
    led_command = 1;
    camera_command = 0;
  }

  else if(axes[1] == 1.0){
    ROS_INFO("joystick_rov_forward");
    rov_command =1;
  }
  else if(axes[0] == 1.0){
    ROS_INFO("joystick_rov_left");
    rov_command = 2;
  }
  else if(axes[0] == -1.0){
    ROS_INFO("joystick_rov_right");
    rov_command =3;
  }
  else if(axes[1] == -1.0){
    ROS_INFO("joystick_rov_backward");
    rov_command =4;
  }

  else if(axes[3] == 1.0){
    ROS_INFO("joystick_turn_left");
    rov_command =5;
  }
  else if(axes[3] == -1.0){
    ROS_INFO("joystick_turn_right");
    rov_command =6;
  }
  else if(axes[4] == 1.0){
    ROS_INFO("joystick_rov_up");
    rov_command =7;
  }
  else if(axes[4] == -1.0){
    ROS_INFO("joystick_rov_down");
    rov_command =8;
  }

// led on, off는 눌러서 하는게 좋을듯. 고정이니까.
  else if(axes[6] == 1.0){
    ROS_INFO("joystick_camera_left");
    camera_command = 1;
  }
  else if(axes[6] == -1.0){
    ROS_INFO("joystick_camera_right");
    camera_command = 2;
  }
  else if(axes[7] == 1.0){
    ROS_INFO("joystick_camera_up");
    camera_command = 3;
  }
  else if(axes[7] == -1.0){
    ROS_INFO("joystick_camera_down");
    camera_command = 4;
  }
  else if(abs(axes[6]) < 0.2 and abs(axes[7]) < 0.2 and abs(buttons[4]) < 0.2){
    camera_command = 0;
  }

  else if(abs(buttons[4]) > 0.8){
    camera_command = 5;
  }


}
}  // namespace simple_gui




//md_driver_status int number 1
