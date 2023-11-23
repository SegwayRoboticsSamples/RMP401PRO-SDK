#include <math.h>
#include <iostream>
#include <termios.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "segwayrmp/robot.h"



#define PRINTHELP		  'h'
#define ADDLINEVEL		  'w'
#define DECLINEVEL		  's'
#define ADDANGULARVEL	  'a'
#define DECANGULARVEL	  'd'
#define PRINTCURVEL		  'f'
#define VELRESETZERO	  'g'
#define ENABLECMD         'e'
#define CHASSISPAUSE      'q'
#define IAPCHASSIS        'v'
#define IAPMOTORFRONT     'b'
#define IAPMOTORREAR      'n'
#define LOOPFRONTFIRST    'l'
#define LOOPREARFIRST     'K'

int rate = 100;
ros::Publisher cmd_pub;
ros::ServiceServer chassis_send_event_srv_server;

ros::ServiceClient ros_set_chassis_enable_cmd_client;
segway_msgs::ros_set_chassis_enable_cmd ros_set_chassis_enable_srv;
segway_msgs::ros_set_iap_cmdGoal ros_set_iap_cmd_goal;

char const* print_help() {
    char const* printHelp = 
        "\t h : Displays the required keys and their meaning\n"
        "\t w : Increase forward speed by 0.1m/s\n"
        "\t s : Decrease forward speed by 0.1m/s\n"
        "\t a : Increase the angular velocity by 0.1rad/s\n"
        "\t d : Decrease the angular velocity by 0.1rad/s\n"
        "\t f : Displays current speed Settings\n"
        "\t g : Speed reset to zero\n"
        "\t e : Chassis enable switch\n"
        "\t q : Running pause. Click 'q'key again to resume running by the previous speed. W/S/A/D keys can also restore chassis running\n"
        "\t v : Iap the chassis board, please put the bin file in /sdcard/firmware/\n"
        "\t b : Iap the route board, please put the bin file in /sdcard/firmware/\n"
        "\t n : Iap the connect board, please put the bin file in /sdcard/firmware/\n"
        "\t l : Cycle forward and backward for 2 seconds at 0.05m/s. First forward/\n"
        "\t k : Cycle forward and backward for 2 seconds at 0.05m/s. First backward/\n"
        "\t others : Do nothing\n";
    return printHelp;
}

void changemode(int dir)
{
  static struct termios oldt, newt;
 
  if ( dir == 1 )
  {
    tcgetattr( STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);
  }
  else
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
}

int kbhit (void)
{
  struct timeval tv;
  fd_set rdfs;  //一组fd集合
 
  tv.tv_sec = 0;
  tv.tv_usec = 0;   //时间为0，非阻塞
 
  FD_ZERO(&rdfs);   //fd集合清空
  FD_SET (STDIN_FILENO, &rdfs); //增加新的fd
 
  select(STDIN_FILENO+1, &rdfs, NULL, NULL, &tv);   //非阻塞检测fd为0的tty读取状态，rdfs特定位置1
  return FD_ISSET(STDIN_FILENO, &rdfs);     //判断指定的标准输入fd是否在rdfs集合中
}

static int scanKeyboard()
{
    int input_char = 0;
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(0, &stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);  //非规范模式：每次返回单个字符
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(0,&stored_settings);
    new_settings.c_cc[VMIN] = 1;    //读取的最小字节数
    tcsetattr(0, TCSANOW, &new_settings);

    changemode(1);
    if (kbhit()) {
        input_char = getchar();
        printf("\n");
    }
    changemode(0);

    tcsetattr(0, TCSANOW, &stored_settings);
    return input_char;
}

char get_keyboard()
{
    char keyvalue = scanKeyboard();
    return keyvalue;
}

// void goal_response_callback(std::shared_future<goalHandleIapCmd::SharedPtr> future)
// {
//   auto goal_handle = future.get();
//   if (!goal_handle) {
//     RCLCPP_ERROR(rclcpp::get_logger("drive_segway_sample"), "Goal was rejected by server");
//   } else {
//     RCLCPP_INFO(rclcpp::get_logger("drive_segway_sample"), "Goal accepted by server, waiting for result");
//   }
// }

// void feedback_callback(
//   goalHandleIapCmd::SharedPtr,
//   const std::shared_ptr<const iapCmd::Feedback> feedback)
// {
//   RCLCPP_INFO(rclcpp::get_logger("drive_segway_sample"), "IAP process percentage：[%d]", feedback->iap_percent);
// }

// void result_callback(const goalHandleIapCmd::WrappedResult & result)
// {
//   switch (result.code) {
//     case rclcpp_action::ResultCode::SUCCEEDED:
//       break;
//     case rclcpp_action::ResultCode::ABORTED:
//       RCLCPP_ERROR(rclcpp::get_logger("drive_segway_sample"), "Goal was aborted");
//       return;
//     case rclcpp_action::ResultCode::CANCELED:
//       RCLCPP_ERROR(rclcpp::get_logger("drive_segway_sample"), "Goal was canceled");
//       return;
//     default:
//       RCLCPP_ERROR(rclcpp::get_logger("drive_segway_sample"), "Unknown result code");
//       return;
//   }
//   if (result.result->iap_success == 1) {
//     RCLCPP_INFO(rclcpp::get_logger("drive_segway_sample"), "iap success!");
//   } else {
//     RCLCPP_INFO(rclcpp::get_logger("drive_segway_sample"), "iap fail!");
//   }
// }
void iapActionDoneCb(const actionlib::SimpleClientGoalState& state,
                        const segway_msgs::ros_set_iap_cmdResultConstPtr& result)
{
    ROS_INFO_NAMED("drive_sample", "iap result:%d, iap_result_errorcode:%d", result->iap_result, result->error_code);
}

void iapActionActiveCb()
{
   ROS_INFO_NAMED("drive_sample", "iap goal just went active");
}

void iapActionFeedbackCb(const segway_msgs::ros_set_iap_cmdFeedbackConstPtr& feedback)
{
    ROS_INFO_NAMED("drive_sample", "percent_complete : %d", feedback->iap_percent);
}

void drive_chassis_test(iapActionClient& ac)
{
    static uint16_t set_enable_cmd = 1;
    uint8_t enable_switch = 0;
    static double set_line_speed; 
    static double set_angular_speed;
    static double send_line_speed;
    static double send_angular_speed;
    static uint8_t chassis_pause = 0;
    uint8_t iap_flag = 0;
    static uint16_t loopcnt = 0;
    static double looplinespeed = 0.05; // unit: m/s
    static char last_keyvalue = 0;

    segway_msgs::ros_set_chassis_enable_cmd enable_request;

    char keyvalue = get_keyboard();
    switch (keyvalue)
    {
    case PRINTHELP:
        printf("%s\n", print_help());
        break;
    case ADDLINEVEL		:
        set_line_speed += 0.1;
        send_line_speed = set_line_speed;
        send_angular_speed = set_angular_speed;
        printf("current set_line_speed[%lf], set_angular_speed[%lf]\n", set_line_speed, set_angular_speed);
        break;
    case DECLINEVEL		:
        set_line_speed -= 0.1;
        send_line_speed = set_line_speed;
        send_angular_speed = set_angular_speed;
        printf("current set_line_speed[%lf], set_angular_speed[%lf]\n", set_line_speed, set_angular_speed);
        break;
    case ADDANGULARVEL	:
        set_angular_speed += 0.1;
        send_line_speed = set_line_speed;
        send_angular_speed = set_angular_speed;
        printf("current set_line_speed[%lf], set_angular_speed[%lf]\n", set_line_speed, set_angular_speed);
        break;
    case DECANGULARVEL	:
        set_angular_speed -= 0.1;
        send_line_speed = set_line_speed;
        send_angular_speed = set_angular_speed;
        printf("current set_line_speed[%lf], set_angular_speed[%lf]\n", set_line_speed, set_angular_speed);
        break;
    case PRINTCURVEL	:
        printf("current set_line_speed[%lf], set_angular_speed[%lf]\n", set_line_speed, set_angular_speed);
        break;
    case VELRESETZERO	:
        set_line_speed = 0; 
        set_angular_speed = 0;
        send_line_speed = 0;
        send_angular_speed = 0;
        printf("current set_line_speed[%lf], set_angular_speed[%lf]\n", set_line_speed, set_angular_speed);
        break;
    case ENABLECMD      : 
        enable_switch = 1;
        enable_request.request.ros_set_chassis_enable_cmd = set_enable_cmd;
        ++set_enable_cmd;        
        set_enable_cmd %= 2;
        ROS_INFO_NAMED("drive_sample", "enable chassis switch[%d]", enable_request.request.ros_set_chassis_enable_cmd);
        printf("current set_line_speed[%lf], set_angular_speed[%lf]\n", set_line_speed, set_angular_speed);
        break;  
    case CHASSISPAUSE   :
        ++chassis_pause;
        chassis_pause %= 2;
        if (chassis_pause) {
            send_line_speed = 0;
            send_angular_speed = 0;
            printf("Stop the chassis temporarily\n");
        } else {
            send_line_speed = set_line_speed;
            send_angular_speed = set_angular_speed;
            printf("current set_line_speed[%lf], set_angular_speed[%lf]\n", set_line_speed, set_angular_speed);
        }
        break;   
    case IAPCHASSIS:
        ROS_INFO_NAMED("drive_sample", "iap chassis");
        iap_flag = 1;
        break;
    case IAPMOTORFRONT:
        ROS_INFO_NAMED("drive_sample",  "iap motor front");
        iap_flag = 2;
        break;
    case IAPMOTORREAR:
        ROS_INFO_NAMED("drive_sample",  "iap motor rear");
        iap_flag = 3;
        break;
    case LOOPFRONTFIRST:
        set_line_speed = 0; 
        set_angular_speed = 0;        
        send_angular_speed = 0; 
        looplinespeed = 0.05; // unit: m/s
        loopcnt = 0;
        break;
    case LOOPREARFIRST:
        set_line_speed = 0; 
        set_angular_speed = 0;        
        send_angular_speed = 0; 
        looplinespeed = -0.05; // unit: m/s
        loopcnt = 0;
        break;
    default:
        keyvalue = last_keyvalue;
        break;
    }
    last_keyvalue = keyvalue;

    if (last_keyvalue == LOOPFRONTFIRST || last_keyvalue == LOOPREARFIRST) {
        if (++loopcnt < rate * 2){
            send_line_speed = looplinespeed;// unit: m/s
        }
        else {
            looplinespeed = (looplinespeed > 0 ? -fabs(looplinespeed) : fabs(looplinespeed));   
            loopcnt = 0;
        }
    }
    else {
        loopcnt = 0;
    }
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = send_line_speed;
    cmd_vel.angular.z = send_angular_speed;
    cmd_pub.publish(cmd_vel);

    if (enable_switch){
        ros_set_chassis_enable_cmd_client.call(enable_request);
    }

    if (iap_flag & 3) {
        ros_set_iap_cmd_goal.board_index_for_iap = iap_flag;
        ros_set_iap_cmd_goal.board_index_for_iap = 0;
        ac.sendGoal(ros_set_iap_cmd_goal, &iapActionDoneCb, &iapActionActiveCb, &iapActionFeedbackCb);
    }
}

bool ros_get_chassis_send_event_callback(segway_msgs::chassis_send_event::Request &req, segway_msgs::chassis_send_event::Response &res)
{
    ROS_INFO("The ROS test node receives the event ID:%d", req.chassis_send_event_id);
    switch (req.chassis_send_event_id)
    {
    case OnEmergeStopEvent:
        ROS_INFO_NAMED("drive_sample", "CHASSIS EVENT: OnEmergeStopEvent");
        break;
    case OutEmergeStopEvent:
        ROS_INFO_NAMED("drive_sample", "CHASSIS EVENT: OutEmergeStopEvent");
        break;   
    default:
        break;
    }    
    res.ros_is_received = true;
    return true;
}

int main(int argc, char * argv[])
{
/////////////////////////////////////////////////////////////////////////////////////////
    ros::init(argc, argv, "drive_segway_sample");
    ros::NodeHandle n_;

    //chassis
    cmd_pub = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros_set_chassis_enable_cmd_client = n_.serviceClient<segway_msgs::ros_set_chassis_enable_cmd>("ros_set_chassis_enable_cmd_srv");

    chassis_send_event_srv_server = n_.advertiseService("chassis_send_event_srv", &ros_get_chassis_send_event_callback);

    iapActionClient ac("ros_set_iap_cmd_action", true);
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started.");

    ros::Rate loop_rate(rate);
    printf("%s\n", print_help());
    while (ros::ok()) {
        drive_chassis_test(ac);
  
        loop_rate.sleep();
        ros::spinOnce();
    }
    
    return 0;
}