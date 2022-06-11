#include <tym_joycon_commander.hpp>

void JoyConCommander::joyConCallback(const sensor_msgs::JoyConstPtr &msg)
{
    msg_cmd_vel.angular.z = setAngFromJoyCon(msg);
    msg_cmd_vel.linear.x = setVelFromJoyCon(msg);
}

void JoyConCommander::publish()
{
    pub_cmd_vel.publish(msg_cmd_vel);
}
/**
 * @brief argの値をmin以上max以下に制限する
 * 
 * @param min 最小値
 * @param arg 制限する値
 * @param max 最大値
 * @return double 制限されたarg
 */
double JoyConCommander::clip(double min, double arg, double max)
{
    return min > arg ? min : (max > arg ? arg : max);
}
double JoyConCommander::setVelFromJoyCon(const sensor_msgs::JoyConstPtr &joy_con_msg)
{
}
double JoyConCommander::setAngFromJoyCon(const sensor_msgs::JoyConstPtr &joy_con_msg)
{
}

bool JoyConCommander::isDown(std::vector<int> buttons, SWITCH_BUTTON button)
{
    if (buttons[button] == UP)
    {
        return false;
    }
    return true;
}

JoyConCommander::JoyConCommander(int argc, char *argv[])
{
    ros::init(argc, argv, "tym_joy_con_commander");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    pnh.getParam("max_vel_double", MAX_VEL);
    pnh.getParam("min_vel_double", MIN_VEL);
    pnh.getParam("max_angvel_double", MAX_ANG);
    pnh.getParam("min_angvel_double", MIN_ANG);
    pnh.getParam("vel_step_double", STP_VEL);
    pnh.getParam("ang_step_double", STP_ANG);

    pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000, this);
    sub_joy_con = nh.subscribe("/joy", 10, joyConCallback, this);
}

void JoyConCommander::mainLoop()
{
    ros::Rate loop_rate(10);
    ros::spinOnce();
    while (ros::ok())
    {
        ROS_INFO("vel : %lf", msg_cmd_vel.linear.x);
        ROS_INFO("ang : %lf", msg_cmd_vel.angular.z);
        publish();
        loop_rate.sleep();
    }
}

int main(int argc, char *argv[])
{
    JoyConCommander joy_con_commander(argc, argv);
    joy_con_commander.mainLoop();
    return 0;
}