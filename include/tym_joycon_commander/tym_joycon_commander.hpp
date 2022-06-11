#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

/**
 * @brief ジョイコンを使用してロボットを動かすためのクラス
 *
 */
class JoyConCommander
{
private:
    /*Switch JoyConのボタン配置*/
    enum SWITCH_BUTTON
    {
        DOWN = 1,    //押されている
        UP = 0,      //押されてない
        L2 = 6,      //並進速度減速
        L1 = 4,      //並進速度加速
        R2 = 7,      //角速度減速
        R1 = 5,      //角速度加速
        STICK_L = 1, //左スティック
        STICK_R = 2  //右スティック
    };

    /*パラメータ定数*/
    double MAX_VEL = 0.5;
    double MIN_VEL = 0.1;
    double MAX_ANG = 0.3;
    double MIN_ANG = 0.1;
    double STP_VEL = 0.1;
    double STP_ANG = 0.05;

    /*変数*/
    double vel;
    double ang;

    /*ros publish用*/
    geometry_msgs::Twist msg_cmd_vel;
    ros::Publisher pub_cmd_vel;

    /*ros subscriber用*/
    ros::Subscriber sub_joy_con;

    /*内部処理*/
    void joyConCallback(const sensor_msgs::JoyConstPtr &msg);
    double clip(double min, double arg, double max);
    double stepDecimalNum(double arg, const double step);
    double setVelFromJoyCon(const sensor_msgs::JoyConstPtr &joy_con_msg);
    double setAngFromJoyCon(const sensor_msgs::JoyConstPtr &joy_con_msg);
    void publish(void);
    bool isDown(std::vector<int> buttons, SWITCH_BUTTON button);

public:
    JoyConCommander(int argc, char *argv[]);
    void mainLoop();
};
