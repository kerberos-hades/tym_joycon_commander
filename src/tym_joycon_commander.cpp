#include <tym_joycon_commander.hpp>
/**
 * @brief ジョイコンのコールバック関数
 * 
 * @param msg ジョイコンの状態を表すメッセージ
 */
void JoyConCommander::joyConCallback(const sensor_msgs::JoyConstPtr &msg)
{
    msg_cmd_vel.angular.z = setAngFromJoyCon(msg);
    msg_cmd_vel.linear.x = setVelFromJoyCon(msg);
}
/**
 * @brief すべてのメッセージをPublishする
 * 
 */
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
/**
 * @brief argの値をstep(小数点以下3桁まで)ずつ変化させる
 * 
 * @param arg 変化させる値
 * @param step ステップ
 * @return double 変化させた後の値
 */
double JoyConCommander::stepDecimalNum(double arg, const double step)
{
    // 0.1などの小さな値を足していくのは危ないので1000倍してから演算
    arg *= 1000;
    arg += step * 1000;
    arg /= 1000;
    return arg;
}
/**
 * @brief ジョイコンの左スティックをもとに並進速度を指定
 * 
 * @param joy_con_msg Topic"/joy"のメッセージ
 * @return double 指定すべき速度
 */
double JoyConCommander::setVelFromJoyCon(const sensor_msgs::JoyConstPtr &joy_con_msg)
{
    /*最大速度の値を設定*/
    if (isDown(joy_con_msg->buttons, L2))
    {
        vel = stepDecimalNum(vel, STP_VEL);
    }
    else if (isDown(joy_con_msg->buttons, L1))
    {
        vel = stepDecimalNum(vel, -STP_VEL);
    }
    /*最大速度の最大値と最小値にクリップ*/
    vel = clip(MIN_VEL, vel, MAX_VEL);
    /*ジョイコンのレバーの値をもとに速度を決定して返す*/
    return vel * joy_con_msg->axes[STICK_L];
}
/**
 * @brief ジョイコンの右スティックをもとに角速度を指定
 * 
 * @param joy_con_msg Topic"/joy"のメッセージ
 * @return double 指定すべき角速度
 */
double JoyConCommander::setAngFromJoyCon(const sensor_msgs::JoyConstPtr &joy_con_msg)
{
    /*最大角速度の値を設定*/
    if (isDown(joy_con_msg->buttons, R2))
    {
        ang = stepDecimalNum(ang, STP_ANG);
    }
    else if (isDown(joy_con_msg->buttons, R1))
    {
        ang = stepDecimalNum(ang, -STP_ANG);
    }
    /*最大角速度の最大値と最小値にクリップ*/
    ang = clip(MIN_ANG, ang, MAX_ANG);
    /*ジョイコンのレバーの値をもとに角速度を決定して返す*/
    return ang * joy_con_msg->axes[STICK_R];
}
/**
 * @brief ボタンが押されているかの判定
 * 
 * @param buttons ジョイコンのボタンすべての情報（Vector）
 * @param button ボタンの指定
 * @return true 押されてる
 * @return false 押されてない
 */
bool JoyConCommander::isDown(std::vector<int> buttons, SWITCH_BUTTON button)
{
    if (buttons[button] == UP)
    {
        return false;
    }
    return true;
}
/**
 * @brief JoyConCommanderのコンストラクタ
 * 
 * @param argc main関数のargc
 * @param argv main関数のargv
 */
JoyConCommander::JoyConCommander(int argc, char *argv[])
{
    /*ノード名設定*/
    ros::init(argc, argv, "tym_joy_con_commander");
    /*パラメータの値を変更*/
    ros::NodeHandle pnh("~");
    pnh.getParam("max_vel_double", MAX_VEL);
    pnh.getParam("min_vel_double", MIN_VEL);
    pnh.getParam("max_angvel_double", MAX_ANG);
    pnh.getParam("min_angvel_double", MIN_ANG);
    pnh.getParam("vel_step_double", STP_VEL);
    pnh.getParam("ang_step_double", STP_ANG);

    /*各種パラメータ更新後の初期化*/
    vel = MIN_VEL;
    ang = MIN_ANG;
    /*Publisher,Subscriberの初期化*/
    ros::NodeHandle nh;
    pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000, this);
    sub_joy_con = nh.subscribe("/joy", 10, &JoyConCommander::joyConCallback, this);
}
/**
 * @brief メインループ
 * 
 */
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