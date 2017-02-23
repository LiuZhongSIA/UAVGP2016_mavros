/**
* @file       : missionABA_node.cpp
* @brief      : take off --> pos A --> hover --> pos B --> hover --> pos A --> land
* @author     : liu zhong
* @time       : 2016/02/23
*/

#include <ros/ros.h>
#include <math.h>
#include <mavros/frame_tf.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeRad.h>

// 定义变量，代表状态机状态
static const int NONE = 0;
static const int TAKE_OFF = 1;
static const int POS_A = 2;
static const int A_HOVER = 3;
static const int POS_B = 4;
static const int B_HOVER = 5;
static const int POS_A_Re = 6;
static const int LAND = 7;
// 当前的状态机状态
int current_pos_state = NONE;

// 设置A、B点位置变量，将通过ros发送出去
geometry_msgs::PoseStamped pose_a;
geometry_msgs::PoseStamped pose_b;
// 配置ros service，用于飞机解锁
mavros_msgs::CommandBool arm_cmd;
// 配置ros service，用于改变模式
mavros_msgs::SetMode offb_set_mode;
// 配置ros service，用于降落
mavros_msgs::CommandTOL landing_cmd;
// 订阅“状态”消息，以及相应回调函数
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
// 订阅“位置”消息，以及相应的回调函数
geometry_msgs::PoseStamped current_pos;
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pos = *msg;
}
// 订阅"姿态"消息，以及相应的回调函数
mavros_msgs::AttitudeRad current_att;
void att_cb(const mavros_msgs::AttitudeRad::ConstPtr& msg){
    current_att = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "missionABA_node"); //节点名称
    ros::NodeHandle nh; //ros节点句柄

    ros::Time stay_A = ros::Time::now();
    ros::Time stay_B = ros::Time::now();
    ros::Time last_request = ros::Time::now();

    // 用于发布位置期望
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    // 发布service msg，用于解锁
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    // 设置飞行模式
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    // 起降
    ros::ServiceClient tol_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    // 用于消息订阅
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pos_cb);
    ros::Subscriber att_sub = nh.subscribe<mavros_msgs::AttitudeRad>("mavros/attitude_rad", 10, att_cb);

    // --初始化变量--
    // A点位置，东北天
    pose_a.pose.position.x = 0;
    pose_a.pose.position.y = 0;
    pose_a.pose.position.z = 5;
    // B点位置
    pose_b.pose.position.x = 0;
    pose_b.pose.position.y = 5;
    pose_b.pose.position.z = 5;
    // 航向期望
    auto quat_yaw = mavros::ftf::quaternion_from_rpy(0.0, 0.0, 0.0);
    pose_a.pose.orientation.x = quat_yaw.x();
    pose_a.pose.orientation.y = quat_yaw.y();
    pose_a.pose.orientation.z = quat_yaw.z();
    pose_a.pose.orientation.w = quat_yaw.w();
    pose_b.pose.orientation.x = quat_yaw.x();
    pose_b.pose.orientation.y = quat_yaw.y();
    pose_b.pose.orientation.z = quat_yaw.z();
    pose_b.pose.orientation.w = quat_yaw.w();
    // Service msg 赋值
    arm_cmd.request.value = true;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // 设置消息发送频率
    ros::Rate rate(10.0);
    // 等待飞行控制器链接
    while(ros::ok() && !current_state.connected){
        // 如果控制器没连接上，将一直处于循环里
        ROS_INFO("Waiting connection.");
        ros::spinOnce();
        rate.sleep();
    }

    // 开始之前先要进行期望位置的发送！！！
    for(int i = 50; ros::ok() && i > 0; --i){
        ROS_INFO("Sending msg before starting.");
        local_pos_pub.publish(pose_a);
        ros::spinOnce();
        rate.sleep();
    }
    last_request = ros::Time::now();

    bool Simulation = true;
    bool Shown = false;
    while(ros::ok()){
        // 下面这几行是针对仿真的
        // 因为仿真中没法解锁并切换模式
        if(Simulation == true)
        {
            if(!current_state.armed && // 如果还没有解锁，进进行解锁
               ros::Time::now() - last_request > ros::Duration(2.0))
            {
                if(arming_client.call(arm_cmd) &&
                   arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
            else
            {
                if(current_state.mode != "OFFBOARD" && //如果当前还不是offboard模式
                   ros::Time::now() - last_request > ros::Duration(2.0))
                {
                    // 设置为offboard模式
                    if(set_mode_client.call(offb_set_mode) &&
                        offb_set_mode.response.success)
                    {
                         ROS_INFO("Offboard enabled");
                    }
                    last_request = ros::Time::now();
                }
            }
        }

        // -----任务状态机----- //
        switch(current_pos_state)
        {
            case NONE:
                // 这样保证在解锁过程中一直有位置数据发送
                current_pos_state=TAKE_OFF;
                ROS_INFO("Start state machine!");
            break;
            case TAKE_OFF:
                if(Shown == false)
                {
                    ROS_INFO("Taking off!");
                    Shown = true;
                }
                local_pos_pub.publish(pose_a);
                if(current_pos.pose.position.z > 3)
                {
                    current_pos_state = POS_A;
                    Shown = false;
                }
            break;
            case POS_A:
                if(Shown == false)
                {
                    ROS_INFO("Flying to position A!");
                    Shown = true;
                }
                local_pos_pub.publish(pose_a);
                if(abs(current_pos.pose.position.x - pose_a.pose.position.x) < 0.1 && //到达A点后悬停
                   abs(current_pos.pose.position.y - pose_a.pose.position.y) < 0.1 &&
                   abs(current_pos.pose.position.z - pose_a.pose.position.z) < 0.1)
                {
                    current_pos_state = A_HOVER;
                    last_request = ros::Time::now();
                    Shown = false;
                }
            break;
            case A_HOVER:
                if(Shown == false)
                {
                    ROS_INFO("Hovering at position A!");
                    Shown = true;
                }
                local_pos_pub.publish(pose_a);
                if(ros::Time::now() - last_request > ros::Duration(5.0))
                {
                    current_pos_state = POS_B;
                    Shown = false;
                }
            break;
            case POS_B:
                if(Shown == false)
                {
                    ROS_INFO("Flying to position B!");
                    Shown = true;
                }
                local_pos_pub.publish(pose_b);
                if(abs(current_pos.pose.position.x - pose_b.pose.position.x) < 0.1 && //到达B点后悬停
                   abs(current_pos.pose.position.y - pose_b.pose.position.y) < 0.1 &&
                   abs(current_pos.pose.position.z - pose_b.pose.position.z) < 0.1)
                {
                    current_pos_state = B_HOVER;
                    last_request = ros::Time::now();
                    Shown = false;
                }
            break;
            case B_HOVER:
                if(Shown == false)
                {
                    ROS_INFO("Hovering at position B!");
                    Shown = true;
                }
                local_pos_pub.publish(pose_b);
                if(ros::Time::now() - last_request > ros::Duration(5.0))
                {
                    current_pos_state = POS_A_Re;
                    Shown = false;
                }
            break;
            case POS_A_Re:
                if(Shown == false)
                {
                    ROS_INFO("Going back to position A!");
                    Shown = true;
                }
                local_pos_pub.publish(pose_a);
                if(abs(current_pos.pose.position.x - pose_a.pose.position.x) < 0.1 && //回到A点后降落
                   abs(current_pos.pose.position.y - pose_a.pose.position.y) < 0.1 &&
                   abs(current_pos.pose.position.z - pose_a.pose.position.z) < 0.1)
                {
                    current_pos_state = LAND;
                    last_request = ros::Time::now();
                    Shown = false;
                }
            break;
            case LAND:
                if(current_state.mode == "OFFBOARD" &&
                   current_state.mode != "AUTO.LAND" &&
                   ros::Time::now() - last_request > ros::Duration(1.0))
                {
                    if(tol_client.call(landing_cmd) &&
                        landing_cmd.response.success)
                    {
                        if(Shown == false)
                        {
                            ROS_INFO("AUTO LANDING!");
                            Shown = true;
                        }
                        last_request = ros::Time::now();
                        Simulation = false; //防止仿真时再次解锁
                    }
                }
                else
                    local_pos_pub.publish(pose_a);
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
