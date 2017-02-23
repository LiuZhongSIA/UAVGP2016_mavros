/**
* @file       : missionAB_node.cpp
* @brief      : take off --> pos A --> pos B --> land
* @author     : huang li long <huanglilongwk@outlook.com>
* @annotation : liu zhong
* @time       : 2016/07/27
*/

#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeRad.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros/frame_tf.h>
void state_machine(void); //声明状态机函数

// 定义变量，代表状态机状态
static const int POS_A = 0;
static const int POS_B = 1;
static const int LAND  = 2;
// 当前的位置状态，在点A上
int current_pos_state = POS_A;

// 设置A、B点，msg，将通过ros发送出去
geometry_msgs::PoseStamped pose_a;
geometry_msgs::PoseStamped pose_b;
ros::Publisher local_pos_pub;

// 订阅“状态”消息，以及相应回调函数
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
// 订阅“位置消息”，以及相应的回调函数
geometry_msgs::PoseStamped current_pos;
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pos = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "missionAB_node"); //节点名称
    ros::NodeHandle nh; //ros节点句柄

    // 配置回调函数，得到飞机的状态信息
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    // 配置回调函数，得到飞机的位置信息
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pos_cb);
    // 要向飞机发布位置指令
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    // 配置ros service，用于飞机解锁
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    // 配置ros service，用于改变模式
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    // 配置ros service，用于降落
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");

    // 设置消息发送频率
    ros::Rate rate(20.0);
    // 等待飞行控制器链接
    while(ros::ok() && !current_state.connected){
        // 如果控制器没连接上，将一直处于循环里
        ROS_INFO("Waiting connection.");
        ros::spinOnce();
        rate.sleep();
    }

    // A点位置
    pose_a.pose.position.x = 0;
    pose_a.pose.position.y = 0;
    pose_a.pose.position.z = 5;
    // B点位置
    pose_b.pose.position.x = 0;
    pose_b.pose.position.y = 5;
    pose_b.pose.position.z = 5;
    // 姿态期望
    auto quat_yaw = mavros::ftf::quaternion_from_rpy(0.0, 0.0, 0.0);
    pose_a.pose.orientation.x = quat_yaw.x();
    pose_a.pose.orientation.y = quat_yaw.y();
    pose_a.pose.orientation.z = quat_yaw.z();
    pose_a.pose.orientation.w = quat_yaw.w();
    pose_b.pose.orientation.x = quat_yaw.x();
    pose_b.pose.orientation.y = quat_yaw.y();
    pose_b.pose.orientation.z = quat_yaw.z();
    pose_b.pose.orientation.w = quat_yaw.w();
    // 开始之前先要进行期望位置的发送！！！
    for(int i = 100; ros::ok() && i > 0; --i){
        ROS_INFO("Sending msg before starting.");
        local_pos_pub.publish(pose_a);
        ros::spinOnce();
        rate.sleep();
    }

    // 定义模式msg，写入offboard模式
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    // 定义解锁msg，写入解锁
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    // 定义降落msg
    mavros_msgs::CommandTOL landing_cmd;
    landing_cmd.request.min_pitch = 1.0;
    // 时间戳
    ros::Time last_request = ros::Time::now();
    ros::Time landing_last_request = ros::Time::now();

    while(ros::ok()){
        // 下面这几行是针对仿真的
        // 因为仿真中没法解锁并切换模式
        if(current_state.mode != "OFFBOARD" && //如果当前还不是offboard模式
           (ros::Time::now() - last_request > ros::Duration(5.0)))
         {
            // 设置为offboard模式
            if(set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success)
             {
                 ROS_INFO("Offboard enabled");
             }
             last_request = ros::Time::now();
         } else
         {
             if(!current_state.armed &&      // 如果还没有解锁，进进行解锁
               (ros::Time::now() - last_request > ros::Duration(5.0)))
               {
                 if(arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
                 {
                     ROS_INFO("Vehicle armed");
                 }
                 last_request = ros::Time::now();
             }
         }

        // 如果是飞真实飞机，可以只运行下面这部分
        // 没解锁的情况下，模式状态在A点
        if(!current_state.armed)
        {
            current_pos_state = POS_A;
        }
        // 自动起飞显示，仅显示
        if( current_state.mode == "AUTO.TAKEOFF"){
            ROS_INFO("AUTO TAKEOFF!");
        }
        // 如果状态机要求降落，且当前不是自动降落，且上一起降落要求在
        // 发送降落指令
        if((current_pos_state == LAND) && (current_state.mode == "OFFBOARD")){
            if( current_state.mode != "AUTO.LAND" &&
                (ros::Time::now() - landing_last_request > ros::Duration(5.0))){
                if(land_client.call(landing_cmd) &&
                    landing_cmd.response.success){
                    ROS_INFO("AUTO LANDING!");
                }
                landing_last_request = ros::Time::now();
            }
        }
        state_machine();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

// 任务状态机
void state_machine(void){
    switch(current_pos_state){
        case POS_A:
            local_pos_pub.publish(pose_a); //发布A点位置信息
            if((abs(current_pos.pose.position.x - pose_a.pose.position.x) < 0.1) && //到达A点后切换到下一点
               (abs(current_pos.pose.position.y - pose_a.pose.position.y) < 0.1) &&
               (abs(current_pos.pose.position.z - pose_a.pose.position.z) < 0.1))
               {
                    current_pos_state = POS_B;
               }
            break;
        case POS_B:
            local_pos_pub.publish(pose_b); //发布B点位置信息
            if((abs(current_pos.pose.position.x - pose_b.pose.position.x) < 0.1) && //到达B点后准备降落
               (abs(current_pos.pose.position.y - pose_b.pose.position.y) < 0.1) &&
               (abs(current_pos.pose.position.z - pose_b.pose.position.z) < 0.1))
               {
                    current_pos_state = LAND;
               }
            break;
        case LAND:
            break;
    }
}
