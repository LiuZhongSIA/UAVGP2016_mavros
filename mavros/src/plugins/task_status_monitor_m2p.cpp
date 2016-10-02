/**
* @file     : task_status_monitor_m2p.cpp
* @brief    : task_status_monitor_m2p plugin.
* @author   : libn
* @time     : Oct 2, 2016 8:20:31 AM
*/

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/TASK_STATUS_MONITOR_M2P.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief TaskStatusMonitor plugin.
 */
class TASK_STATUS_MONITOR_M2PPlugin : public plugin::PluginBase {
public:
	TASK_STATUS_MONITOR_M2PPlugin() : PluginBase(),
        nh("~")
    { }

    /**
     * Plugin initializer. Constructor should not do this.
     */
    void initialize(UAS &uas_)
    {
        PluginBase::initialize(uas_);

        nh.param<std::string>("frame_id", frame_id, "map");
        //mavros_msg_pub = nh.advertise<mavros_msgs::TaskStatusMonitor>("task_status_monitor", 10);
		mavros_msg_sub = nh.subscribe("task_status_monitor_m2p", 10, &TASK_STATUS_MONITOR_M2PPlugin::task_status_monitor_m2p_cb, this);
    }

    Subscriptions get_subscriptions()
    {
        return {
            //make_handler(&TaskStatusMonitorPlugin::handle_mavros_msg),
        };
    }

private:
    ros::NodeHandle nh;
    std::string frame_id;

    //ros::Publisher mavros_msg_pub;
	ros::Subscriber mavros_msg_sub;

    // void handle_mavros_msg(const mavlink::mavlink_message_t *msg, mavlink::pixhawk::msg::TASK_STATUS_MONITOR &task_status_monitor)
    // {
    //     auto ros_msg = boost::make_shared<mavros_msgs::TaskStatusMonitor>();

    //     ros_msg->header = m_uas->synchronized_header(frame_id, task_status_monitor.timestamp);

	// 	ros_msg->num_odd_even 	= task_status_monitor.num_odd_even;
	// 	ros_msg->task_status 	= task_status_monitor.task_status;
	// 	ros_msg->loop_value 	= task_status_monitor.loop_value;
	// 	ros_msg->target_lat 	= task_status_monitor.target_lat;
	// 	ros_msg->target_lon 	= task_status_monitor.target_lon;
	// 	ros_msg->target_alt 	= task_status_monitor.target_alt;

    //     mavros_msg_pub.publish(ros_msg);
    // }
	void task_status_monitor_m2p_cb(const mavros_msgs::TASK_STATUS_MONITOR_M2P::ConstPtr &req) {
		mavlink::pixhawk::msg::TASK_STATUS_MONITOR_M2P test_msg{};

//		test_msg.timestamp = ros::Time::now().toNSec() / 1000;

		test_msg.num_odd_even 	= req->num_odd_even;
		test_msg.task_status 	= req->task_status;
		test_msg.loop_value 	= req->loop_value;
		test_msg.target_lat 	= req->target_lat;
		test_msg.target_lon 	= req->target_lon;
		test_msg.target_alt 	= req->target_alt;

		UAS_FCU(m_uas)->send_message_ignore_drop(test_msg);
	}
};
}   // namespace std_plugins
}   // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::TASK_STATUS_MONITOR_M2PPlugin, mavros::plugin::PluginBase)
