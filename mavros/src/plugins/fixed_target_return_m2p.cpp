/**
 * @brief 	: FixedTargetPositionP2M plugin
 * @file 	: fixed_target_position_p2m.cpp
 * @author 	: libn
 * @time 	: 2016/09/12
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/FIXED_TARGET_RETURN_M2P.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief FixedTargetPositionM2P plugin.
 */
class FixedTargetReturnM2PPlugin : public plugin::PluginBase {
public:
	FixedTargetReturnM2PPlugin() : PluginBase(),
        nh("~")
    { }

    /**
     * Plugin initializer. Constructor should not do this.
     */
    void initialize(UAS &uas_)
    {
        PluginBase::initialize(uas_);

        nh.param<std::string>("frame_id", frame_id, "map");
//        mavros_msg_pub = nh.advertise<mavros_msgs::FIXED_TARGET_RETURN_M2P>("fixed_target_return_m2p", 10);
		mavros_msg_sub = nh.subscribe("fixed_target_return_m2p", 10, &FixedTargetReturnM2PPlugin::fixed_target_return_m2p_cb, this);
    }

    Subscriptions get_subscriptions()
    {
        return {
//            make_handler(&FixedTargetPositionP2MPlugin::handle_mavros_msg),
        };
    }

private:
    ros::NodeHandle nh;
    std::string frame_id;

//    ros::Publisher mavros_msg_pub;
	ros::Subscriber mavros_msg_sub;

	 void fixed_target_return_m2p_cb(const mavros_msgs::FIXED_TARGET_RETURN_M2P::ConstPtr &req)
	 {
	 	mavlink::pixhawk::msg::FIXED_TARGET_RETURN_M2P test_msg{};

//	 	test_msg.timestamp = ros::Time::now().toNSec() / 1000;

	 	test_msg.home_lat        	= req->home_lat;
	 	test_msg.home_lon        	= req->home_lon;
	 	test_msg.home_alt        	= req->home_alt;

	 	test_msg.observe_lat        = req->observe_lat;
	 	test_msg.observe_lon        = req->observe_lon;
	 	test_msg.observe_alt        = req->observe_alt;

	 	test_msg.spray_left_lat     = req->spray_left_lat;
	 	test_msg.spray_left_lon     = req->spray_left_lon;
	 	test_msg.spray_left_alt     = req->spray_left_alt;

	 	test_msg.spray_right_lat    = req->spray_right_lat;
	 	test_msg.spray_right_lon    = req->spray_right_lon;
	 	test_msg.spray_right_alt    = req->spray_right_alt;

	 	UAS_FCU(m_uas)->send_message_ignore_drop(test_msg);
	 }
};
}   // namespace std_plugins
}   // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::FixedTargetReturnM2PPlugin, mavros::plugin::PluginBase)
