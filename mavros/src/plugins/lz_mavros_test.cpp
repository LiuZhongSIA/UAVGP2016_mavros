/**
 * @brief LzMavrosTest plugin
 * @file lz_mavros_test.cpp
 * @author Liu Zhong
 *
 * For exercise
 */

#include <mavros/mavros_plugin.h>
#include <mavros_msgs/LzMavrosTest.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief LzMavrosTest plugin.
 */
class LzMavrosTestPlugin : public plugin::PluginBase {
public:
    LzMavrosTestPlugin() : PluginBase(),
		nh("~")
	{ }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		nh.param<std::string>("frame_id", frame_id, "map");
            lz_mavros_test_sub = nh.subscribe("lz_mavros_test", 10, &LzMavrosTestPlugin::LzMavrosTestCallback, this);
    }

	Subscriptions get_subscriptions()
	{
		return {
            // nothing
		};
	}

private:
	ros::NodeHandle nh;
	std::string frame_id;
        ros::Subscriber lz_mavros_test_sub;

    void LzMavrosTestCallback(const mavros_msgs::LzMavrosTest::ConstPtr &req)
    {
        mavlink::pixhawk::msg::LZ_MAVROS_TEST test_msg{};

        test_msg.timestamp =  ros::Time::now().toNSec() / 1000;
        test_msg.test1 = req->test1;
        test_msg.test2 = req->test2;
        UAS_FCU(m_uas) -> send_message_ignore_drop(test_msg);
    }
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::LzMavrosTestPlugin, mavros::plugin::PluginBase)
