/**
 * @brief AttitudeRad plugin
 * @file attitude_rad.cpp
 * @author Liu Zhong
 *
 * For exercise
 */

#include <mavros/mavros_plugin.h>
#include <mavros_msgs/AttitudeRad.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief AttitudeRad plugin.
 */
class AttitudeRadPlugin : public plugin::PluginBase {
public:
    AttitudeRadPlugin() : PluginBase(),
		nh("~")
	{ }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		nh.param<std::string>("frame_id", frame_id, "map");
        attitude_rad_pub = nh.advertise<mavros_msgs::AttitudeRad>("attitude_rad", 10);
	}

	Subscriptions get_subscriptions()
	{
		return {
            make_handler(&AttitudeRadPlugin::handle_attitude_rad),
		};
	}

private:
	ros::NodeHandle nh;
	std::string frame_id;

    ros::Publisher attitude_rad_pub;

    void handle_attitude_rad(const mavlink::mavlink_message_t *msg, mavlink::common::msg::ATTITUDE &attitude)
	{
        auto ros_msg = boost::make_shared<mavros_msgs::AttitudeRad>();
        ros_msg->header = m_uas->synchronized_header(frame_id, attitude.time_boot_ms);

        ros_msg->roll = attitude.roll;
        ros_msg->pitch = attitude.pitch;
        ros_msg->yaw = attitude.yaw;
        ros_msg->rollspeed = attitude.rollspeed;
        ros_msg->pitchspeed = attitude.pitchspeed;
        ros_msg->yawspeed = attitude.yawspeed;

        attitude_rad_pub.publish(ros_msg);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::AttitudeRadPlugin, mavros::plugin::PluginBase)
