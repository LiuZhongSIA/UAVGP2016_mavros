/**
 * @brief Attitude plugin
 * @file attitude.cpp
 * @author Andreas Antener <andreas@uaventure.com>
 * @author huang li long <huanglilongwk@outlook.com>
 * @time 2016/07/05
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2015 Andreas Antener <andreas@uaventure.com>.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <mavros_msgs/Attitude.h>

namespace mavplugin {
/**
 * @brief Attitude plugin.
 */
class AttitudePlugin : public MavRosPlugin {
public:
    AttitudePlugin() :
        nh("~"),
        uas(nullptr)
    { }

    /**
     * Plugin initializer. Constructor should not do this.
     */
    void initialize(UAS &uas_)
    {
        uas = &uas_;
        nh.param<std::string>("frame_id", frame_id, "map");
        attitude_pub = nh.advertise<mavros_msgs::Attitude>("attitude", 10);
    }

    const message_map get_rx_handlers() {
        return {
                   MESSAGE_HANDLER(MAVLINK_MSG_ID_ATTITUDE, &AttitudePlugin::handle_attitude),
        };
    }

private:
    ros::NodeHandle nh;
    UAS *uas;
    std::string frame_id;

    ros::Publisher attitude_pub;

    void handle_attitude(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
        mavlink_attitude_t attitude;
        mavlink_msg_attitude_decode(msg, &attitude);

        auto ros_msg = boost::make_shared<mavros_msgs::Attitude>();
        ros_msg->header = uas->synchronized_header(frame_id, attitude.time_boot_ms);
        
        ros_msg->roll        = attitude.roll;
        ros_msg->pitch       = attitude.pitch;
        ros_msg->yaw         = attitude.yaw;
        ros_msg->rollspeed   = attitude.rollspeed;
        ros_msg->pitchspeed  = attitude.pitchspeed;
        ros_msg->yawspeed    = attitude.yawspeed;

        attitude_pub.publish(ros_msg);
    }

};
};  // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::AttitudePlugin, mavplugin::MavRosPlugin)

