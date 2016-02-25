#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include "mavros_extras/Viconmocap.h"

namespace mavplugin{

class ViconPlugin : public MavRosPlugin{

public:
	ViconPlugin():
	vicon_nh("~vicon"),
	uas(nullptr)
    { };

    void initialize(UAS &uas_){
    	uas = &uas_;
        
    	vicon_nh.param<std::string>("frame_id", frame_id, "vicon");
        //subcribe the topic and excute the callback function
    	vicon_sub = vicon_nh.subscribe("/vicon",500,&ViconPlugin::vicon_cb,this);

    	ROS_INFO("initialize\n");

    }
    
    std::string get_name() {
		return "vicon";
    }


     const message_map get_rx_handlers() {
		return {
			     
		};
	}


private:
	ros::NodeHandle vicon_nh;
	ros::Subscriber vicon_sub;
	UAS *uas;

	std::string frame_id;

    void vicon_send(
		float x, float y, float z,
		float vx, float vy, float vz) {
    	mavlink_message_t vicon_msg;
        mavlink_msg_vicon_pack_chan(UAS_PACK_CHAN(uas), &vicon_msg,
				x,
				y,
				z,
				vx,
				vy,
				vz);
	UAS_FCU(uas)->send_message(&vicon_msg);
	ROS_INFO("send");
    }
    
    //callbacks
    void vicon_cb(const mavros_extras::Viconmocap &msg){
    	vicon_send(
    		msg.position.x,
    		msg.position.y,
    		msg.position.z,
    		msg.velocity.x,
    		msg.velocity.y,
    		msg.velocity.z);
    	//ROS_INFO("callback");
    }
};

};

PLUGINLIB_EXPORT_CLASS(mavplugin::ViconPlugin, mavplugin::MavRosPlugin)
