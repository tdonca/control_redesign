
#include <client/WorldStateClient.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace client {
	
	
	
	bool WorldStateClient::getPartType( std::string type, PlannerPart & part_found ){
		
		// call find part type service  
		control_redesign::FindPartType fpt_srv;
		fpt_srv.request.type = type;
		
		if( m_find_part_type_srv.call(fpt_srv) ){
			if( fpt_srv.response.success ){
				//ROS_INFO("Message: %s", fpt_srv.response.message.c_str());
				part_found.name = fpt_srv.response.name;
				part_found.type = fpt_srv.response.type;
				part_found.id = fpt_srv.response.id;
				part_found.current_pose = fpt_srv.response.current_pose;
				return true;
			}
			else{
				ROS_ERROR("Message: %s", fpt_srv.response.message.c_str());
				return false;
			}
		}
		else{
			ROS_ERROR("Error calling FindPartType service.");
			return false;
		}
	}
	
	
	bool WorldStateClient::releasePart( PlannerPart const & part ){
		
		control_redesign::ReleasePart rp_srv;
		rp_srv.request.name = part.name;
		
		if( m_release_part_srv.call(rp_srv) ){
			if( rp_srv.response.success ){
				ROS_INFO("Successfully released %s", part.name.c_str());
				return true;
			}
			else{
				ROS_ERROR("Message %s", rp_srv.response.message.c_str());
				return false;
			}
		}
		else{
			ROS_ERROR("Error calling ReleasePart service.");
			return false;
		}
		return true;
	}
	
	
	
	
	void getRPY( const geometry_msgs::Pose  pose, double & r, double & p, double & y ){
		
		tf2::Quaternion arm_q;
		tf2::fromMsg( pose.orientation, arm_q );
		tf2::Matrix3x3(arm_q).getRPY( r, p, y );
		
	}
}
