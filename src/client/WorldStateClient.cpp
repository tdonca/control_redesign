
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
				part_found.name = fpt_srv.response.part.name;
				part_found.type = fpt_srv.response.part.type;
				part_found.id = fpt_srv.response.part.id;
				part_found.current_pose = fpt_srv.response.part.current_pose;
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
	
	
	
	// ******* FREE ALL PARTS BEFORE ABORTING OR PREEMPTING A SHIPMENT *********
	bool WorldStateClient::markPartUsed( PlannerPart const & part ){
		
		control_redesign::MarkPartUsed mp_srv;
		mp_srv.request.name = part.name;
		
		if( m_mark_part_used_srv.call(mp_srv) ){
			if( mp_srv.response.success ){
				ROS_INFO("Successfully marked used %s", part.name.c_str());
				return true;
			}
			else{
				ROS_ERROR("Message: %s", mp_srv.response.message.c_str());
				return false;
			}
		}
		else{
			ROS_ERROR("Error calling MarkPartUsed service.");
			return false;
		}	
	}
	
	
	
	// ******* FREE ALL PARTS BEFORE ABORTING OR PREEMPTING A SHIPMENT *********
	bool WorldStateClient::releasePart( PlannerPart const & part ){
		
		control_redesign::ReleasePart rp_srv;
		rp_srv.request.name = part.name;
		
		if( m_release_part_srv.call(rp_srv) ){
			if( rp_srv.response.success ){
				ROS_INFO("Successfully released %s", part.name.c_str());
				return true;
			}
			else{
				ROS_ERROR("Message: %s", rp_srv.response.message.c_str());
				return false;
			}
		}
		else{
			ROS_ERROR("Error calling ReleasePart service.");
			return false;
		}		
	}
	
	
	bool WorldStateClient::getGripperPart( PlannerPart & part_found ){
		
		control_redesign::GetGripperPart gp_srv;
		
		if( m_gripper_part_srv.call(gp_srv) ){
			if( gp_srv.response.success ){
				ROS_INFO("Success: %s", gp_srv.response.message.c_str());
				part_found.name = gp_srv.response.part.name;
				part_found.type = gp_srv.response.part.type;
				part_found.id = gp_srv.response.part.id;
				part_found.current_pose = gp_srv.response.part.current_pose;
				return true;
			}
			else{
				ROS_ERROR("Message: %s", gp_srv.response.message.c_str());
				return false;
			}
		}
		else{
			ROS_ERROR("Error calling GetGripperPart service.");
			return false;
		}		
	}
	
	
			
	bool WorldStateClient::getBoxParts( std::vector<PlannerPart> & parts_found ){
		
		control_redesign::GetBoxParts bp_srv;
		
		if( m_box_parts_srv.call(bp_srv) ){
			if( bp_srv.response.success ){
				ROS_INFO("Success: %s", bp_srv.response.message.c_str());
				parts_found.clear();
				parts_found.resize( bp_srv.response.parts.size() );
				for( int i = 0; i < parts_found.size(); ++i ){
					parts_found[i].name = bp_srv.response.parts[i].name;
					parts_found[i].type = bp_srv.response.parts[i].type;
					parts_found[i].id = bp_srv.response.parts[i].id;
					parts_found[i].current_pose = bp_srv.response.parts[i].current_pose;
				}
				return true;
			}
			else{
				ROS_ERROR("Message: %s", bp_srv.response.message.c_str());
				return false;
			}
		}
		else{
			ROS_ERROR("Error calling GetBoxParts service.");
			return false;
		}	
	}
	
	
			
			
			
			
	
	void getRPY( const geometry_msgs::Pose  pose, double & r, double & p, double & y ){
		
		tf2::Quaternion arm_q;
		tf2::fromMsg( pose.orientation, arm_q );
		tf2::Matrix3x3(arm_q).getRPY( r, p, y );
		
	}
}
