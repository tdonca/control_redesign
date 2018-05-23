#ifndef CLIENT_WORLDSTATE
#define CLIENT_WORLDSTATE

#include <ros/ros.h>
#include <control_redesign/FindPartType.h>
#include <control_redesign/ReleasePart.h>


namespace client {
	
	struct PlannerPart;
	void getRPY( const geometry_msgs::Pose  pose, double & r, double & p, double & y );
	
	
	class WorldStateClient {
		
		public:
			
			WorldStateClient()
			:	m_node(),
				m_find_part_type_srv(),
				m_release_part_srv()
			
			{
				m_find_part_type_srv = m_node.serviceClient<control_redesign::FindPartType>("find_part_type");
				m_release_part_srv = m_node.serviceClient<control_redesign::ReleasePart>("release_part");
			}
			
			
			// actions that the planners and executors will use
			
			bool getPartType( std::string type, PlannerPart & part_found );
			
			bool releasePart( PlannerPart const & part );
		
		private:
		
			// internal communication with WorldState node
			
			
			
			ros::NodeHandle m_node;
			ros::ServiceClient m_find_part_type_srv;
			ros::ServiceClient m_release_part_srv;
		
	};
	
	
	
	struct PlannerPart{
		
		std::string name;
		std::string type;
		std::string id;
		geometry_msgs::Pose current_pose;
		geometry_msgs::Pose goal_pose;
		
		void printPart(){
			double r, p, y;
			getRPY(current_pose, r, p, y );
			ROS_INFO("Name: %s", name.c_str());
			ROS_INFO("Type: %s", type.c_str());
			ROS_INFO("Position: %.3f, %.3f, %.3f",current_pose.position.x,current_pose.position.y,current_pose.position.z);
			ROS_INFO("Orientation: %.3f, %.3f, %.3f, %.3f",current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w);
			ROS_INFO("RollPitchYaw: %.3f, %.3f, %.3f", r, p, y);
			ROS_INFO(" ");
		}
	};
}



#endif
