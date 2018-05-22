#ifndef CLIENT_WORLDSTATE
#define CLIENT_WORLDSTATE

#include <ros/ros.h>


namespace client {
	
	
	class WorldStateClient {
		
		public:
			
			WorldStateClient()
			:	m_node()
			{
				if( initializeClient() ){
					ROS_INFO("Initialized the client.");
				}
				else{
					ROS_ERROR("Client initialization failed!");
				}
			}
			
			
			// actions that the planners and executors will use
			
		
		private:
		
			// internal communication with WorldState node
			
			
			bool initializeClient();
			
			ros::NodeHandle m_node;
			ros::Subscriber m_robot_state_sub;
			ros::ServiceClient m_update_robot_state_srv;
			ros::ServiceClient m_find_part_type_srv;
			ros::ServiceClient m_get_part_info_srv;
			ros::ServiceClient m_path_between_states_srv;
			ros::ServiceClient m_update_part_state_srv;
			ros::ServiceClient m_update_part_availability_srv;
		
	};
	
}



#endif
