#ifndef TASK_ADD_PARTS
#define TASK_ADD_PARTS

#include <ros/ros.h>
#include <task/Task.hpp>
#include <client/WorldStateClient.hpp>
#include <actionlib/server/simple_action_server.h>
#include <control_redesign/FillShipmentAction.h>


namespace task {
	
	
	class AddPartTask: public Task {
		
		public:
		
			AddPartTask( client::PlannerPart const & part )
			:	m_name("add part"),
				m_part(part)
				
			{}
			
			
			bool execute() {
				ROS_INFO("Executing AddPartTask for %s.", m_part.name.c_str());
				
				return true;
			}
			
			virtual ~AddPartTask() {}
		
		
		private:
			
			std::string m_name;
			client::PlannerPart const & m_part;
		
			
	};
	
}

#endif
