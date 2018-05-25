#ifndef TASK_REMOVE_FAULTY_PART
#define TASK_REMOVE_FAULTY_PART

#include <ros/ros.h>
#include <task/Task.hpp>
#include <client/WorldStateClient.hpp>
#include <actionlib/server/simple_action_server.h>
#include <control_redesign/FillShipmentAction.h>


namespace task {
	
	
	class RemoveFaultyPartTask: public Task {
		
		public:
		
			RemoveFaultyPartTask(  )
			:	m_name("RemoveFaultyPartTask")
			
			{}
			
			bool execute() {
				ROS_INFO("Executing RemoveFaultyPartTask.");
				
				return true;
			}
			
			virtual ~RemoveFaultyPartTask() {}
		
		
		private:
			
			std::string m_name;

			
	};
	
}

#endif
