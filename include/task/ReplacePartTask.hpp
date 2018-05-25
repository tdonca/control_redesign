#ifndef TASK_REPLACE_PART
#define TASK_REPLACE_PART

#include <ros/ros.h>
#include <task/Task.hpp>
#include <client/WorldStateClient.hpp>
#include <actionlib/server/simple_action_server.h>
#include <control_redesign/FillShipmentAction.h>


namespace task {
	
	
	class ReplacePartTask: public Task {
		
		public:
		
			ReplacePartTask()
			:	m_name("Replace Part")
				
			{}
			
			bool execute() {
				ROS_INFO("Executing ReplacePartTask.");
				
				// find a new part to replace the removed faulty part	
				
				// create next task
				
				return true;
			}
			
			virtual ~ReplacePartTask() {}
		
		
		private:
			
			std::string m_name;
			
			
	};
	
}

#endif
