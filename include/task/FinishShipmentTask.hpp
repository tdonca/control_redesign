#ifndef TASK_FINISH_SHIPMENT
#define TASK_FINISH_SHIPMENT

#include <ros/ros.h>
#include <task/Task.hpp>
#include <client/WorldStateClient.hpp>
#include <actionlib/server/simple_action_server.h>
#include <control_redesign/FillShipmentAction.h>


namespace task {
	
	
	class FinishShipmentTask: public Task {
		
		public:
		
			FinishShipmentTask( )
			:	m_name("FinishShipmentTask")
				
			{}
			
			bool execute() {
				ROS_INFO("Executing FinishShipmentTask.");
				
				return true;
			}
			
			virtual ~FinishShipmentTask() {}
		
		
		private:
			
			std::string m_name;
	
			
	};
	
}

#endif
