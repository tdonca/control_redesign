#ifndef TASK_PUT_PARTS_BACK
#define TASK_PUT_PARTS_BACK

#include <ros/ros.h>
#include <task/Task.hpp>
#include <client/WorldStateClient.hpp>
#include <actionlib/server/simple_action_server.h>
#include <control_redesign/FillShipmentAction.h>


namespace task {
	
	
	class PutPartsBackTask: public Task {
		
		public:
		
			PutPartsBackTask( std::vector<client::PlannerPart> const box_parts )
			:	m_name("put parts back"),
				m_parts(box_parts)
				
			{}
			
			bool execute() {
				ROS_INFO("Executing PutPartsBackTask.");
				
				return true;
			}
			
			virtual ~PutPartsBackTask() {}
		
		
		private:
			
			std::string m_name;
			std::vector<client::PlannerPart> m_parts;
			
	};
	
}

#endif
