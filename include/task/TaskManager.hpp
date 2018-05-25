#ifndef TASK_MANAGER
#define TASK_MANAGER

#include <ros/ros.h>
#include <string>
#include <memory>
#include <task/Task.hpp>
#include <task/AddPartTask.hpp>
#include <task/PutPartsBackTask.hpp>
#include <task/RemoveFaultyPartTask.hpp>
#include <task/ReplacePartTask.hpp>
#include <task/FinishShipmentTask.hpp>
#include <client/WorldStateClient.hpp>
#include <actionlib/server/simple_action_server.h>
#include <control_redesign/FillShipmentAction.h>


namespace task {
	
	
	class TaskManager {
		
		public:
		
			TaskManager( actionlib::SimpleActionServer<control_redesign::FillShipmentAction> & as )
			:	m_world_client(),
				m_tasks_q(),
				m_parts_q(),
				m_shipment_as(as)
			{}
			
			
			bool hasNextTask() const;
			
			void executeNextTask();
			
			void receiveNewShipment( const osrf_gear::Shipment & shipment );
			
			void createAddPartTask( client::PlannerPart const & part );
			
			void createPutPartsBackTask( std::vector<client::PlannerPart> const box_parts );
			
			void createRemoveFaultyPartTask();
			
			void createReplacePartTask();
			
			void createFinishShipmentTask();
		



		private:
		
			client::WorldStateClient m_world_client;
			std::deque< std::unique_ptr<Task> > m_tasks_q;
			std::deque<client::PlannerPart> m_parts_q;
			
			actionlib::SimpleActionServer<control_redesign::FillShipmentAction> & m_shipment_as;
			
	};
	
}

#endif
