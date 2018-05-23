#ifndef TASK_MANAGER
#define TASK_MANAGER

#include <ros/ros.h>
#include <string>
#include <memory>
#include <task/Task.hpp>
#include <task/FindPartsTask.hpp>
#include <client/WorldStateClient.hpp>
#include <actionlib/server/simple_action_server.h>
#include <control_redesign/FillShipmentAction.h>


namespace task {
	
	
	class TaskManager {
		
		public:
		
			TaskManager( client::WorldStateClient & wc, actionlib::SimpleActionServer<control_redesign::FillShipmentAction> & as )
			:	m_tasks(),
				m_world_client(wc),
				m_shipment_as(as)
			{}
			
			Task * createFindPartsTask( osrf_gear::Shipment const & current_shipment, std::deque<client::PlannerPart> & parts_queue ) {
				std::unique_ptr<Task> t = std::unique_ptr<Task>( new FindPartsTask( current_shipment, parts_queue, m_world_client, m_shipment_as ) );
				m_tasks.push_back(std::move(t));
				ROS_INFO("Created a FindPartsTask.");
				return m_tasks.back().get();
			} 
		
		
		private:
		
			std::vector<std::unique_ptr<Task> > m_tasks;
			client::WorldStateClient & m_world_client;
			actionlib::SimpleActionServer<control_redesign::FillShipmentAction> & m_shipment_as;
			
	};
	
}

#endif
