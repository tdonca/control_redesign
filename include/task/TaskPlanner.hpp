#ifndef TASK_PLANNER
#define TASK_PLANNER

#include <ros/ros.h>
#include <deque>
#include <functional>
#include <osrf_gear/Shipment.h>
#include <osrf_gear/Product.h>
#include <actionlib/server/simple_action_server.h>
#include <control_redesign/FillShipmentAction.h>
#include <client/WorldStateClient.hpp>
#include <task/TaskManager.hpp>


namespace task {
	
	class TaskPlanner {
		
		public:
		
			TaskPlanner()
			:	m_node(),
				m_shipment_as( m_node, "fill_shipment", false ),
				m_process_t(),
				m_task_manager( m_shipment_as ),
				m_current_shipment()
				
			{																					   //oneshot, autostart
				m_process_t = m_node.createTimer(ros::Duration(0), &TaskPlanner::cb_processParts, this, true, false);
				
				m_shipment_as.registerGoalCallback( std::bind( &TaskPlanner::acb_goalReceived, this ) );
				m_shipment_as.registerPreemptCallback( std::bind( &TaskPlanner::acb_preemptReceived, this ) );
				m_shipment_as.start();
				
				ROS_INFO("Task Planner initialized.");	
			}
			
			
			
		private:
			
			void acb_goalReceived();
			
			void acb_preemptReceived();
			
			void cb_processParts( const ros::TimerEvent & t );
			
			
			
			ros::NodeHandle m_node;
			actionlib::SimpleActionServer<control_redesign::FillShipmentAction> m_shipment_as;
			ros::Timer m_process_t;
	
			TaskManager m_task_manager;
			osrf_gear::Shipment m_current_shipment;
	};
	
	
}


#endif
