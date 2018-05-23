#include <task/TaskPlanner.hpp>


namespace task {
	
	void TaskPlanner::acb_goalReceived(){
		
		m_current_shipment = m_shipment_as.acceptNewGoal()->shipment;
		ROS_INFO("New shipment received: %s", m_current_shipment.shipment_type.c_str());
		for( int i = 0; i < m_current_shipment.products.size(); ++i ){
			ROS_INFO("Part %d: %s", i, m_current_shipment.products[i].type.c_str());
		}
		ROS_INFO(" ");
		
		
		//~ ROS_INFO("Checking to see if all parts are avilable in the world...");
		//~ for( int i = 0; i < shipment.products.size(); ++i ){
			//~ client::PlannerPart p;
			//~ if( m_world_client.getPartType( shipment.products[i].type, p ) ){
				//~ ROS_INFO("Found part for order: %s", p.name.c_str());
				//~ m_parts_q.push_back(p);
			//~ }
			//~ else{
				//~ ROS_ERROR("Could not find any available %s type! Aborting the shipment...", shipment.products[i].type.c_str());
				//~ m_parts_q.clear();
				//~ control_redesign::FillShipmentResult result;
				//~ result.success = false;
				//~ result.message = "Aborting the shipment because there are not enough available parts.";
				//~ result.fail_reason = result.NOT_ENOUGH_PARTS;
				//~ m_shipment_as.setAborted(result);
				//~ return;
			//~ }	
		//~ }
		
		

		m_process_t.start();
	}
			
	void TaskPlanner::acb_preemptReceived(){
		
		ROS_ERROR("Preempt received for the current Shipment!");
		
		control_redesign::FillShipmentResult result;
		result.success = false;
		result.message = "Order Scheduler preempted the shipment.";
		result.fail_reason = result.TRY_AGAIN;
		m_shipment_as.setPreempted(result);
	}
	
	
	void TaskPlanner::cb_processParts( const ros::TimerEvent & t ){
		
		if( !m_shipment_as.isActive() ) return;
		
		
		m_process_t.stop();
		ROS_INFO("Create and do the first Task to find the parts and the order in which to fulfill.");
		m_tasks_q.push_back( m_task_manager.createFindPartsTask( m_current_shipment, m_parts_q ) );
		m_tasks_q.front()->execute();
		m_tasks_q.pop_front();
		
		
		
		// while there are parts left
		while( m_parts_q.size() > 0 ){
			ROS_INFO("Processing %s", m_parts_q.front().name.c_str());
			
			if( !m_shipment_as.isActive() ) return;
			
			
			
			// while there are task left
			while( m_tasks_q.size() > 0){
				
				if( !m_shipment_as.isActive() ) return;
				
				// execute the task
				m_tasks_q.front()->execute();
				
				// finished with the current task
				m_tasks_q.pop_front();
			}
			
			// finished with the current part
			ROS_INFO("Finished with %s", m_parts_q.front().name.c_str());
			m_parts_q.pop_front();
		}
		
		if( !m_shipment_as.isActive() ) return;
		
		
		ROS_INFO("Finished the shipment goal.");
		control_redesign::FillShipmentResult result;
		result.success = true;
		result.message = "Pretending: finished all parts.";
		m_shipment_as.setSucceeded(result);
	}
	
}
