#include <task/TaskPlanner.hpp>


namespace task {
	
	void TaskPlanner::acb_goalReceived(){
		
		m_current_shipment = m_shipment_as.acceptNewGoal()->shipment;
		ROS_INFO("New shipment received: %s", m_current_shipment.shipment_type.c_str());
		for( int i = 0; i < m_current_shipment.products.size(); ++i ){
			ROS_INFO("Part %d: %s", i, m_current_shipment.products[i].type.c_str());
		}
		ROS_INFO(" ");
		
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
		
		m_process_t.stop();
		if( !m_shipment_as.isActive() ) return;
		ROS_INFO("Initialize Task Manager with the new shipment");
		m_task_manager.receiveNewShipment( m_current_shipment );
		
		
		// Execute all tasks created by the manager for the current shipment
		while( m_task_manager.hasNextTask() ){
			
			if( !m_shipment_as.isActive() ) return;
			m_task_manager.executeNextTask();
			if( !m_shipment_as.isActive() ) return;
		}  
	
		
		ROS_INFO("Finished the shipment goal.");
		control_redesign::FillShipmentResult result;
		result.success = true;
		result.message = "Pretending: finished all parts.";
		m_shipment_as.setSucceeded(result);
	}
	
}
