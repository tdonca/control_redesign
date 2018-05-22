#include <task/TaskPlanner.hpp>


namespace task {
	
	void TaskPlanner::acb_goalReceived(){
		
		osrf_gear::Shipment shipment = m_shipment_as.acceptNewGoal()->shipment;
		ROS_INFO("New shipment received: %s", shipment.shipment_type.c_str());
		for( int i = 0; i < shipment.products.size(); ++i ){
			ROS_INFO("Part %d: %s", i, shipment.products[i].type.c_str());
		}
		ROS_INFO(" ");
		
		
		ROS_INFO("Checking to see if all parts are avilable in the world...");
		// use WorldStateClient to find a part for each type requested
		// create a Planner representation of Parts
		
		ROS_INFO("Storing found parts in the queue.");
		// add the found PlannerParts to the queue
		
		
		// start timer (oneshot) to call process callback
		m_process_t.start();
	}
			
	void TaskPlanner::acb_preemptReceived(){
		
		ROS_ERROR("Preempt received for the current Shipment!");
		
		control_redesign::FillShipmentResult result;
		result.success = false;
		result.message = "Order Scheduler preempted the shipment.";
		m_shipment_as.setPreempted();
	}
	
	
	void TaskPlanner::cb_processParts( const ros::TimerEvent & t ){
		
		m_process_t.stop();
		ROS_INFO("Pretend to process the whole shipment instantly.");
		
		control_redesign::FillShipmentResult result;
		result.success = true;
		result.message = "Just pretending for now.";
		m_shipment_as.setSucceeded(result);
	}
	
}
