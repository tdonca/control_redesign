#ifndef TASK_FIND_PARTS
#define TASK_FIND_PARTS

#include <ros/ros.h>
#include <task/Task.hpp>
#include <client/WorldStateClient.hpp>
#include <actionlib/server/simple_action_server.h>
#include <control_redesign/FillShipmentAction.h>


namespace task {
	
	
	class FindPartsTask: public Task {
		
		public:
		
			FindPartsTask( osrf_gear::Shipment const & shipment, std::deque<client::PlannerPart> & parts_queue, client::WorldStateClient & wc, 
											actionlib::SimpleActionServer<control_redesign::FillShipmentAction> & as )
			:	m_name("find parts"),
				m_shipment(shipment),
				m_parts_q(parts_queue),
				m_world_client(wc),
				m_shipment_as(as)
				
			{}
			
			bool execute() {
				ROS_INFO("Executing FindPartsTask.");
				
				ROS_INFO("Checking to see if all parts are avilable in the world...");
				for( int i = 0; i < m_shipment.products.size(); ++i ){
					
					client::PlannerPart p;
					if( m_world_client.getPartType( m_shipment.products[i].type, p ) ){
						ROS_INFO("Found part for order: %s", p.name.c_str());
						m_parts_q.push_back(p);
					}
					else{
						ROS_ERROR("Could not find any available %s type! Aborting the shipment...", m_shipment.products[i].type.c_str());
						
						for( int j = 0; j < m_parts_q.size(); ++j ){
							m_world_client.releasePart( m_parts_q[j] );
						}
						m_parts_q.clear();
						
						control_redesign::FillShipmentResult result;
						result.success = false;
						result.message = "Aborting the shipment because there are not enough available parts.";
						result.fail_reason = result.NOT_ENOUGH_PARTS;
						
						m_shipment_as.setAborted(result);
						return false;
					}	
				}	
				
				return true;
			}
			
			virtual ~FindPartsTask() {}
		
		
		private:
			
			std::string m_name;
			osrf_gear::Shipment const & m_shipment;
			std::deque<client::PlannerPart> & m_parts_q;
			client::WorldStateClient & m_world_client;
			actionlib::SimpleActionServer<control_redesign::FillShipmentAction> & m_shipment_as;
			
			
	};
	
}

#endif
