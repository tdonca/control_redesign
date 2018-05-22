#include <order/OrderScheduler.hpp>


namespace order {
	
	void OrderScheduler::cb_newOrder( const osrf_gear::Order::ConstPtr & msg ){
		
		// regular oder
		if( msg->order_id == "order_0" ){
			
			ROS_INFO("Add new shipments from regular order.");
			
			// queue each shipment in order
			for( int i = 0; i < msg->shipments.size(); ++i ){
				m_shipment_q.push_back( msg->shipments[i] );
			}
			
			// send first shipment action goal
			control_redesign::FillShipmentGoal goal;
			goal.shipment = m_shipment_q.front();
			m_shipment_ac.sendGoal(goal, std::bind( &OrderScheduler::acb_shipmentFinished, this, std::placeholders::_1,
																								 std::placeholders::_2 ) );
			ROS_INFO("Shipment goal sent to Task Planner.");
		}
		
		
		// order update
		else if ( msg->order_id == "order_0_update_0" ){
			
			// remove shipments from queue
			m_shipment_q.clear();
			
			// queue each shipment in order
			for( int i = 0; i < msg->shipments.size(); ++i ){
				m_shipment_q.push_back( msg->shipments[i] );
			}
			
			//TODO: cancel current order fulfillment
			ROS_INFO("Cancel current shipment for order update!");
		}
		
		
		// emergency order
		else if ( msg->order_id == "order_1" ){
			
			// queue emergency shipments at the front
			for( int i = msg->shipments.size()-1; i >= 0; --i ){
				m_shipment_q.push_front( msg->shipments[i] );
			}
			
			//TODO: cancel current order fulfillment
			ROS_INFO("Cancel current shipment for emergency order!");
			
		}
		
		
		else {
			ROS_ERROR("Unusual order ID !!!");
			ROS_ERROR("Unusual order ID !!!");
			ROS_ERROR("Unusual order ID !!!");
		}
	
	
		// Print current queue
		for( int i = 0; i < m_shipment_q.size(); ++i ){
			ROS_INFO("Shipment %d: %s", i, m_shipment_q[i].shipment_type.c_str());
			for( int j = 0; j < m_shipment_q[i].products.size(); ++j ){
				ROS_INFO("Part %d: %s", j, m_shipment_q[i].products[j].type.c_str());
			}
			ROS_INFO(" ");
		}
	
	}
	
	
	
	
	void OrderScheduler::acb_shipmentFinished( const actionlib::SimpleClientGoalState & state, const control_redesign::FillShipmentResultConstPtr & result ){
		
		if( state == actionlib::SimpleClientGoalState::SUCCEEDED ){
			
			// if success remove shipment from queue
			if( result->success ){
				ROS_INFO("Shipment goal completed.");
			}
			else{
				ROS_ERROR("Shipment goal failed!");
			}
			ROS_INFO("Message: %s", result->message.c_str());
		}
		else{
			
			ROS_ERROR("Shipment Action did not succeed!");
		}
		
		// use msg to find out why and if to retry
		
		// does result still get filled if ABORTED?	
	}
	
	
	
}
