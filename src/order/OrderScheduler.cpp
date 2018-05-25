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
				ROS_INFO("Shipment goal completed, moving to the next shipment.");
				m_shipment_q.pop_front();
			}
			else{
				ROS_ERROR("Shipment goal failed! Why???");
				ROS_INFO("Message: %s", result->message.c_str());
			}	
		}
		else{
			ROS_ERROR("Shipment Action did not succeed: %s", result->message.c_str());
			// deal with failure
			
			if( result->fail_reason == result->TRY_AGAIN ){
				ROS_ERROR("Retrying the current shipment...");
				control_redesign::FillShipmentGoal goal;
					goal.shipment = m_shipment_q.front();
					m_shipment_ac.sendGoal(goal, std::bind( &OrderScheduler::acb_shipmentFinished, this, std::placeholders::_1,
																										 std::placeholders::_2 ) );
			}
			
			else if( result->fail_reason == result->NOT_ENOUGH_PARTS ){
				ROS_ERROR("Not enough useable parts for the shipment! Moving on to the next shipment...");
				m_shipment_q.pop_front();
				if( m_shipment_q.size() > 0 ){
					control_redesign::FillShipmentGoal goal;
					goal.shipment = m_shipment_q.front();
					m_shipment_ac.sendGoal(goal, std::bind( &OrderScheduler::acb_shipmentFinished, this, std::placeholders::_1,
																										 std::placeholders::_2 ) );
				}	
			}
			
			else if( result->fail_reason == result->ROBOT_STUCK ){
				ROS_ERROR("Robot is not responsive anymore, quitting...");
			}
			
			else {
				ROS_ERROR("Invalid fail reason provided, quitting...");
			}
		}
	}
	
	
	
}
